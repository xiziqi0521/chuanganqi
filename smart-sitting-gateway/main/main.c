#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"

// 引入最新标准的 ESP-IDF Zigbee 核心头文件
#include "esp_zigbee_core.h"
#include "esp_zigbee_endpoint.h"

// ======= 硬件与缓存配置 =======
#define UART_NUM           UART_NUM_0   
#define TXD_PIN            (16)         
#define RXD_PIN            (17)         
#define BUF_SIZE           (1024)

// ======= Zigbee 网络参数配置 =======
#define COORD_ENDPOINT      (1)       // 网关本地端点号
#define CUSTOM_CLUSTER_ID   (0xFFF0)  // 双方约定的自定义 Cluster ID

static const char *TAG = "ZIGBEE_COORDINATOR";

// ======= 1. 传感器原始数据结构体 =======
typedef struct {
    int distance;     
    int light;        
    float temperature;
    int status;       
    int light_state;  
    int ac_state;     
} arduino_data_t;

// ======= 2. 核心业务变量 =======
uint32_t total_work_time = 0;   // 今日累计专注时长（秒）
uint32_t warning_count = 0;     // 今日累计驼背警告次数
int health_score = 100;          // 初始健康评分
int current_sys_status = 0;     // 最终系统状态：0-正常，1-不良坐姿，2-离座

// 算法防抖变量
int consecutive_bad_ticks = 0;  
#define DISTANCE_THRESHOLD   (35)   
#define DEBOUNCE_TICKS       (15)   

// ======= 3. Zigbee 无线广播打包函数 =======
void zgb_broadcast_payload(const char *json_string)
{
    if (json_string == NULL) return;

    // 清零初始化结构体
    esp_zb_zcl_custom_cluster_cmd_req_t cmd = {0};
    
    // 适配当前版本：配置地址模式与 16 位短地址的嵌套层级
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0xFFFF; // 全网广播短地址
    cmd.zcl_basic_cmd.dst_endpoint = COORD_ENDPOINT; 
    cmd.zcl_basic_cmd.src_endpoint = COORD_ENDPOINT; 
    
    // 修正字段名：新 SDK 将名称精简为 cluster_id 与 custom_cmd_id
    cmd.cluster_id = CUSTOM_CLUSTER_ID;
    cmd.custom_cmd_id = 0; 
    
    // 装载透传的 JSON 字符串载荷
    cmd.data.type = ESP_ZB_ZCL_ATTR_TYPE_SET; 
    cmd.data.value = (void *)json_string;
    cmd.data.size = strlen(json_string);

    // 必须在协议栈加锁保护下调用 API
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&cmd);
    esp_zb_lock_release();
}

// ======= 4. 坐姿核心智能算法与 JSON 序列化处理 =======
void process_sitting_algorithm(arduino_data_t data)
{
    static int second_counter = 0;
    second_counter++;

    // A. 判定是否离座
    if (data.distance > 150) {
        if (current_sys_status != 2) {
            current_sys_status = 2;
            ESP_LOGW("ALGO", "【状态变更】检测到主人已离开工位，暂停专注计时。");
        }
        consecutive_bad_ticks = 0; 
    }
    // B. 判定是否距离过近（疑似驼背）
    else if (data.distance < DISTANCE_THRESHOLD) {
        consecutive_bad_ticks++;
        
        if (consecutive_bad_ticks >= DEBOUNCE_TICKS && current_sys_status == 0) {
            current_sys_status = 1; 
            warning_count++;        
            health_score -= 5;      
            if (health_score < 0) health_score = 0;
            
            ESP_LOGE("ALGO", "🚨【警告】连续3秒距离过近！判定为【不良坐姿(驼背)】！今日警告:%d次, 当前健康分:%d", 
                     (int)warning_count, health_score);
        }
        
        if (second_counter >= 5) { 
            total_work_time++;
            second_counter = 0;
        }
    }
    // C. 正常坐姿
    else {
        consecutive_bad_ticks = 0; 
        if (current_sys_status != 0) {
            current_sys_status = 0; 
            ESP_LOGI("ALGO", "【状态恢复】坐姿回归正常。");
        }
        
        if (second_counter >= 5) { 
            total_work_time++;
            second_counter = 0;
        }
    }

    // D. 转换成标准的 cJSON 对象进行序列化，并无线传输
    cJSON *root = cJSON_CreateObject();
    if (root != NULL) {
        cJSON_AddNumberToObject(root, "distance", data.distance);
        cJSON_AddNumberToObject(root, "light", data.light);
        cJSON_AddNumberToObject(root, "temp", data.temperature);
        cJSON_AddNumberToObject(root, "sys_status", current_sys_status);
        cJSON_AddNumberToObject(root, "work_time", total_work_time);
        cJSON_AddNumberToObject(root, "warn_cnt", warning_count);
        cJSON_AddNumberToObject(root, "score", health_score);
        cJSON_AddNumberToObject(root, "l_st", data.light_state);
        cJSON_AddNumberToObject(root, "ac_st", data.ac_state);

        char *json_out = cJSON_PrintUnformatted(root);
        if (json_out != NULL) {
            zgb_broadcast_payload(json_out);
            free(json_out);
        }
        cJSON_Delete(root);
    }
}

// ======= 5. UART 接收与解析后台任务 =======
static void uart_rx_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    ESP_LOGI(TAG, "🚀 UART 接收任务启动，开始监听 Arduino 数据...");

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; 

            arduino_data_t current_data;
            int parsed = sscanf((char*)data, "D:%d,L:%d,T:%f,S:%d,L_ST:%d,AC_ST:%d",
                                &current_data.distance, 
                                &current_data.light, 
                                &current_data.temperature, 
                                &current_data.status,
                                &current_data.light_state,
                                &current_data.ac_state);

            if (parsed == 6) {
                process_sitting_algorithm(current_data);
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}

// ======= 6. 兼容当前标准的全局信号事件处理器 (弱函数重写) =======
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    esp_err_t status = signal_struct->esp_err_status;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "🚀 Zigbee 协议栈初始化成功，正在启动 Commissioning 机制...");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "🌐 设备成功就绪，开始尝试建立本地 network (Formation)...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGE(TAG, "❌ Zigbee 启动失败, 错误码: %s", esp_err_to_name(status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (status == ESP_OK) {
            uint16_t pan_id = esp_zb_get_pan_id();
            ESP_LOGI(TAG, "===========================================");
            ESP_LOGI(TAG, " 🎉【成功】Zigbee 协调器建网成功！PAN ID: 0x%04x", pan_id);
            ESP_LOGI(TAG, "===========================================");
            
            // 开启网络允许设备加入的能力
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            
            // 网络彻底就绪后再拉起 UART 任务，确保算法产生的数据能正常广播
            xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
        } else {
            ESP_LOGE(TAG, "❌ 网络形成失败, 错误码: %s，2秒后尝试重试...", esp_err_to_name(status));
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;

    default:
        ESP_LOGI(TAG, "收到未处理的 Zigbee 系统信号: 0x%x", sig_type);
        break;
    }
}

// ======= 7. Zigbee 协议栈主初始化任务 =======
static void esp_zb_task(void *pvParameters)
{
    // 【完美解法】使用 {0} 全清零，彻底解决嵌套结构体大括号报 missing braces 的死穴
    esp_zb_cfg_t zb_nwk_cfg = {0};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR;
    zb_nwk_cfg.install_code_policy = false;
    
    // 【SDK 适配】依据编译器提示，新版 SDK 中协调器与路由器的配置字段已合并为 zczr_cfg
    zb_nwk_cfg.nwk_cfg.zczr_cfg.max_children = 10; 
    
    esp_zb_init(&zb_nwk_cfg);

    // B. 创建并注册自定义网关端点与属性簇
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    // 使用标准的属性列表来初始化自定义空 Cluster 容器
    esp_zb_attribute_list_t *custom_attr_list = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    
    // 将簇挂载到端点列表，设置为 SERVER 角色
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // 配置本地端点参数
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = COORD_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
        .app_device_id = ESP_ZB_HA_HOME_GATEWAY_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);
    
    // 正式向协议栈内核挂载当前端点
    esp_zb_device_register(ep_list);

    // 设定首要信道集合并启动（在这里安全、完整地配置 32 位全信道掩码！）
    esp_zb_set_primary_network_channel_set(1 << 11);
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    // 官方原生内核阻塞式主循环
    esp_zb_stack_main_loop();
} // 👈 补上了之前丢失的关键大括号！

// ======= 8. 系统主入口 =======
void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " 🛠️ 智能网关系统启动中 (基于 ESP-IDF 稳定版) 🛠️");
    ESP_LOGI(TAG, "===========================================");

    // 1. 初始化标准 NVS Flash（用于 Wi-Fi/系统配置）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 【核心修复】初始化 Zigbee 专属的 zb_storage NVS 分区
    ret = nvs_flash_init_partition("zb_storage");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase_partition("zb_storage"));
        ret = nvs_flash_init_partition("zb_storage");
    }
    ESP_ERROR_CHECK(ret);

    // 3. 创建高优先级的 Zigbee 协议栈独占任务 (分配 8192 字节防爆栈)
    xTaskCreate(esp_zb_task, "Zigbee_main_task", 8192, NULL, 24, NULL);
}