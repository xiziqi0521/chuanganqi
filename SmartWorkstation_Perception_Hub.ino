/**
 * @file SmartWorkstation_Perception_Hub.ino
 * @brief 智能工位数字孪生系统 - 底层感知与核心状态机核心
 * * 【全新架构变更说明】：
 * 1. 彻底移除物理红外库(IRremote)与相关物理引脚，改用软件状态机维护虚拟电器状态。
 * 2. 引入光敏电阻半导体物理模型，将逆向非线性的 ADC 数值（0-1023）完美校准转换为符合人类直觉的正向标准照度值（单位：Lux）。
 * 3. 优化分发总线，通过高频串口（115200波特率）向 ESP32-C6 和网页前端无线推送全量数字孪生数据流。
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==========================================
// 1. PIN DEFINITIONS (物理引脚分配)
// ==========================================
const int PIN_TRIG       = 2;   // 超声波发送端
const int PIN_ECHO       = 3;   // 超声波接收端
const int PIN_DS18B20    = 4;   // 1-Wire 温度总线
const int PIN_LED_DESK   = 8;   // 本地台灯控制引脚（保留作为状态指示/本地闭环）
const int PIN_LIGHT_ADC  = A0;  // 光敏电阻直插引脚（配合内部 INPUT_PULLUP）

// ==========================================
// 2. ALGORITHM & TIMING CONSTANTS (算法与时间常量)
// ==========================================
const int DIST_THRESHOLD_WARN = 30;   // 驼背/坐姿过近阈值 (cm)
const int DIST_THRESHOLD_AWAY = 150;  // 离座判定阈值 (cm)

// 🚀 核心标定变更：现在的单位是标准的物理勒克斯(Lux)。
// 经过反向分压推导，室内工位照度低于 150 Lux 判定为环境昏暗（天黑），触发补光。
const int LIGHT_THRESHOLD_DARK = 150; 

const unsigned long TIME_WINDOW_WARN = 3000;   // 连续驼背 3 秒触发网页/屏幕高危警告
const unsigned long TIME_WINDOW_AWAY = 300000; // 离座维持 5 分钟（300000ms）自动关闭网页虚拟空调

// 任务非阻塞分发定时器（利用 millis 彻底杜绝 delay）
const unsigned long TICK_SENSOR_SAMPLE  = 80;   // 采样周期 80ms，避开空气声波余震
const unsigned long TICK_SLOW_SENSORS   = 1000; // 环境温度、光强校准及台灯闭环控制周期 1s
const unsigned long TICK_SERIAL_SYNC    = 200;  // 向上游网络网关同步高频流数据周期 200ms
const unsigned long TICK_LCD_REFRESH    = 300;  // 本地 LCD1602 面板刷新周期 300ms

// ==========================================
// 3. OBJECT & STATE INITIALIZATION (对象与全局状态)
// ==========================================
LiquidCrystal_I2C lcd(0x27, 16, 2); 
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// 全局数字孪生核心变量
float currentDistance = 100.0; // 滑动窗口滤波后的距离 (cm)
int currentLight = 300;        // 校准转化后的真实正向照度值 (Lux)
float currentTemp = 25.0;      // 摄氏温度 (℃)
int systemState = 1;           // 状态机：0-节能/无人, 1-正常工作, 2-坐姿警告
bool isDeskLightOn = false;    // 虚拟台灯运行状态
bool isAcOn = true;            // 🚀 虚拟空调运行状态，默认开机为开启状态

// 超声波滑动窗口中值平均滤波器（去除由于答辩现场人员走动带来的毛刺跳变）
const int WINDOW_SIZE = 5;
float distBuffer[WINDOW_SIZE] = {100.0, 100.0, 100.0, 100.0, 100.0};
int bufferIndex = 0;

// 定时任务时间戳记录
unsigned long lastSensorTick = 0;
unsigned long lastSlowSensorTick = 0;
unsigned long lastSerialTick = 0;
unsigned long lastLcdTick = 0;

// 状态机计时器变量
unsigned long slouchStartTime = 0;
unsigned long awayStartTime = 0;
bool isSlouchingTimerActive = false;
bool isAwayTimerActive = false;
int lastRenderedState = -1;    // LCD 消隐防闪烁辅助标志

// ==========================================
// 4. FUNCTION IMPLEMENTATIONS (功能核心实现)
// ==========================================

/**
 * @brief 带硬件盲区消隐和防抖的抗干扰超声波底层驱动
 */
float getRawDistance() {
    // 1. 清空引脚残余电平
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(5);
    
    // 2. 发射标准 10us 超声波脉冲
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    
    // 3. 核心消抖设计：硬件盲区硬延迟 150 微秒，过滤发射瞬间的贴片震动干扰
    delayMicroseconds(150); 
    
    // 4. 等待 Echo 引脚变高，防死锁超时设为 4ms
    unsigned long startMicros = micros();
    while (digitalRead(PIN_ECHO) == LOW) {
        if (micros() - startMicros > 4000) return 999.0; 
    }
    
    // 5. 记录声波返回的真正起始点
    unsigned long echoStart = micros();
    
    // 6. 等待回波结束，最大量程维持 24ms
    while (digitalRead(PIN_ECHO) == HIGH) {
        if (micros() - echoStart > 24000) return 999.0; 
    }
    
    // 7. 计算净耗时并换算物理厘米
    unsigned long duration = micros() - echoStart;
    return ((duration + 150) * 0.0343) / 2.0;
}

/**
 * @brief 5项中值冒泡排序滤波器（剔除脉冲噪声）
 */
float getFilteredDistance() {
    float raw = getRawDistance();
    
    if (raw > 3.0 && raw <= 999.0) {
        distBuffer[bufferIndex] = raw;
        bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
    }

    float sorted[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; i++) sorted[i] = distBuffer[i];

    // 经典冒泡排序
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    // 剥离最大值和最小值，取中间3组求算术平均值
    float sum = 0;
    for (int i = 1; i < WINDOW_SIZE - 1; i++) {
        sum += sorted[i];
    }
    return sum / 3.0;
}

/**
 * @brief 核心工位多条件确定性有限状态机 (FSM)
 */
void updateSystemState() {
    unsigned long now = millis();

    // 条件 A：用户坐姿倾斜或距离过近 (触发驼背倒计时)
    if (currentDistance < DIST_THRESHOLD_WARN) {
        if (!isSlouchingTimerActive) {
            slouchStartTime = now;
            isSlouchingTimerActive = true;
        } else if (now - slouchStartTime >= TIME_WINDOW_WARN) {
            systemState = 2; // 连续 3 秒状态机切入：坐姿警告状态
        }
        isAwayTimerActive = false; 
    }
    // 条件 B：用户完全不在工位 (包含彻底空旷的 999 边界)
    else if (currentDistance > DIST_THRESHOLD_AWAY || currentDistance >= 990.0) {
        systemState = 0; // 瞬时切入：离座节能状态
        isSlouchingTimerActive = false;

        if (!isAwayTimerActive) {
            awayStartTime = now;
            isAwayTimerActive = true;
        } else if (now - awayStartTime >= TIME_WINDOW_AWAY) {
            // 🚀 核心联动：人离开满5分钟，软件自动关闭虚拟空调状态，后续将通过串口通知网页
            isAcOn = false; 
            isAwayTimerActive = false; 
        }
    }
    // 条件 C：用户处于标准完美工作姿态
    else {
        systemState = 1; // 状态机切回：正常状态
        isSlouchingTimerActive = false;
        
        // 自动联动恢复：只要用户重回座位，网页端的虚拟空调自动重开
        if (!isAcOn) {
            isAcOn = true; 
        }
        isAwayTimerActive = false;
    }
}

/**
 * @brief 本地液晶屏渲染逻辑（主要用于研发阶段的本地调试）
 */
void refreshLocalDisplay() {
    // 状态切换时清屏，防止残余字符叠加乱码
    if (systemState != lastRenderedState) {
        lcd.clear();
        lastRenderedState = systemState;
    }

    if (systemState == 2) { 
        lcd.setCursor(0, 0);
        lcd.print("   !!! WARN !!!  ");
        lcd.setCursor(0, 1);
        lcd.print("SIT UP STRAIGHT!");
    } 
    else if (systemState == 0) { 
        lcd.setCursor(0, 0);
        lcd.print("STATUS: IDLE    ");
        lcd.setCursor(0, 1);
        lcd.print("ENERGY SAVING... ");
    } 
    else { 
        lcd.setCursor(0, 0);
        lcd.print("D:");
        lcd.print((int)currentDistance);
        lcd.print("cm L:");
        lcd.print(currentLight);
        if(isDeskLightOn) lcd.print("*");
        lcd.print("   "); 

        lcd.setCursor(0, 1);
        lcd.print("T:");
        lcd.print(currentTemp, 1);
        lcd.print("C  STATE:OK   ");
    }
}

/**
 * @brief 统一数字孪生总线数据帧打包串口外发 (波特率 115200)
 */
void sendDataToHub() {
    // 组装格式统一、严格规范的报文传送给上游网关
    Serial.print("D:");
    Serial.print((int)currentDistance);
    Serial.print(",L:");
    Serial.print(currentLight);        // 正向物理意义的 Lux 照度值
    Serial.print(",T:");
    Serial.print(currentTemp, 1);
    Serial.print(",S:");
    Serial.print(systemState);
    Serial.print(",L_ST:");
    Serial.print(isDeskLightOn ? 1 : 0);
    Serial.print(",AC_ST:");
    Serial.println(isAcOn ? 1 : 0);    // 广播虚拟空调开关状态，用于响应网页前端动画
}

// ==========================================
// 5. ARDUINO STANDARD ENTRY (系统入口初始化)
// ==========================================
void setup() {
    // 初始化高速串口总线
    Serial.begin(115200); 
    
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    // 配置 A0 引脚为单片机内部上拉，免去面包板上的 10k 物理电阻布线
    pinMode(PIN_LIGHT_ADC, INPUT_PULLUP); 
    pinMode(PIN_LED_DESK, OUTPUT); 
    
    sensors.begin();
    sensors.setWaitForConversion(false); // 启用非阻塞异步温度转化，防止单片机在原地死等延迟
    
    lcd.init();
    lcd.backlight();
    
    lcd.setCursor(0, 0);
    lcd.print("System Reboot...");
    
    delay(500);
    lcd.clear();
}

/**
 * @brief 主时间分发循环 (纯非阻塞架构)
 */
void loop() {
    unsigned long currentMillis = millis();

    // 任务序列 1：超声波高速测距与状态机高频运算 (80ms 周期)
    if (currentMillis - lastSensorTick >= TICK_SENSOR_SAMPLE) {
        lastSensorTick = currentMillis;
        currentDistance = getFilteredDistance();
        updateSystemState(); 
    }

    // 任务序列 2：慢速传感器采样、双对数曲线校准及本地台灯闭环控制 (1000ms 周期)
    if (currentMillis - lastSlowSensorTick >= TICK_SLOW_SENSORS) {
        lastSlowSensorTick = currentMillis;
        
        // 【核心数学换算】：把逆向非线性的分压原始码（0-1023）标定换算为正向 Lux 照度
        int rawADC = analogRead(PIN_LIGHT_ADC);
        
        // 边界限幅软保护，防止计算时发生除以0或对数越界崩溃
        if (rawADC >= 1020) rawADC = 1020;
        if (rawADC <= 5) rawADC = 5;

        // 分压反推公式：利用单片机内部约 10k 欧姆的上拉电阻
        float v_out = ((float)rawADC * 5.0) / 1023.0;
        float r_ldr = (10000.0 * v_out) / (5.0 - v_out); 
        
        // 5528光敏电阻物理照度拟合公式 (光线越亮，Lux数值越大)
        currentLight = (int)pow((500000.0 / r_ldr), 1.428); 
        
        // 🚀 软件闭环判断：如果人在位(S!=0) 且 环境真实照度【低于】150 Lux时，台灯触发仿真开灯
        if (systemState != 0 && currentLight < LIGHT_THRESHOLD_DARK) {
            digitalWrite(PIN_LED_DESK, HIGH); 
            isDeskLightOn = true;
        } else {
            digitalWrite(PIN_LED_DESK, LOW);
            isDeskLightOn = false;
        }
        
        // 异步非阻塞提取 DS18B20 温度数据
        float t = sensors.getTempCByIndex(0);
        if (t != DEVICE_DISCONNECTED_C && t > -50.0) { 
            currentTemp = t;
        }
        sensors.requestTemperatures(); // 发出下一次异步采样请求，绝不卡死等待
    }

    // 任务序列 3：本地液晶屏数据显示刷新 (300ms 周期)
    if (currentMillis - lastLcdTick >= TICK_LCD_REFRESH) {
        lastLcdTick = currentMillis;
        refreshLocalDisplay();
    }

    // 任务序列 4：向下游网关（ESP32-C6）高频透传全量数字孪生数据包 (200ms 周期)
    if (currentMillis - lastSerialTick >= TICK_SERIAL_SYNC) {
        lastSerialTick = currentMillis;
        sendDataToHub();
    }
}