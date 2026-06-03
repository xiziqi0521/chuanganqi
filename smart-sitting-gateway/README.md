# 基于 ESP32-C6 与 Zigbee 的智能坐姿监测网关系统
# Smart Sitting Posture Monitoring Gateway based on ESP32-C6 and Zigbee

[![ESP-IDF Version](https://img.shields.io/badge/ESP--IDF-v5.3.5-blue)](https://github.com/espressif/esp-idf)
[![Platform](https://img.shields.io/badge/Platform-ESP32--C6-orange)](https://www.espressif.com/en/products/socs/esp32-c6)
[![Protocol](https://img.shields.io/badge/Protocol-Zigbee_3.0-green)](https://csa-iot.org/all-solutions/zigbee/)

## 📝 项目简介 / Overview

本项目设计并实现了一个**边缘计算智能坐姿监测网关系统**。系统通过上游微控制器（Arduino）实时采集多模态环境与行为数据（包括人机距离、环境光照、温度等），利用核心网关（ESP32-C6）进行基于时间窗口的防抖边缘控制算法分析，精准识别“正常坐姿”、“不良坐姿（驼背）”及“离座”等行为状态。动态计算健康评分与有效专注时长，并通过 Zigbee 3.0 自定义局域网集群（Custom Cluster）面向全网进行高性能无损广播。

This repository implements an **Edge-Computing Smart Sitting Posture Monitoring Gateway System**. Multi-modal environmental and behavioral data (distance, illuminance, temperature) are captured by the upstream microcontroller (Arduino) and transmitted to the core gateway (ESP32-C6). Utilizing a time-window-based debounce edge algorithm, the gateway precisely identifies behavior states ("Normal", "Slouching", and "Away"), dynamically evaluates health scores and focused duration, and broadcasts the status synchronously over a Zigbee 3.0 network via a custom cluster.

---

## ✨ 核心特性 / Key Features

* **多模态数据融合与解析**：基于多传感器串行协同机制，实现高速高效的结构化文本数据解析（UART `sscanf` 架构）。
* **边缘智能防抖算法**：设计了基于 Ticks 计数器的坐姿状态判定算法，设置 15-ticks (约 3 秒) 的时序防抖窗口，有效过滤偶发性动作扰动，避免误报。
* **标准 Zigbee 3.0 本地建网**：基于 ESP-IDF 最新稳定版规范，重写弱函数内核事件处理器，实现协调器（Coordinator）自动网络形成（Formation）与设备导向（Steering）。
* **高性能透传互联**：构建专有应用层无线互联协议，封装 `0xFFF0` 自定义属性簇，采用紧凑型 cJSON 序列化流进行全网无损广播。

---

## 🛠️ 系统架构与数据流 / Architecture & Data Flow

```text
[ Arduino 传感器节点 ] 
       │  (实时采集: 超声波距离、光敏、温度)
       ▼  串口通信 (UART: 115200 bps 结构化文本)
[ ESP32-C6 智能网关 (本项目核心) ] 
       │  1. 状态机动态解析 (sscanf)
       │  2. 边缘防抖智能算法处理 (Debounce Status Machine)
       │  3. cJSON 序列化封装 (Unformatted JSON String)
       ▼  无线广播 (Zigbee Custom Cluster: 0xFFF0)
[ Zigbee 终端 / 路由器显示节点 (如 LCD/LED 报警) ]

