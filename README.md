# 智能工位通信协议 v1.0

## 1. Arduino → ESP32-C6-A（串口）
**波特率：** 115200
**格式：** D:45,L:320,T:24.5,S:1

- D = 距离(cm)
- L = 光强(0-1023)
- T = 温度(°C)
- S = 状态(0=无人, 1=正常, 2=警告)

---

## 2. ESP32-C6-A → ESP32-C6-B（Zigbee）
**格式：** JSON

```json
{
  "distance": 45,
  "light": 320,
  "temperature": 24.5,
  "status": 1,
  "work_time": 3600,
  "warnings": 3,
  "score": 85
}
```

---

## 3. ESP32-C6-B → 云端（MQTT）

**上传主题：** `workspace/data`
**控制主题：** `workspace/control/ir/ac`
**控制主题：** `workspace/control/ir/light`
