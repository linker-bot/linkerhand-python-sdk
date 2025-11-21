
---

# Linker Hand Python API Documentation

## API Overview

This document provides a detailed overview of the Python API for the Linker Hand, including functions for controlling the hand's movements, retrieving sensor data, and setting operational parameters.


## Public API

### 设置速度
```python
def set_speed(self,speed=[100,100,100,100,100]) # 设置速度,O6、L6长度为6，L7长度为7，L10长度为10，其他长度为5
```
**Description**:  
设置手部的运动速度。  
**Parameters**:  
- `speed`: 一个包含速度数据的 list，长度为5个元素对应每个关节的速度值，如何是L7则为7个元素，对应每个电机速度。 每个元素值范围:0~255

---

### 设置五根手指的转矩限制 - 力度
```python
def set_torque(self, torque=[180,100,80,99,255]) # 设置速度,O6、L6长度为6，L7长度为7，L10长度为10，其他长度为5
```
**Description**:  
设置手指的转矩限制或力度，用于控制抓取力度。  
**Parameters**:  
- `torque`: 一个包含力度数据的 list，长度为5个元素对应每个手指的力度值，如何是L7则为7个元素，对应每个电机力度值。每个元素值范围:0~255

---

### 设置关节位置
```python
def finger_move(self,pose=[120,120,120,120,120,120,120,120,120,120]) # L10为例
```
**Description**:  
设置关节的目标位置，用于控制手指的运动。  
**Parameters**:  
- `pose`: 一个包含目标位置数据的 float类型的list，O6、L6长度为6，L7长度为7个元素，L10长度为10个元素，L20长度为20个元素，L25长度为25个元素。每个元素值范围:0~255

---

### 设置电机电流值
```python
def set_current(self, current=[99，72，80，66，20]) # L20为例
```
**Description**:  
设置电机的电流值。  
**Parameters**:  
- `current`: 一个包含目标电流数据的 int类型list，长度为5个元素，当前只支持L20版本。每个元素值范围:0~255

---

### 获取速度
```python
def get_speed(self)
return [180, 200, 200, 200, 200]
```
**Description**:  
获取当前设置的速度值。提示：需设置关节位置后才能获取到速度值。

**Returns**:  
- 返回一个 list，包含当前的手指速度设置值。每个元素值范围:0~255

---

### 获取当前关节状态
```python
def get_state(self)
return [81, 79, 79, 79, 79, 79, 83, 76, 80, 78]
```
**Description**:  
获取当前关节的状态float类型的list信息。提示：需要设置关节位置后才能获取到状态信息，O6、L6长度为6，L7长度为7个元素，L10长度为10个元素，L20长度为20个元素，L25长度为25个元素。每个元素值范围:0~255。

**Returns**:  
- 返回一个 float类型的list，包含当前关节的状态数据。每个元素值范围:0~255

---

### 获取法向压力、切向压力、切向方向、接近感应
```python
def get_force(self)
return [[255.0, 0.0, 0.0, 77.0, 192.0], [82.0, 0.0, 0.0, 230.0, 223.0], [107.0, 255.0, 255.0, 31.0, 110.0], [255.0, 0.0, 20.0, 255.0, 255.0]]
```
**Description**:  
获取手部传感器的综合数据，包括法向压力、切向压力、切向方向和接近感应。  
**Returns**:  
- 返回一个二维list，其中每个子list包含不同类别的list压力数据[[法向压力],[切向压力],[切向压力方向],[接近感应]]。类别每一个元素对应拇指、食指、中指、无名指、小拇指
每个元素值范围:0~255
---

### 获取版本号
```python
def get_version(self)
return [10, 6, 22, 82, 20, 17, 0]
```
**Description**:  
获取当前软件或硬件的版本号。  
**Returns**:  
- 返回一个字符串，表示当前的版本号。list元素依次表示:自由度\版本号\序号\左手76右手82\内部序列号

---
--------------------------------------------------------------
### 获取扭矩
```python
def get_torque(self)
return [200, 200, 200, 200, 200]
```
**Description**:  
获取当前手指扭矩list信息。表示每根手指当前电机扭矩，支持L20、L25。

**Returns**:  
- 返回一个 float类型的list。每个元素值范围:0~255

---

### 获取电机温度
```python
def get_temperature(self)
return [41, 71, 45, 40, 50, 47, 58, 50, 63, 70]
```
**Description**:  
获取当前关节的电机温度。

**Returns**:  
- 返回一个 list数据，包含当前关节的电机温度。

---

### 获取电机故障码
```python
def get_fault(self)
return [0, 4, 0, 0, 0, 0, 0, 0, 0, 0]
```
**Description**:  
获取当前关节电机故障，0表示正常 数字1电流过载 数字2温度过高 数字3编码错误 数字4过压/欠压。

**Returns**:  
- 返回一个 float类型的list，包含当前关节电机故障。

---

### 清除电机故障码
```python
def clear_faults(self)
```
**Description**:  
尝试清除电机故障，无返回值。只支持L20
**Returns**:  
无
---

## Example Usage

以下是一个完整的示例代码，展示如何使用上述 API：

```python

from LinkerHand.linker_hand_api import LinkerHandApi
def main():
    # 初始化API hand_type:left or right   hand_joint:L7 or L10 or L20 or L25
    linker_hand = LinkerHandApi(hand_type="left", hand_joint="L10")
    # 设置手指速度
    linker_hand.set_speed(speed=[120,200,200,200,200])
    # 设置手扭矩
    linker_hand.set_torque(torque=[200,200,200,200,200])
    # 获取手当前状态
    hand_state = linker_hand.get_state()
    # 打印状态值
    print(hand_state)

```

---

## Notes
- 在使用 API 之前，请确保手部设备已正确连接并初始化。
- 参数值（如速度、力度等）的具体范围和含义请参考设备的技术手册。

---

## Contact
- 如果有任何问题或需要进一步支持，请联系 [support@linkerhand.com](mailto:support@linkerhand.com)。

---
