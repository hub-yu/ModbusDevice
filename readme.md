## 硬件
- cpu:  stm32f030f4p6 
- 晶振:  8MHz

## modbus 协议

**从机地址：17 (0x11)**

功能码：
- 读输出：1 (0x01)  有效寄存器地址(0~5)
- 读输入：2 (0x02)  返回假数据
- 读保持寄存器：3 (0x03) 返回假数据
- 读输入寄存器：4 (0x04) 返回假数据
- 写单个输出：5 (0x05)  有效寄存器地址(0~5) 
- 写单个保持寄存器：6 (0x06)  返回假数据
- 写多个输出：15 (0x0F)  有效寄存器地址(0~5) 
- 写多个保持寄存器：6：16 (0x10) 返回假数据


## 注意

为方便测试 当CRC为0时 未校验CRC


## 命令表
```
       { name: "打开1", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":0, "number":0xff00})},
       { name: "打开2", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":1, "number":0xff00})},
       { name: "打开3", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":2, "number":0xff00})},
       { name: "打开4", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":3, "number":0xff00})},
       { name: "打开5", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":4, "number":0xff00})},
       { name: "打开6", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":5, "number":0xff00})},
       { name: "关闭1", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":0, "number":0})},
       { name: "关闭2", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":1, "number":0})},
       { name: "关闭3", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":2, "number":0})},
       { name: "关闭4", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":3, "number":0})},
       { name: "关闭5", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":4, "number":0})},
       { name: "关闭6", str: JSON.stringify({"slaveId":17, "functionCode":5, "address":5, "number":0})},
       { name: "打开全部", str: JSON.stringify({"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[0x3f]})},
       { name: "关闭全部", str: JSON.stringify({"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[0]})},
```

## 资源使用情况

```
[100%] Linking C executable ..\..\STM32F030F4P6.elf
Invoking: Cross ARM GNU Print Size
   text    data     bss     dec     hex filename
  12332      52    3956   16340    3fd4 E:/stm32f070f6p6/build/STM32F030F4P6.elf
      0   12384       0   12384    3060 E:/stm32f070f6p6/build/STM32F030F4P6.hex
[100%] Built target STM32F030F4P6.elf

```