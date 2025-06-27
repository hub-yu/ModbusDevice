
// MODBUS CRC16校验计算
function calculateCRC16(data) {
   let crc = 0xFFFF;
   for (let i = 0; i < data.length; i++) {
       crc ^= data[i];
       for (let j = 0; j < 8; j++) {
           if (crc & 0x0001) {
               crc = (crc >> 1) ^ 0xA001;
           } else {
               crc = crc >> 1;
           }
       }
   }
   return crc;
}

function serialize(str) {

   // 尝试解析JSON格式
   const modbusCmd = JSON.parse(str);
           
   switch(modbusCmd.functionCode) {
        case 5:
        case 3:
        case 6:
                // 创建MODBUS请求帧
            if(1) {
                const frame = new Uint8Array(8);
                frame[0] = modbusCmd.slaveId;                   // 设备ID
                frame[1] = modbusCmd.functionCode;              // 功能码
                frame[2] = (modbusCmd.address >> 8) & 0xFF;     // 寄存器地址高字节
                frame[3] = modbusCmd.address & 0xFF;            // 寄存器地址低字节
                frame[4] = (modbusCmd.number >> 8) & 0xFF;      // 寄存器数量高字节
                frame[5] =  modbusCmd.number & 0xFF;            // 寄存器数量低字节

                // 计算CRC
                const crc = calculateCRC16(frame.slice(0, 6));
                frame[6] = crc & 0xFF;      // CRC低字节
                frame[7] = (crc >> 8) & 0xFF;// CRC高字节

                return frame;
            }
            break;
        case 15:
            if(1) {
                const frame = new Uint8Array(9 + modbusCmd.bytes);
                frame[0] = modbusCmd.slaveId;                   // 设备ID
                frame[1] = modbusCmd.functionCode;              // 功能码
                frame[2] = (modbusCmd.address >> 8) & 0xFF;     // 寄存器地址高字节
                frame[3] = modbusCmd.address & 0xFF;            // 寄存器地址低字节
                frame[4] = (modbusCmd.number >> 8) & 0xFF;      // 寄存器数量高字节
                frame[5] =  modbusCmd.number & 0xFF;            // 寄存器数量低字节
                frame[6] = modbusCmd.bytes;                     // 写入字节数

                for (let i = 0; i < modbusCmd.bytes; i++) 
                    frame[7 + i] = modbusCmd.data[i];
                
                // 计算CRC
                const crc = calculateCRC16(frame.slice(0, 7 + modbusCmd.bytes));
                frame[7 + modbusCmd.bytes] = crc & 0xFF;      // CRC低字节
                frame[8 + modbusCmd.bytes] = (crc >> 8) & 0xFF;// CRC高字节

                return frame;
            }

           break;
       default:
           break;
   }

}


/**
* @param {Uint8Array} array 接收缓冲区数组
* @return {number} 缓冲区数组中，能正确解析完整一包的长度
*/
function deserialize(array) {
   //检索协议\n的结束标识
   let len = array.findIndex(item => item == 10);
   len += len >= 0 ? 1 : 0;
   return len > 0 ? len : array.length;
}

function log(array) {

   let log = '';
   console.log(array);

   // 创建一个新的Uint8Array来存储数据
   const data = new Uint8Array(array.slice(0, array.length + 1));

   try {
       // 使用TextDecoder解码Uint8Array
       log = new TextDecoder().decode(data);
   } catch (error) {
       console.error('解码错误:', error);
       log = '';
   }

return log;
}


async function test(transfer) {

   for(let i = 0; i < 2; i++) {
       transfer(JSON.stringify({"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[0x3f]}))
       await new Promise(resolve => setTimeout(resolve, 200));
       transfer(JSON.stringify({"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[0]}))
       await new Promise(resolve => setTimeout(resolve, 200));
   }   

   for(let i = 0; i < 24; i++) {
       transfer(`{"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[ ${1 << (i%6)}]}`)
       await new Promise(resolve => setTimeout(resolve, 200));
   }

   for(let i = 0; i < 3; i++) {
       transfer(JSON.stringify({"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[0x3f]}))
       await new Promise(resolve => setTimeout(resolve, 200));
       transfer(JSON.stringify({"slaveId":17, "functionCode":15, "address":0, "number":6, "bytes": 1, "data":[0]}))
       await new Promise(resolve => setTimeout(resolve, 200));
   }
}


function customJson() {

   return [
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
       { name: "老化测试", fun: "test"},
       { name: "查询设备ID", str: JSON.stringify({"slaveId":17, "functionCode":3, "address":0, "number":0x0001})},
       { name: "设置设备ID=3", str: JSON.stringify({"slaveId":17, "functionCode":6, "address":0, "number":0x0003})},
       { name: "设置设备ID=17", str: JSON.stringify({"slaveId":17, "functionCode":6, "address":0, "number":0x0011})},
   ];

}