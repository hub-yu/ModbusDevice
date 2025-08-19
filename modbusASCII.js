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

// MODBUS LRC校验计算
function calculateLRC(data) {
    let lrc = 0;
    for (let i = 0; i < data.length; i++)
        lrc += data[i];
    return (~lrc + 1) & 0xff;
}

function hexToAscii(val, lowercase) {
    if (val >= 0 && val <= 9)
    {
        return 0x30 + val;
    }
    else if (val >= 10 && val <= 15)
    {
        return (lowercase ? 0x61 : 0x41) + (val - 10);
    }
    return '\0'; // 非法值返回空字符
}

function byteToAscii(frame) {
    const byte = new Uint8Array(frame.length * 2 + 3);
    byte[0] = 0x3a;

    for (let i = 0; i < frame.length; i++) {
        byte[1 + 2*i] = hexToAscii((frame[i] &0xf0) >> 4, 0);
        byte[2 + 2*i] = hexToAscii((frame[i] &0xf), 0);
    }    

    byte[frame.length * 2 + 1] = 0x0d;
    byte[frame.length * 2 + 2] = 0x0a;
    return byte;
}

function serialize(str) {

    // 尝试解析JSON格式
    const modbusCmd = JSON.parse(str);

    switch (modbusCmd.functionCode) {
        case 1:
        case 2:
        case 5:
        case 3:
        case 4:
        case 6:
            // 创建MODBUS请求帧
            if (1) {
                const frame = new Uint8Array(7);
                frame[0] = modbusCmd.slaveId;                   // 设备ID
                frame[1] = modbusCmd.functionCode;              // 功能码
                frame[2] = (modbusCmd.address >> 8) & 0xFF;     // 寄存器地址高字节
                frame[3] = modbusCmd.address & 0xFF;            // 寄存器地址低字节
                frame[4] = (modbusCmd.number >> 8) & 0xFF;      // 寄存器数量高字节
                frame[5] = modbusCmd.number & 0xFF;            // 寄存器数量低字节
                frame[6] = calculateLRC(frame.slice(0, 6));
                return byteToAscii(frame);
            }
            break;
        case 15:
        case 16:
            if (1) {
                const frame = new Uint8Array(8 + modbusCmd.bytes);
                frame[0] = modbusCmd.slaveId;                   // 设备ID
                frame[1] = modbusCmd.functionCode;              // 功能码
                frame[2] = (modbusCmd.address >> 8) & 0xFF;     // 寄存器地址高字节
                frame[3] = modbusCmd.address & 0xFF;            // 寄存器地址低字节
                frame[4] = (modbusCmd.number >> 8) & 0xFF;      // 寄存器数量高字节
                frame[5] = modbusCmd.number & 0xFF;            // 寄存器数量低字节
                frame[6] = modbusCmd.bytes;                     // 写入字节数

                for (let i = 0; i < modbusCmd.bytes; i++)
                    frame[7 + i] = modbusCmd.data[i];

                frame[7 + modbusCmd.bytes] = calculateLRC(frame.slice(0, 7 + modbusCmd.bytes));
                return byteToAscii(frame);
                // // 计算CRC
                // const crc = calculateCRC16(frame.slice(0, 7 + modbusCmd.bytes));
                // frame[7 + modbusCmd.bytes] = crc & 0xFF;      // CRC低字节
                // frame[8 + modbusCmd.bytes] = (crc >> 8) & 0xFF;// CRC高字节

                // return frame;
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

    for (let i = 0; i < 3; i++) {
        transfer(JSON.stringify({ "slaveId": 0, "functionCode": 15, "address": 0, "number": 32, "bytes": 4, "data": [0xff, 0xff, 0xff, 0xff] }))
        await new Promise(resolve => setTimeout(resolve, 300));
        transfer(JSON.stringify({ "slaveId": 0, "functionCode": 15, "address": 0, "number": 32, "bytes": 4, "data": [0, 0, 0, 0] }))
        await new Promise(resolve => setTimeout(resolve, 300));
    }
}


function customJson() {

    return [
        { name: "打开全部", str: JSON.stringify({ "slaveId": 0, "functionCode": 15, "address": 0, "number": 32, "bytes": 4, "data": [0xff, 0xff, 0xff, 0xff] }), },
        { name: "关闭全部", str: JSON.stringify({ "slaveId": 0, "functionCode": 15, "address": 0, "number": 32, "bytes": 4, "data": [0, 0, 0, 0] }) },
        { name: "获取所有输出", str: JSON.stringify({ "slaveId": 0, "functionCode": 1, "address": 0, "number": 32 }) },
        { name: "获取所有输入", str: JSON.stringify({ "slaveId": 0, "functionCode": 2, "address": 0, "number": 32 }) },
        { name: "老化测试", fun: "test" },

        { name: "复位", str: JSON.stringify({"slaveId":0, "functionCode":6, "address":1, "number":1}), title: "保持寄存器修改后，需要重新上电或复位生效"},
        { name: "恢复出厂", str: JSON.stringify({"slaveId":0, "functionCode":6, "address":1, "number":2}), title: "保持寄存器修改后，需要重新上电或复位生效"},

        { name: "查询ID", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 0, "number": 1 }) },
        { name: "查询配置", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 2, "number": 1 }) },
        { name: "查询MAC", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 3, "number": 3 }) },
        { name: "查询IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 6, "number": 2 }) },
        { name: "查询掩码", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 8, "number": 2 }) },
        { name: "查询网关", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 10, "number": 2 }) },
        { name: "查询DNS", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": 12, "number": 2 }) },

        { name: "甬道0类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (0 * 22) + 0), "number": 1 }) },
        { name: "甬道0超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (0 * 22) + 1), "number": 1 }) },
        { name: "甬道0端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (0 * 22) + 2), "number": 1 }) },
        { name: "甬道0目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (0 * 22) + 3), "number": 1 }) },
        { name: "甬道0目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (0 * 22) + 4), "number": 2 }) },
        { name: "甬道0目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (0 * 22) + 6), "number": 16 }) },

        { name: "甬道1类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (1 * 22) + 0), "number": 1 }) },
        { name: "甬道1超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (1 * 22) + 1), "number": 1 }) },
        { name: "甬道1端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (1 * 22) + 2), "number": 1 }) },
        { name: "甬道1目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (1 * 22) + 3), "number": 1 }) },
        { name: "甬道1目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (1 * 22) + 4), "number": 2 }) },
        { name: "甬道1目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (1 * 22) + 6), "number": 16 }) },

        { name: "甬道2类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (2 * 22) + 0), "number": 1 }) },
        { name: "甬道2超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (2 * 22) + 1), "number": 1 }) },
        { name: "甬道2端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (2 * 22) + 2), "number": 1 }) },
        { name: "甬道2目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (2 * 22) + 3), "number": 1 }) },
        { name: "甬道2目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (2 * 22) + 4), "number": 2 }) },
        { name: "甬道2目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (2 * 22) + 6), "number": 16 }) },

        { name: "甬道3类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (3 * 22) + 0), "number": 1 }) },
        { name: "甬道3超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (3 * 22) + 1), "number": 1 }) },
        { name: "甬道3端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (3 * 22) + 2), "number": 1 }) },
        { name: "甬道3目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (3 * 22) + 3), "number": 1 }) },
        { name: "甬道3目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (3 * 22) + 4), "number": 2 }) },
        { name: "甬道3目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (3 * 22) + 6), "number": 16 }) },

        { name: "甬道4类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (4 * 22) + 0), "number": 1 }) },
        { name: "甬道4超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (4 * 22) + 1), "number": 1 }) },
        { name: "甬道4端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (4 * 22) + 2), "number": 1 }) },
        { name: "甬道4目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (4 * 22) + 3), "number": 1 }) },
        { name: "甬道4目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (4 * 22) + 4), "number": 2 }) },
        { name: "甬道4目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (4 * 22) + 6), "number": 16 }) },

        { name: "甬道5类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (5 * 22) + 0), "number": 1 }) },
        { name: "甬道5超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (5 * 22) + 1), "number": 1 }) },
        { name: "甬道5端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (5 * 22) + 2), "number": 1 }) },
        { name: "甬道5目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (5 * 22) + 3), "number": 1 }) },
        { name: "甬道5目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (5 * 22) + 4), "number": 2 }) },
        { name: "甬道5目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (5 * 22) + 6), "number": 16 }) },

        { name: "甬道6类型", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (6 * 22) + 0), "number": 1 }) },
        { name: "甬道6超时", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (6 * 22) + 1), "number": 1 }) },
        { name: "甬道6端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (6 * 22) + 2), "number": 1 }) },
        { name: "甬道6目标端口", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (6 * 22) + 3), "number": 1 }) },
        { name: "甬道6目标IP", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (6 * 22) + 4), "number": 2 }) },
        { name: "甬道6目标域名", str: JSON.stringify({ "slaveId": 0, "functionCode": 3, "address": (14 + (6 * 22) + 6), "number": 16 }) },


        { name: "设置ip", str: JSON.stringify({ "slaveId": 0, "functionCode": 16, "address": 6, "number": 2, "bytes": 4, "data": [192, 168, 1, 50] }), title: "设置ip为 192.168.1.50" },
        { name: "设置ID=3", str: JSON.stringify({ "slaveId": 0, "functionCode": 6, "address": 0, "number": 3 }) },
        { name: "设置ID=17", str: JSON.stringify({ "slaveId": 0, "functionCode": 6, "address": 0, "number": 17 }) },




        //   { name: "获取ip", str: JSON.stringify({"slaveId":0, "functionCode":3, "address":8, "number":4})},
        //   { name: "获取gw", str: JSON.stringify({"slaveId":0, "functionCode":3, "address":12, "number":4})},
        //   { name: "获取mask", str: JSON.stringify({"slaveId":0, "functionCode":3, "address":16, "number":4})},
        //   { name: "获取mac", str: JSON.stringify({"slaveId":0, "functionCode":3, "address":20, "number":6})},
        //   { name: "获取udp端口", str: JSON.stringify({"slaveId":0, "functionCode":3, "address":26, "number":1})},
        //   { name: "获取tcp服务器端口", str: JSON.stringify({"slaveId":0, "functionCode":3, "address":27, "number":1})},


          { name: "打开1", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":0, "number":0xff00})},
          { name: "打开2", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":1, "number":0xff00})},
          { name: "打开3", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":2, "number":0xff00})},
          { name: "打开4", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":3, "number":0xff00})},
          { name: "打开5", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":4, "number":0xff00})},
          { name: "打开6", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":5, "number":0xff00})},
          { name: "打开7", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":6, "number":0xff00})},
          { name: "打开8", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":7, "number":0xff00})},
          { name: "关闭1", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":0, "number":0})},
          { name: "关闭2", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":1, "number":0})},
          { name: "关闭3", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":2, "number":0})},
          { name: "关闭4", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":3, "number":0})},
          { name: "关闭5", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":4, "number":0})},
          { name: "关闭6", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":5, "number":0})},
          { name: "关闭7", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":6, "number":0})},
          { name: "关闭8", str: JSON.stringify({"slaveId":0, "functionCode":5, "address":7, "number":0})},

        //  { name: "获取模拟量", str: JSON.stringify({"slaveId":17, "functionCode":4, "address":0, "number":0x0002})},
    ];

}