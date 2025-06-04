# cubicat_ble

A nimBLE client and server framework for the ESP32, easy to develop custom BLE smart appliances.

##### 服务器(Peripheral)：

创建的BLEServer实例默认包含一个Service和一个Characteristic，UUID分别是CUBICAT_SERVICE_UUID和
CUBICAT_PROTOCOL_CHAR_UUID。增加服务端功能只需要获取这个BLECharacteristic,用addOperation函数
添加对应操作，比如开灯操作，就addOperation(OP_GENERIC_SWITCH， function)在function内部实现收到
OP_GENERIC_SWITCH操作码后需要执行的逻辑

##### 客户端(Central)：

创建一个BLEClient实例，初始化后开始扫描，扫描到第一个外围设备后，会延迟3秒(可设定)开始连接所有扫描到的设备。
连接成功后就可以操作外围设备了，用连接句柄conn handle和Cubicat协议特征uuid CUBICAT_PROTOCOL_CHAR_UUID
可以获得协议通信用的BLECharacteristic，这个特征上的opCodes就包含连接的设备的所有可用功能的操作码，比如
发现连接设备的opCodes里包含OP_GENERIC_SWITCH说明这个设备有开关功能，你就可以写入这个操作码来控制设备的
开关

#### 协议说明

所有协议相关的都定义在ble_service_definitions.h文件中，OP_DEFINE用于定义操作码，操作码为32位整形，
高16位为操作业务代码，低16位为业务数据大小，CubicatBLE协议第一个字节为操作码数量，之后数据为操作码和
操作数据交替组成，BLEProtocol类负责编码和解码。

#### 其它

Cubicat BLE 具有客户端发现设备断连自动重新搜索和发起连接的功能，服务端有发现客户端断开后自动重新广播的功能，
双端依靠心跳机制保活。