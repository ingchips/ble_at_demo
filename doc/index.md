# BLE AT Demo

实现了一系列 AT 指令，指令格式参考了 [ESP32 BLE AT 指令](https://docs.espressif.com/projects/esp-at/en/release-v2.1.0.0_esp32s2/AT_Command_Set/BLE_AT_Commands.html)，部分指令与之完全兼容。

**说明：**

1. 对于 ING916，10 个连接时内存使用已接近极限；
2. 扫描、连接等参数针对多连接做了调整，连接速度较慢。

## 编译配置

* `MAX_CONN_AS_MASTER`：作为主角色的个数（即最多可连接到多少个从机），默认 $8$ 个；

* `MAX_CONN_AS_SLAVE`：作为从角色的个数（即最多可被多少个主机连接），默认 $2$ 个。

## AT 指令说明

指令分为读写两种模式，写模式写作 `AT+XXX=.....`，读模式写作 `AT+XXX?` 或者 `AT+XXX`。

### 基础设置

1. 复位：`AT+RESET`

    清空 Flash 里存储的信息，并复位。复位后的默认值：

    * UART 波特率 115200

1. `AT+BLEADDR`

    读写蓝牙随机地址。写地址时的命令格式：

    `AT+BLEADDR=<addr_type>,<random_addr>`

1. UART 配置：`AT+UART=<baud>`

    设置 UART 波特率。

1. 关机模式：`AT+SHUTDOWN`

    关机后，可拉高 `WAKEUP_PIN` （GPIO 6）唤醒。

1. 省电模式：`AT+POWERSAVING`

    `AT+POWERSAVING=<enable>`

    注意：使能后，深睡眠时，UART 调电，无法通信。

### 广播

1. 读写广播数据：`AT+BLEADVDATA`

    写数据使用 `AT+BLEADVDATA=<adv_data>`。例如：`AT+BLEADVDATA="1122334455"`

    默认广播数据可通过[编辑器](https://ingchips.github.io/user_guide_cn/core-tools.html#%E5%90%91%E5%AF%BC)修改。
    默认广播设备名为“BLE-AT_0000”。

1. 读写扫描响应数据：`BLESCANRSPDATA`

1. 广播参数：`AT+BLEADVPARAM`

    `AT+BLEADVPARAM:<adv_int_min>,<adv_int_max>,<adv_type>,<own_addr_type>,<channel_map>,<adv_filter_policy>,<peer_addr_type>,<peer_addr>,<tx_power>`

1. 开始广播：`AT+BLEADVSTART`

1. 停止广播：`AT+BLEADVSTOP`

### 扫描

1. 扫描参数：`AT+BLESCANPARAM`

    `AT+BLESCANPARAM:<scan_type>,<own_addr_type>,<filter_policy>,<scan_interval>,<scan_window>`

1. 启停扫描：`AT+BLESCAN`

    `AT+BLESCAN=<enable>[[,<interval>],<filter_type>,<filter_param>]`

1. 扫描上报：`+BLESCAN`

    `+BLESCAN:<addr>,<rssi>,<adv_data>,<rsp_data>,<addr_type>`

### 连接

1. 连接参数：`AT+BLECONNPARAM`

    `AT+BLECONNPARAM:<conn_index>,<min_interval>,<max_interval>,<interval>,<latency>,<timeout>`


1. 发起连接：`AT+BLECONN`

    `AT+BLECONN=<conn_index>,<remote_address>,<addr_type>[,<timeout>]`


1. 断开连接：`AT+BLEDISCONN`

    `AT+BLEDISCONN=<conn_index>`

1. 主动上报：

    * 连接建立：`+BLECONN:<conn_index>,<addr>`

    * 连接断开：`+BLEDISCONN:<conn_index>,<reason>`

### GATT Server

GATT Profile 通过[图形化编辑器](https://ingchips.github.io/user_guide_cn/core-tools.html#%E5%90%91%E5%AF%BC)设置。

1. 主动上报：

    * 当客户端向特征写入值时：`+BLEGATTSWR:<conn_index>,<value_handle>,<hex_value>`

    * 当客户端读取特征的值时：`+BLEGATTSRD:<conn_index>,<value_handle>`

        此时，通过 `AT+BLEGATTSRD=<conn_index>,<att_handle>,<hex_data>` 发送响应数据。

1. 特征的值的主动上报：`AT+BLEGATTSWR=<conn_index>,<att_handle>,<mode>,<hex_data>`

    * mode: 0 表示 notify；1 表示 indicate。

### GATT Client

1. 发现服务：`AT+BLEGATTC`

    使用 `AT+BLEGATTC=<conn_index>` 发现指定连接上的 GATT Server Profile。
    发现过程中主动上报服务、特征、描述符信息，最终以上报 `+BLEGATTCC:<conn_index>,<status>` 结束。

    * 上报服务：`+BLEGATTCPRIMSRV:<conn_index>,<start_group_handle>,<end_group_handle>,<uuid>`

    * 上报特征：`+BLEGATTCCHAR:<conn_index>,<start_handle>,<end_handle>,<value_handle>,<properties>,<uuid>`

    * 上报描述符：`+BLEGATTCDESC:<conn_index>,<handle>,<uuid>`

1. 读取特征：`AT+BLEGATTCRD`

    `AT+BLEGATTCRD=<conn_index>,<handle>`

    成功读取后的响应：`+BLEGATTCRD:<conn_index>,<handle>,0,<value>`

    读取失败：`+BLEGATTCRD:<conn_index>,<handle>,<error_code>`

1. 写入特征：`AT+BLEGATTCWR`

    `AT+BLEGATTCWR=<conn_index>,<handle>,<value>`

    响应：`+BLEGATTCWR:<conn_index>,<handle>,<error_code>`

1. 订阅特征：`AT+BLEGATTCSUB`

    `AT+BLEGATTCSUB=<conn_index>,<handle>,<config>[,<desc_handle>]`

    其中，

    * `config`: 0 为取消订阅，1 为订阅 notification，2 为订阅 indication。
    * `desc_handle` 为 CCCD 的句柄，如果省略则取历史值；如果没有历史值，则取 `handle + 1`。


    订阅完成后，将主动上报 Server 数据：

    * `+BLEGATTCNOTI:<conn_index>,<value_handle>,<hex_value>`：notification

    * `+BLEGATTCIND:<conn_index>,<value_handle>,<hex_value>`：indication

### 配对

1. 设置配对参数：`AT+BLESECPARAM`

    `AT+BLESECPARAM:<enable>,<auth_req>,<io_cap>`

    其中，

    * `enable`：是否启用安全管理（默认不启用）；
    * `auth_req`：0 或者以下数值的组合

        ```c
        #define SM_AUTHREQ_BONDING 0x01
        #define SM_AUTHREQ_MITM_PROTECTION 0x04
        ```

    * `io_cap`：IO 能力，

        ```c
        typedef enum {
            IO_CAPABILITY_DISPLAY_ONLY = 0,
            IO_CAPABILITY_DISPLAY_YES_NO,
            IO_CAPABILITY_KEYBOARD_ONLY,
            IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
            IO_CAPABILITY_KEYBOARD_DISPLAY,
        } io_capability_t;
        ```
## Q & A

1. 如何最简单的开启主机从机功能？

    不需要开启，协议栈一直支持主从机功能并发。

1. 扫描后的数据获取

    参考代码 `at_on_adv_report`；[文档](https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-scan.html#%E5%A4%84%E7%90%86%E6%95%B0%E6%8D%AE)。

1. 扫描超时关闭或者主动关闭

    请参考[文档](https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-scan.html#%E8%B5%B7%E5%81%9C%E6%89%AB%E6%8F%8F)。

1. 简单化的主动连接

    请参考代码 `stack_initiate`；[文档](https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-conn.html#%E5%BB%BA%E7%AB%8B%E8%BF%9E%E6%8E%A5)。

1. 广播（多播）

    请参考代码 `get_ble_adv_start`；[文档](https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-adv.html)。

1. 被连接后的数据收发

    请参考 GATT Client/Server 相关 AT 指令的实现。[文档](https://ingchips.github.io/application-notes/pg_ble_stack_cn/ch-adv.html)。


1. 保存配置功能：可以是 fds 文件系统，也可以是 eerom（可重写）；

    协议栈提供了 kv_storage 模块；Flash 也提供了读写接口。Flash 空间不大，未使用文件系统。

    ING916 具备 [EFuse](https://ingchips.github.io/drafts/pg_ing916/ch-efuse.html)。

1. OTA 功能的实现；

    一种参考实现：ota_service。

1. 串口发送接收的 API（简单配置串口引脚即可使用）；

    参考 `cb_putc` 函数，[UART 外设文档](https://ingchips.github.io/drafts/pg_ing916/ch-uart.html)。

1. 各种低功耗的模式设置

    低功耗的模式由系统自动选择，请参考[文档](https://ingchips.github.io/application-notes/pg_power_saving_en/ch-framework.html)。