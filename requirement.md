项目名称：ESP32-C3 智能光唤醒闹钟 (基础版)

核心功能：设备通过蓝牙接收一次设定后，每日在固定时间以模拟日出的方式唤醒用户；同时提供手动长按开启/关闭的台灯模式；短按随时查看四位数码管查看时间。

设计原则：极简交互，最大化深度睡眠以延长电池续航。

基础模板：基于ESP-IDF gatt_server 示例 (gatts_demo.c) 进行开发。

说明（2025-12 调试增强/需求更新）：
1) 常开模式（用于调试）
- 新增 Kconfig：CONFIG_LIGHT_ALARM_ALWAYS_ON
- 作用：设备不进入深度睡眠；BLE GAP 广播持续运行，便于手机/PC 蓝牙调试软件随时发现与连接。
- 现象：串口监视会周期性输出 "ALWAYS_ON tick" 日志。

2) 深度睡眠调试开关
- Kconfig：CONFIG_LIGHT_ALARM_DEBUG_DISABLE_DEEP_SLEEP
- 作用：禁用 deep sleep 以避免 ESP32-C3 进入 deep sleep 后导致主机侧 COM 口掉线/监视退出。

3) BLE 设备名与可发现性
- 设备名：LightClock_001
- 广播数据策略（按 BLE legacy 广播规范，单包 Advertising Data/Scan Response 各自总长均 <= 31 字节）：
    - Advertising Data：包含 Flags（通用可发现 + 不支持 BR/EDR）与服务 UUID (0xFF10)
    - Scan Response：包含 Complete Local Name（设备名），提高部分扫描工具显示名称的兼容性

4) 串口/日志输出（监视稳定性）
- 建议将主控制台输出切换到 USB Serial/JTAG（而不是 UART0），原因：
    - 本项目硬件映射把 GPIO20/21 用作 BTN/BAT_ADC_EN，可能与 UART0 默认管脚冲突
    - 使用 USB Serial/JTAG 可确保日志持续可见

📋 详细需求规格
一、硬件与引脚配置
此部分为AI提供精确的硬件控制映射。
引脚配置类型	功能描述             备注/参数
IO4	开漏输出	I2C SDA，连接 CH455G 驱动数码管	软件I2C，频率100kHz
IO5	开漏输出	I2C SCL，连接 CH455G	与IO4配对
IO6	PWM输出	PWM_WARM，控制暖光LED亮度	LEDC通道0，频率大于20kHz，分辨率500以内
IO7	PWM输出	PWM_COOL，控制冷光LED亮度	LEDC通道1，频率大于20kHz，分辨率500以内
IO20输入，上拉	BTN，用户按键	低电平有效，自带上拉、自带消抖，不需要软件控制
IO3	ADC输入	BAT_ADC，电池电压采样	使能后读取，分压比：15.1k/5.1k
IO21推挽输出	BAT_ADC_EN，电池检测使能	高电平有效，采样时使能，平时关闭
IO10输入	PG，USB电源插入状态检测	高电平表示USB已插入并良好
IO0, IO1	专用	连接32.768kHz外部晶振	用于RTC和深度睡眠定时

二、蓝牙服务 (GATT) 设计
基于 gatts_demo.c 中的 PROFILE_A 进行修改，删除不必要的PROFILE_B。
服务UUID：自定义一个主服务，例如 0xFF10。
特征值定义：在此服务下创建一个核心特征值：
特征UUID：0xFF11
属性：可写、可读
数据格式 (写入)：HHMM（4字节）。例如 0730 代表7点30分。
数据格式 (读取)：同写入格式，返回当前设定的时间。
处理逻辑：当手机App写入该特征值时，设备应将这4字节数据永久保存至NVS，并立即用此新时间重新计算睡眠间隔。

三、核心功能与行为逻辑
定义设备在各种场景下的具体行为。
功能模式/触发条件/设备行为
1. 上电/复位初始化	任何重新启动	1. 初始化所有驱动。2.从NVS读取“预设时间”。若无，则使用默认值 0700 并保存。3.立即计算到下一次“预设时间”的秒数，进入深度睡眠。

2. 蓝牙配置模式	手机连接并写入特征值1. 在 ESP_GATTS_WRITE_EVT 事件中，校验并保存数据至NVS。2. 向手机发送写入成功响应。3. 在3-5秒无操作后，自动进入深度睡眠（等待下一次唤醒）。

3. RTC定时唤醒 (主闹钟)	深度睡眠定时器到期（预设时间到）	1. 系统唤醒，执行灯光渐变任务：在30分钟内，平滑增加IO6/IO7的PWM占空比（从0%至100%）。2. 同步点亮数码管，持续显示当前时间（如 7:30）。3. 30分钟后，关闭PWM和数码管，计算24小时后的唤醒时间，再次深度睡眠。

4. 短按按键 (查看时间)	IO20 检测到短按（按下时间<2秒）	1. 从睡眠中唤醒或中断当前状态。2. 仅点亮数码管，显示当前时间，持续20秒。3. 20秒后，恢复到触发前的状态（睡眠或台灯模式）。

5. 长按按键 (台灯开关)	IO20 检测到长按（按下时间≥2秒）	触发状态切换：
- 若当前为睡眠或闹钟模式：立即以最大亮度开启PWM（台灯亮），数码管常亮显示时间。
- 若当前为台灯模式：立即关闭PWM和数码管，进入深度睡眠。

四、系统状态机
这是软件逻辑的核心，AI必须严格实现以下状态及转换。

状态/描述/进入动作/退出动作
DEEP_SLEEP/深度睡眠，仅RTC运行/配置唤醒源（定时/按键），启动睡眠/被RTC或按键中断唤醒
ACTIVE_IDLE/活跃空闲（如短按看时间后/启动20秒定时器，刷新数码管/定时器到点，关闭显示
ALARM_GRADIENT/执行日出渐变/启动30分钟渐变任务，开启数码管/任务结束，关闭所有输出
MANUAL_LIGHT/手动台灯模式/置PWM为最大值，开启数码管/关闭所有输出

状态转换规则：

任何状态下，长按都在 MANUAL_LIGHT 和 DEEP_SLEEP 之间切换。

DEEP_SLEEP和MANUAL_LIGHT 状态下短按都进入显示时间，并唤醒蓝牙，使上位机可以扫描到设备。ALARM_GRADIENT下，短按为关闭光闹钟，进入 DEEP_SLEEP。

DEEP_SLEEP 状态下，RTC定时到则进入 ALARM_GRADIENT。

ALARM_GRADIENT 结束后，自动回到 DEEP_SLEEP。


五、关键数据与存储结构
定义NVS中保存的数据格式。

typedef struct {
    uint8_t alarm_hour;   // 闹钟时 (0-23)
    uint8_t alarm_minute; // 闹钟分 (0-59)
    // 未来可扩展：渐变时长、亮度、日期等
} device_config_t;
⚠️ 关键注意事项
深度睡眠与蓝牙：ESP32-C3进入深度睡眠后蓝牙会断开。设计上必须是：配置完成后 -> 断开连接 -> 进入睡眠。在 ESP_GATTS_DISCONNECT_EVT 事件中延迟数秒再睡眠。

可发现性/广播：
- 设备应在需要配置/调试的窗口内保持可发现（可扫描到）。
- 常开调试模式（CONFIG_LIGHT_ALARM_ALWAYS_ON=1）下：设备始终保持广播（除非已连接）。
- 非常开模式下：短按触发显示时间时，唤醒并开启广播一段时间，便于上位机扫描与连接。

调试模式（常开 + 持续广播）：

- CONFIG_LIGHT_ALARM_ALWAYS_ON：设备不进入深度睡眠，BLE GAP 广播持续运行，便于调试软件稳定扫描到设备。
- CONFIG_LIGHT_ALARM_DEBUG_DISABLE_DEEP_SLEEP：禁用 deep sleep（用于排查 monitor/端口掉线问题）。

BLE 广播规范实现（为提升兼容性）：

- 设备名：LightClock_001
- Advertising Data（raw，≤31字节）：包含 Flags(0x06) + 16-bit Service UUID 列表(0xFF10)
- Scan Response（raw，≤31字节）：包含 Complete Local Name=LightClock_001

串口/监视注意：

- 由于本项目硬件映射使用 GPIO20/21 作为 BTN/BAT_ADC_EN，可能与 UART0 默认管脚冲突。
- 建议将日志控制台切换到 USB Serial/JTAG 以确保 monitor 能稳定看到日志输出。

功耗管理：

在进入 DEEP_SLEEP 前，必须将 IO21 (BAT_ADC_EN) 设为低，关闭分压电路。

检查 PWM_WARM/PWM_COOL、SEG_SDA/SEG_SCL 等引脚在睡眠时应输出固定低电平，防止漏电。

驱动准备：

CH455G驱动：需提供I2C基本写函数及数码管编码映射表。

LEDC PWM驱动：需提供初始化及占空比设置函数。

测试验证点：

蓝牙写入时间后，重启设备检查是否记忆。

用电源监控仪表测量深度睡眠电流（应<100μA）。

手动调整RTC时间，测试定时唤醒是否精确。

按键在各种状态下响应是否准确。