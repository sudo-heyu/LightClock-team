# PWM 渐变控制策略说明

本文档总结当前固件中针对冷/暖光的 PWM 控制与渐变策略，便于调试和维护。

## 硬件与通道映射
- 控制器：ESP32-C3
- 引脚：
  - IO6 → PWM_WARM (LEDC 低速模式，通道 0)
  - IO7 → PWM_COOL (LEDC 低速模式，通道 1)
- 定时器：LEDC TIMER0，分辨率 9bit (0..511)，频率 24 kHz (>20 kHz 要求)

## PWM 初始化与自检
- `pwm_led_init` 完成定时器与两通道配置。
- 初始化后执行自检：
  1. Warm 100% / Cool 0%，200ms
  2. Warm 0% / Cool 100%，200ms
  3. 全关
- 若自检阶段只有一侧点亮，优先检查对应引脚走线/驱动/LED 供电。

## 混光算法
- 输入：
  - 总亮度 `total_brightness` (0..100)
  - 色温 `color_temp` (0..100，0=全冷，100=全暖)
- 计算：
  - `warm = round(total * color_temp / 100)`
  - `cool = total - warm`
- 低亮保底：当总亮度 > 0 且某侧因取整变 0 时，强制给该侧 1%，从另一侧扣 1%，确保两路都能起亮。
- 调用：
  - `app_apply_light_linear_mix` 负责混光并调用 `pwm_led_set_percent`。

## 渐变策略（闹钟唤醒）
- 函数：`app_run_alarm_gradient`
- 持续时间：`sunrise_duration` 分钟（1..60，默认使用配置项或回退值；若检测到“已经晚于日出开始”，会根据距离闹钟的剩余时间缩短总时长）
- 进度到亮度映射：二次缓增（近似指数增长）从 0 → `wake_bright`：
  - 令 $p = \frac{elapsed}{total}$，则亮度比例 $\approx p^2$，前期更平缓、后期更快。
  - 为避免长时间全黑，progress>0 时亮度最小会抬到 1%。
- 亮度输出方式：通过 LEDC 的硬件 fade（逐步改变 PWM duty）实现视觉上的平滑过渡。
- 色温：渐变过程中保持当前配置的 `color_temp`。
- 中断：短按按钮可中断渐变。

## 台灯模式（手动常亮）
- 长按 1.5s 进入/退出台灯模式。
- 持续应用配置的 `wake_bright` 与 `color_temp`，短按仅显示时间不影响灯。
- 显示时间窗口期间（show time），长按可直接切入台灯模式，不会被短按逻辑阻塞。

## 日志与节流
- 日志 TAG：`PWM`
- 设定 duty 时的实时日志被限速为 3 秒一次：`set percent warm=X% cool=Y% duty=(a,b)`，便于监测又避免刷屏。
- 按键日志 TAG：`BTN`（pressed/long/short），长按阈值 1.5s。

## 关键参数（可调）
- 淡入时间：当前每次混光更新使用约 300ms 的 LEDC fade 时间（用于让 PWM duty 变化更平滑）。
- 渐变曲线：当前为二次缓增（$p^2$）。若还觉得过快/过慢，可以改为 $p^3$（更慢起步）或调节混光更新周期与 fade 时间。

## 调试指引
- 如果暖光不亮：
  - 查看初始化自检阶段是否亮；
  - 检查 `light mix` 与 `PWM` 日志中的 warm 分配是否为 0；
  - 确认 IO6 走线、MOSFET/驱动及 LED 供电。
- 如果亮度变化不平滑：
  - 可将渐变曲线从线性改为二次缓增（progress^2）再映射到 0..`wake_bright`。
- 如需更低日志频率，可调节 `pwm_led.c` 中 3 秒限速的间隔。
