# 舵机 / 云台现场调试记录

> 适用固件: r36。目标是先安全找端点,再标定 AIM 几何方案的零点和方向。

## 1. 当前固件初值

| 项 | 当前值 | 位置 |
|---|---:|---|
| PWM | TIMA1 / PA17(PAN) / PA16(TILT), 50Hz | `empty.syscfg` |
| 舵机轨 | PB23, 高电平上电, 默认断电 | `servo.c` |
| PAN 脉宽 | 0.6ms .. 2.4ms | `SERVO_PAN_PULSE_*_US` |
| TILT 脉宽 | 0.6ms .. 2.4ms | `SERVO_TILT_PULSE_*_US` |
| PAN 软件限位 | 10° .. 170°, init=90° | `SERVO_PAN_DEG_*` |
| TILT 软件限位 | 40° .. 140°, init=90° | `SERVO_TILT_DEG_*` |
| AIM 默认 | `AIM_FIXED_MODE=0`, 几何解算 | `app.c` |

## 2. VOFA 命令

| 命令 | 作用 | 备注 |
|---|---|---|
| `C` | PAN/TILT 写 90° 中位 | 先写目标,再上电 |
| `O` | 舵机轨上电 | 仅 IDLE/STOP 生效 |
| `o` | 舵机轨断电 | 任意状态可用 |
| `J` / `j` | PAN +5° / -5° | 用于扫水平端点 |
| `K` / `k` | TILT +5° / -5° | 用于扫俯仰端点 |
| `?` | 回显状态 | 含 PAN/TILT 角度与脉宽 |

## 3. 端点标定流程

1. 不装负载或松开连杆,只插信号线和舵机电源,确认共地。
2. 上电后看串口横幅包含 `r36: servo/gimbal bring-up`。
3. VOFA 发送 `C`,再发送 `O`。舵机应到中位,无连续抖动/堵转。
4. 用 `J` 慢慢增大 PAN,快到结构极限前停,记下 `?` 回显角度作为 `PAN_MAX`。
5. 用 `j` 慢慢减小 PAN,同理记下 `PAN_MIN`。
6. TILT 用 `K/k` 重复一遍,得到 `TILT_MAX/MIN`。
7. 把端点留 5° 机械余量后回填到 `servo.c` 的 `SERVO_*_DEG_MIN/MAX`。

记录:

| 通道 | 安全最小角 | 安全最大角 | 中位是否正 |
|---|---:|---:|---|
| PAN |  |  |  |
| TILT |  |  |  |

## 4. AIM 零点和方向

几何方案假设:车停 B 点、车头沿 A→B,舵机 90° 时激光指向车正左方。按当前场地常量,目标角约:

| 量 | 理论值 |
|---|---:|
| PAN | 133° |
| TILT | 116° |

调法:

1. 先用手动命令确认 `90°` 是否指向车正左方。若偏一个固定角,优先重装舵盘;来不及则改 `AIM_PAN_ZERO_DEG`。
2. 发 `2` 选 F2,再发 `s` 进入 AIM。若 PAN 往反方向动,把 `AIM_PAN_SIGN` 改成 `-1.0f`,同时把 `AIM_PAN_ZERO_DEG` 改成 `180.0f`。
3. 若 TILT 是机械固定支架,只记录光点高度,不必接 TILT 舵机;若接了 180°舵机,同理用 `AIM_TILT_ZERO_DEG/SIGN` 修方向。

## 5. 烧录前回填项

- `servo.c`: `SERVO_PAN_DEG_MIN/MAX`, `SERVO_TILT_DEG_MIN/MAX`,必要时 `SERVO_*_REVERSE`。
- `app.c`: `AIM_PAN_ZERO_DEG`, `AIM_PAN_SIGN`,必要时 `AIM_TILT_ZERO_DEG`, `AIM_TILT_SIGN`。
- README: 把最终端点和零点写进“舵机/云台快调”段,避免下次断片。
