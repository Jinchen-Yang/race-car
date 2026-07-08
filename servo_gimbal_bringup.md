# 舵机 / 云台现场调试记录

> 适用固件: r37 AIM 恢复版。目标是确认通道角色、调准 360° 连续舵机停转脉宽,再微调垂直俯仰角。

## 1. 当前固件初值

| 项 | 当前值 | 位置 |
|---|---:|---|
| PWM | TIMA1 / PA17(PAN) / PA16(TILT), 50Hz | `empty.syscfg` |
| 舵机轨 | PB23, 高电平上电, 默认断电 | `servo.c` |
| PAN 脉宽 | 0.6ms .. 2.4ms | `SERVO_PAN_PULSE_*_US` |
| TILT 脉宽 | 0.6ms .. 2.4ms | `SERVO_TILT_PULSE_*_US` |
| PAN 软件限位 | 10° .. 170°, init=90° | `SERVO_PAN_DEG_*` |
| TILT 软件限位 | 40° .. 140°, init=90° | `SERVO_TILT_DEG_*` |
| AIM 默认 | `AIM_FIXED_MODE=1`, 水平连续舵机手势 + 停转保持 | `app.c` |
| 实测角色 | 水平 360° 连续舵机=`SERVO_TILT/PA16`; 垂直 180° 位置舵机=`SERVO_PAN/PA17` | `app.c` |

## 2. VOFA 命令

> 注意: `G/g` 已用于灰度抗阴影门槛;连续舵机停转微调用 `Y/y`。

| 命令 | 作用 | 备注 |
|---|---|---|
| `C` | PAN/TILT 写 90° 中位 | 先写目标,再上电 |
| `O` | 舵机轨上电 | 仅 IDLE/STOP 生效 |
| `o` | 舵机轨断电 | 任意状态可用 |
| `J` / `j` | PAN +1° / -1° | 垂直 180° 位置舵机微调 |
| `K` / `k` | TILT +5° / -5° | 仅调试用;若这路是 360° 连续舵机,不要角度扫线 |
| `Y` / `y` | 连续舵机停转脉宽 +5us / -5us | 调到水平舵机完全不蠕转 |
| `?` | 回显状态 | 含 PAN/TILT 角度与脉宽 |

## 3. 端点标定流程

1. 不装负载或松开连杆,只插信号线和舵机电源,确认共地。
2. 上电后看串口横幅包含 `r37: AIM restored + gray shadow gate CSV`。
3. 发送 `O` 给舵机轨上电。若水平 360° 连续舵机轻微转,用 `Y/y` 每次 5us 调到完全不转。
4. 记录回显/CSV 里的 `aim_stop_us`,写回 `app.c` 的 `AIM_CONT_STOP_US`。
5. 垂直 180° 位置舵机用 `J/j` 微调高度;若确认机械安全,再回填 `AIM_ELEV_DEG`。
6. 不要用 `K/k` 长时间扫水平连续舵机,它不是位置舵机,会继续转线。

记录:

| 通道 | 安全最小角 | 安全最大角 | 中位是否正 |
|---|---:|---:|---|
| PAN |  |  |  |
| TILT |  |  |  |

## 4. AIM 手势和方向

当前固定方案不做水平定角:进 AIM 前 `AIM_CONT_SWEEP_MS` 内给连续舵机一个转向靶侧的手势脉宽,之后发 `AIM_CONT_STOP_US` 停转保持。

1. 发 `2` 选 F2,再发 `s` 进入 AIM。
2. 若水平舵机转反,改 `AIM_CONT_SWEEP_DIR` 的符号。
3. 若转得过远/不够,改 `AIM_CONT_SWEEP_MS` 或 `AIM_CONT_SWEEP_US`。
4. 若停稳后仍慢慢转,回 IDLE/STOP 后用 `Y/y` 调停转脉宽。

## 5. 烧录前回填项

- `servo.c`: `SERVO_PAN_DEG_MIN/MAX`, `SERVO_TILT_DEG_MIN/MAX`,必要时 `SERVO_*_REVERSE`。
- `app.c`: `AIM_CONT_STOP_US`,必要时 `AIM_CONT_SWEEP_DIR/MS/US`,以及 `AIM_ELEV_DEG`。
- README: 把最终端点和零点写进“舵机/云台快调”段,避免下次断片。
