# k230_aim_main.py —— K230 端瞄准视觉程序(跑在 K230 上, 不是主控!)
#
# 【作用】找靶纸(A4 黑白同心圆)中心 -> 算像素偏差(dx,dy) -> 按 protocol.h 的
#   二进制帧(功能码 0x05 AIM_OFFSET)从 UART1 发给主控。没找到也发(found=0),
#   让主控的 k230_age 保持新鲜, 能区分"在线但没看到靶"和"链路断了"。
#
# 【怎么用】
#   调试期: CanMV IDE 打开本文件 -> 点运行 -> IDE 里看画面/框/串口打印,
#           用 工具->机器视觉->阈值编辑器 调 THRESHOLD_DARK(对着实拍画面拖滑块)。
#   部署期: 调好后在 IDE 里"保存到开发板"存为 main.py(或拷到 SD 卡根目录),
#           K230 独立上电即自动运行, 不再需要电脑。
#
# 【接线】K230 4pin 通讯口(USB口旁) <-> LaunchPad:
#   3:UART1_TXD -> PB16(J2.11, 主控UART2_RX)   4:UART1_RXD -> PB17(J2.18, 主控UART2_TX)
#   2:GND -> 共地    1:5V 台架期不接(K230 用自己 Type-C 供电)
#   ⚠ 亚博文档里 "TXD->PA25/RXD->PA26" 是他家底盘的脚位, 咱们的板不是, 别照抄。
#
# 【符号约定】(主控 app.c 的 AIM_K_PAN/TILT 增益符号据此标定)
#   dx = 靶心x - 画面中心x : 靶在画面右侧 => dx > 0
#   dy = 靶心y - 画面中心y : 靶在画面下方 => dy > 0 (图像坐标系 y 向下!)
#
# 【协议】与工程 protocol.h 逐字节对齐(签名 7362b70aad5ada78), 帧 11 字节:
#   AA 55 06 05 dxL dxH dyL dyH found CHK 0D   (int16 小端; CHK=LEN+FUNC+负载 sum8)

import time
import struct
from media.sensor import *      # K230 CanMV 摄像头 API(Sensor/MediaManager)
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ============================================================================
#  配置(待整定 —— 全在这一块, 下面正文不用动)
# ============================================================================
FRAME_W = 640                   # 画面宽; 主控 AIM_TOL_PX=8 按此宽度约±1.3%视场
FRAME_H = 480
CX0 = FRAME_W // 2              # 画面中心 = 偏差零点。若激光/枪管与摄像头有固定
CY0 = FRAME_H // 2              # 安装偏移, 上靶后直接改这两个数(瞄准零点标定)

# 黑色同心圆的 LAB 阈值(L_min,L_max,A_min,A_max,B_min,B_max) —— 待整定:
# 用 CanMV IDE 阈值编辑器对着实拍画面调, 让"只有靶纸黑圆是白色高亮"
THRESHOLD_DARK = [(0, 45, -25, 25, -25, 25)]

MIN_PIXELS = 150                # 待整定: 有效目标最小像素数(滤远处噪点/碎渣)
MIN_AREA = 300                  # 待整定: 有效目标最小外接矩形面积
ASPECT_MIN, ASPECT_MAX = 0.5, 2.0   # 外接框宽高比窗口: 同心圆近方形, 滤掉
                                    # 混进画面的黑胶带赛道线(细长条)
MERGE_MARGIN = 10               # 同心圆各环 blob 合并余量(环间距内即可并成一块)

UART_BAUD = 115200              # 与主控 UART2 及 protocol.h 一致, 不要动

# ============================================================================
#  协议组帧(对齐 protocol.h 的 proto_build, 勿改字节序/校验算法)
# ============================================================================
def build_aim_frame(dx, dy, found):
    """AIM_OFFSET(0x05) 帧: AA 55 LEN FUNC payload CHK 0D; CHK=sum8(LEN+FUNC+payload)"""
    # int16 夹紧, 防 struct.pack 溢出抛异常(靶飞出视场边缘时 dx 可到 ±320)
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))
    payload = struct.pack('<hhB', dx, dy, 1 if found else 0)   # 小端 int16,int16,uint8
    ln = len(payload) + 1                                       # LEN = 1(func) + 负载
    chk = (ln + 0x05 + sum(payload)) & 0xFF
    return b'\xAA\x55' + bytes([ln, 0x05]) + payload + bytes([chk, 0x0D])

# ============================================================================
#  外设初始化
# ============================================================================
def uart1_init():
    """4pin 通讯口 = GPIO9/10 复用为 UART1(官方 YbUart 同款脚位, 不依赖 ybUtils 库)"""
    fpioa = FPIOA()
    fpioa.set_function(9, FPIOA.UART1_TXD, ie=0, oe=1)
    fpioa.set_function(10, FPIOA.UART1_RXD, ie=1, oe=0)
    return UART(UART.UART1, UART_BAUD)

def find_target(img):
    """找靶心: 黑阈值 blob + merge(同心圆各环外接框重叠 -> 并成一块, 质心≈靶心)。
    返回 (found, cx, cy, blob)。滤波: 像素数/面积下限 + 宽高比窗口(排赛道线)。"""
    blobs = img.find_blobs(THRESHOLD_DARK, merge=True, margin=MERGE_MARGIN,
                           pixels_threshold=MIN_PIXELS, area_threshold=MIN_AREA)
    best = None
    for b in blobs:
        aspect = b.w() / b.h() if b.h() > 0 else 99.0
        if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
            continue                        # 细长条 = 赛道胶带/边框, 不是靶
        if best is None or b.pixels() > best.pixels():
            best = b                        # 多个候选取像素最多的(最近/最大)
    if best is None:
        return False, 0, 0, None
    return True, best.cx(), best.cy(), best

# ============================================================================
#  主程序
# ============================================================================
sensor = None
try:
    uart = uart1_init()

    sensor = Sensor(width=FRAME_W, height=FRAME_H)
    sensor.reset()
    sensor.set_framesize(width=FRAME_W, height=FRAME_H)
    sensor.set_pixformat(Sensor.RGB565)

    # VIRT = 输出到 IDE 虚拟屏(调试用); 脱机自启时无 IDE 也不报错, 帧被丢弃无害
    Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, fps=30)
    MediaManager.init()
    sensor.run()

    clock = time.clock()
    n = 0
    while True:
        clock.tick()
        img = sensor.snapshot()

        found, cx, cy, blob = find_target(img)
        dx = cx - CX0 if found else 0
        dy = cy - CY0 if found else 0

        # 每帧都发(≈30fps, 远快于主控 200ms 失联门限): found=0 也是心跳
        uart.write(build_aim_frame(dx, dy, found))

        # ---- 以下仅调试可视化, 不影响协议 ----
        img.draw_cross(CX0, CY0, color=(0, 255, 0), size=10)        # 画面中心
        if found:
            img.draw_rect(blob.rect(), color=(255, 0, 0))
            img.draw_cross(cx, cy, color=(255, 0, 0), size=10)      # 靶心
        Display.show_image(img)
        n += 1
        if n % 30 == 0:                     # 1秒一条, 别刷屏
            print("fps=%.1f found=%d dx=%d dy=%d" % (clock.fps(), found, dx, dy))

except KeyboardInterrupt:                   # IDE 里点停止
    pass
except Exception as e:
    import sys
    sys.print_exception(e)                  # 脱机跑挂了能在下次连 IDE 时查到原因
finally:
    if sensor is not None:
        sensor.stop()
    Display.deinit()
    MediaManager.deinit()
