"""
YOLOv5n ONNX 实时目标检测脚本 V3
使用 text_onnx/best.onnx 模型，通过 RealSense 摄像头实时检测并框选目标
V3 修复：WATCH/HIT 后窗口持续刷新；相机线程只在真正长时间无帧时报警
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import onnxruntime as ort
import os
import serial
import serial.tools.list_ports
import sys
import time
import threading
#watch -n 1 "awk '{print \$1/1000 \" °C\"}' /sys/class/thermal/thermal_zone0/temp"
# ==================== 配置 ====================
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(PROJECT_ROOT, "text_camer", "text_onnx", "best.onnx")
INPUT_SIZE = 320          # 模型输入尺寸
CONF_THRESHOLD = 0.5      # 置信度阈值
DRAW_COLOR = (0, 255, 0)  # 框选颜色 (绿色)

# 相机画面旋转设置
ROTATE_180 = False

# 苹果尺寸分类阈值 (mm)
BIG_MIN_MM = 60.0    # 大果阈值，偏大减少误判大果，偏小更容易判大果
SMALL_MAX_MM = 60.0  # 小果阈值，偏大更容易判小果，偏小减少小果

# WATCH 稳定性与过滤参数
WATCH_STABLE_FRAMES = 2      # 触发 HIT 需要连续稳定帧数，2-3 比较稳
WATCH_SCORE_MIN = 85         # 置信度下限，降低更灵敏但误触更多
WATCH_SEND_ANY_REACHABLE_ARM = True  # True: 只要映射后的 ARM 坐标有效，就允许 WATCH 发送 HIT
WATCH_STOP_ON_X_OUT_OF_RANGE = True  # True: WATCH 阶段允许仅 X 越界的目标提前触发停车
WATCH_X_STOP_MARGIN_MM = 80.0        # X 轴越界提前停车窗口，最终抓取仍以 SCAN 的严格坐标为准
EDGE_MARGIN_PX = 24          # 边缘过滤像素，增大更保守
DEPTH_JUMP_MAX_MM = 80       # 深度跳变阈值，过小易拒绝，过大易误触
POS_JUMP_MAX_MM = 25         # 机械臂坐标跳变阈值，过小易拒绝
DIAMETER_JUMP_MAX_MM = 12    # 直径跳变阈值，过小易拒绝
DEPTH_VALID_RATIO_MIN = 0.55 # bbox 内有效深度比例，过高易拒绝
SCAN_TIMEOUT_S = 9.0         # SCAN 内部超时(秒)，略小于 STM32 的 10s 等待，超时主动回 NONE

# 低位苹果放宽策略（更容易触发 WATCH->HIT）
LOW_APPLE_RELAX = True
LOW_APPLE_BOTTOM_RATIO = 0.60   # bbox 底部超过画面高度比例则视为低位目标
LOW_APPLE_SCORE_MIN = 50        # 低位目标置信度下限
LOW_APPLE_VALID_RATIO_MIN = 0.10
LOW_APPLE_EDGE_MARGIN_PX = 0

# 深度采样参数
DEPTH_SAMPLE_RADIUS = 2
MIN_VALID_DEPTH_M = 0.05

EXPECTED_DEPTH_W, EXPECTED_DEPTH_H = 640, 480
EXPECTED_COLOR_W, EXPECTED_COLOR_H = 640, 480
EXPECTED_FPS = 15
CAMERA_STALL_TIMEOUT_S = 2.0  # 仅用于报警：超过该时间相机线程没有新帧，才认为可能真掉流
CAMERA_THREAD_WAIT_TIMEOUT_MS = 1000  # 只在相机线程里等待帧；不会阻塞主业务/串口/YOLO
FRAME_DRAIN_MAX = 2             # 相机线程最多丢弃 2 帧积压旧帧，只保留最新帧
CAMERA_THREAD_START_TIMEOUT_S = 5.0
CAMERA_WARN_INTERVAL_S = 2.0
CAMERA_REAL_STALL_S = 2.0       # age 超过该值才打印真正的取流异常，避免 15fps 下误报
FRAME_MAX_AGE_S = 0.8           # 主线程不处理超过 0.8s 的旧帧，避免相机停更后反复用旧画面
SERIAL_READ_TIMEOUT_S = 0.02    # 主循环不要被串口 readline 长时间堵住
CAMERA_DEBUG_INTERVAL_S = 2.0   # 相机线程状态打印间隔
REALSENSE_QUEUE_SIZE = 1        # SDK 内部帧队列尽量只保留最新帧，避免堆积

# 串口配置
CH340_VID = 0x1A86
CH340_PID = 0x7523
SERIAL_BAUD_RATE = 115200

# 运行参数
DEBUG_MODE = True
PROTOCOL_DEBUG = True

# 显示窗口兼容策略：
# 1. 手动在桌面环境运行：默认显示 OpenCV 预览窗口。
# 2. systemd 服务 / SSH / 无 DISPLAY 环境运行：自动关闭窗口，避免 Qt xcb 崩溃。
# 3. 可通过环境变量强制关闭：HEADLESS=1 或 RUIKANG_SHOW_WINDOW=0。
def _env_flag(name, default=None):
    value = os.environ.get(name)
    if value is None:
        return default
    value = value.strip().lower()
    if value in ("1", "true", "yes", "on", "y"):
        return True
    if value in ("0", "false", "no", "off", "n"):
        return False
    return default


def _has_display_environment():
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _resolve_show_window(default=True):
    # HEADLESS 优先级最高，适合 systemd 服务里显式设置。
    if _env_flag("HEADLESS", None) is True or _env_flag("RUIKANG_HEADLESS", None) is True:
        return False

    explicit = _env_flag("RUIKANG_SHOW_WINDOW", None)
    if explicit is False:
        return False

    # 没有图形显示环境时，一律禁用窗口。否则 cv2.imshow/namedWindow 会触发 Qt xcb ABRT。
    if not _has_display_environment():
        return False

    if explicit is True:
        return True
    return bool(default)


SHOW_WINDOW = _resolve_show_window(default=True)
DETECT_EVERY_N_FRAMES = 1
PREVIEW_WHEN_IDLE = True
PREVIEW_MIN_INTERVAL_S = 0.05
WATCH_MIN_INTERVAL_S = 0.1
WATCH_DEBUG_INTERVAL_S = 0.5
SCAN_WAIT_DEBUG_INTERVAL_S = 0.5
CAM_REPORT_INTERVAL_S = 1.0

ARM_X_MIN = 0.0   # 机械臂 X 抓取窗口下限
ARM_X_MAX = 220.0 # 机械臂 X 抓取窗口上限
ARM_X_SOFT_CLAMP_MAX = 280.0

HIT_ZONE_X_MIN = 0.0   # HIT 仅在 GOOD 窗口内触发
HIT_ZONE_X_MAX = 220.0 # HIT 仅在 GOOD 窗口内触发

ARM_Y_MIN = 40.0  # 机械臂 Y 抓取窗口下限
ARM_Y_MAX = 440.0 # 机械臂 Y 抓取窗口上限；超过 440 的值会在软限幅里压回 440
ARM_Y_SOFT_CLAMP_MAX = 500.0
ARM_Y_GRIP_EXTRA_MM = 20.0  # 抓取点 Y 轴额外伸出量：标定算出的是苹果中心，让 Y 轴再多伸出一点，方便剪刀剪断挂果绳

ARM_Z_MIN = 0.0   # 机械臂 Z 抓取窗口下限
ARM_Z_MAX = 400.0 # 机械臂 Z 抓取窗口上限
ARM_Z_SOFT_CLAMP_MIN = -50.0

PI_DIAMETER_MIN_MM = 40
PI_DIAMETER_MAX_MM = 100
PI_SCORE_MIN = 0
PI_SCORE_MAX = 100

g_latest_cam_report = None


def _debug_value(value):
    text = str(value)
    return text.replace("\r", "\\r").replace("\n", "\\n").replace(",", ";")


def debug_event(tag, **fields):
    if not (DEBUG_MODE and PROTOCOL_DEBUG):
        return
    now = time.time()
    stamp = time.strftime("%H:%M:%S", time.localtime(now))
    ms = int((now - int(now)) * 1000)
    parts = [f"PIDBG,{stamp}.{ms:03d},{tag}"]
    for key, value in fields.items():
        parts.append(f"{key}={_debug_value(value)}")
    print(",".join(parts))


def _result_debug_fields(result):
    if result is None:
        return {"result": "NONE"}
    fields = {
        "type": result.get("type"),
        "arm": "{}/{}/{}".format(result.get("x"), result.get("y"), result.get("z")),
        "diameter": result.get("diameter"),
        "score": result.get("score"),
        "depth": result.get("depth_mm"),
        "valid": "{:.2f}".format(float(result.get("valid_ratio", 0.0))),
    }
    bbox = result.get("bbox")
    if bbox is not None:
        fields["bbox"] = "/".join(str(v) for v in bbox)
    return fields


def debug_result_event(tag, result=None, **fields):
    fields.update(_result_debug_fields(result))
    debug_event(tag, **fields)


def tx_line(ser, line):
    ser.write(line.encode("utf-8"))
    if DEBUG_MODE:
        print(f"TX:{line.strip()}")


def camera_to_arm(cam_x, cam_y, cam_z):
    """
    输入: 深度相机坐标 cam_x, cam_y, cam_z (mm)
    输出: 机械臂绝对坐标 arm_x, arm_y, arm_z (mm)
    """
    # 2026-05-26 实车 17 点反向标定拟合：
    # Arm(10,60,40)->cam(-15,91,297), Arm(210,60,40)->cam(135,99,284)
    # Arm(10,60,220)->cam(-21,-46,250), Arm(110,60,220)->cam(55,-38,255)
    # Arm(210,60,220)->cam(119,-63,264)
    # Arm(10,260,40)->cam(-24,110,466), Arm(110,260,40)->cam(63,103,463)
    # Arm(210,260,40)->cam(152,109,469)
    # Arm(10,260,220)->cam(-26,-40,456), Arm(110,260,220)->cam(69,-50,456)
    # Arm(210,260,220)->cam(142,-60,463)
    # Arm(10,440,120)->cam(-38,58,639), Arm(110,440,120)->cam(63,43,635)
    # Arm(210,440,120)->cam(151,37,640)
    # Arm(10,440,320)->cam(-42,-136,643), Arm(110,440,320)->cam(41,-135,642)
    # Arm(210,440,320)->cam(156,-146,646)
    # Arm(110,60,40) 当前完全看不见，未参与拟合。
    # 使用完整二次项拟合，仅建议在上述工作区附近使用。
    cam_x2 = cam_x * cam_x
    cam_y2 = cam_y * cam_y
    cam_z2 = cam_z * cam_z
    cam_xy = cam_x * cam_y
    cam_xz = cam_x * cam_z
    cam_yz = cam_y * cam_z

    arm_x = (1.617624689 * cam_x
             -0.1620570327 * cam_y
             -0.1607091710 * cam_z
             -0.00005542326407 * cam_x2
             +0.0004296451567 * cam_y2
             +0.0002233290152 * cam_z2
             -0.00002550632263 * cam_xy
             -0.0009299830138 * cam_xz
             +0.0002608401665 * cam_yz
             +61.04810073)
    arm_y = (-0.02776201793 * cam_x
             -0.5333899303 * cam_y
             +1.081452519 * cam_z
             -0.0004982098716 * cam_x2
             +0.0005452227004 * cam_y2
             -0.00006932703884 * cam_z2
             +0.0004032730656 * cam_xy
             +0.0001439622679 * cam_xz
             +0.0009223726213 * cam_yz
             -225.1874756)
    arm_z = (-0.02649637134 * cam_x
             -1.375391708 * cam_y
             -0.01945085821 * cam_z
             -0.00009690026882 * cam_x2
             +0.00003253931935 * cam_y2
             +0.00006074229058 * cam_z2
             +0.0003140050175 * cam_xy
             -0.00005684457041 * cam_xz
             +0.0004505962395 * cam_yz
             +162.1855029)
    return arm_x, arm_y, arm_z


def is_camera_target_grabbable(cam_x, cam_y, cam_z):
    if not np.isfinite(cam_x) or not np.isfinite(cam_y) or not np.isfinite(cam_z):
        return False
    return cam_z > 0


def is_arm_point_reachable(arm_x, arm_y, arm_z):
    if arm_x < ARM_X_MIN or arm_x > ARM_X_MAX:
        return False
    if arm_y < ARM_Y_MIN or arm_y > ARM_Y_MAX:
        return False
    if arm_z < ARM_Z_MIN or arm_z > ARM_Z_MAX:
        return False
    return True


def is_arm_yz_reachable(arm_y, arm_z):
    if arm_y < ARM_Y_MIN or arm_y > ARM_Y_MAX:
        return False
    if arm_z < ARM_Z_MIN or arm_z > ARM_Z_MAX:
        return False
    return True


def is_watch_x_stop_window(arm_x):
    return ((ARM_X_MIN - WATCH_X_STOP_MARGIN_MM) <= arm_x <=
            (ARM_X_MAX + WATCH_X_STOP_MARGIN_MM))


def clamp_arm_x_for_hit(arm_x):
    if arm_x < ARM_X_MIN:
        return int(round(ARM_X_MIN))
    if arm_x > ARM_X_MAX:
        return int(round(ARM_X_MAX))
    return int(round(arm_x))


def normalize_arm_point_for_send(arm_x, arm_y, arm_z):
    send_x = arm_x
    send_y = arm_y
    send_z = arm_z

    if ARM_X_MAX < send_x < ARM_X_SOFT_CLAMP_MAX:
        send_x = ARM_X_MAX
    if ARM_Y_MAX < send_y < ARM_Y_SOFT_CLAMP_MAX:
        send_y = ARM_Y_MAX
    if ARM_Z_SOFT_CLAMP_MIN < send_z < ARM_Z_MIN:
        send_z = ARM_Z_MIN

    if not is_arm_point_reachable(send_x, send_y, send_z):
        return None
    return round(send_x), round(send_y), round(send_z)


def normalize_arm_point_for_watch_stop(arm_x, arm_y, arm_z):
    send_y = arm_y
    send_z = arm_z

    if ARM_Y_MAX < send_y < ARM_Y_SOFT_CLAMP_MAX:
        send_y = ARM_Y_MAX
    if ARM_Z_SOFT_CLAMP_MIN < send_z < ARM_Z_MIN:
        send_z = ARM_Z_MIN

    if not is_arm_yz_reachable(send_y, send_z):
        return None
    return round(arm_x), round(send_y), round(send_z)


def clamp_int(value, min_value, max_value):
    value_i = int(round(value))
    if value_i < min_value:
        return min_value
    if value_i > max_value:
        return max_value
    return value_i


def get_grip_point_from_camera(cam_x, cam_y, cam_z, allow_x_out_of_range=False):
    if not is_camera_target_grabbable(cam_x, cam_y, cam_z):
        return None
    arm_x, arm_y, arm_z = camera_to_arm(cam_x, cam_y, cam_z)
    arm_y = arm_y + ARM_Y_GRIP_EXTRA_MM
    if allow_x_out_of_range:
        watch_point = normalize_arm_point_for_watch_stop(arm_x, arm_y, arm_z)
        if watch_point is None:
            return None
        if not is_watch_x_stop_window(watch_point[0]):
            return None
        return watch_point
    return normalize_arm_point_for_send(arm_x, arm_y, arm_z)


def find_ch340_port():
    """遍历系统中的串口设备，根据 VID 和 PID 查找 CH340"""
    ports = serial.tools.list_ports.comports()
    if DEBUG_MODE:
        for port in ports:
            print(
                f"PORTDBG,{port.device},VID={port.vid},PID={port.pid},DESC={port.description}"
            )
    for port in ports:
        if port.vid == CH340_VID and port.pid == CH340_PID:
            return port.device
        if port.description and ("CH340" in port.description or "USB-Serial" in port.description):
            return port.device
    return None


def start_pipeline(serial):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, EXPECTED_DEPTH_W, EXPECTED_DEPTH_H, rs.format.z16, EXPECTED_FPS)
    cfg.enable_stream(rs.stream.color, EXPECTED_COLOR_W, EXPECTED_COLOR_H, rs.format.bgr8, EXPECTED_FPS)

    try:
        profile = pipe.start(cfg)
        print(">>> 采用高分辨率模式启动成功 (USB 3.0 状态良好)。")
        return pipe, profile
    except RuntimeError as e:
        if "Couldn't resolve requests" in str(e):
            print("\n[警告] 无法应用高分辨率配置！正在自动切换至降级配置...\n")

    # 如果高分辨率失败，自动降级为设备默认流配置
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth)
    cfg.enable_stream(rs.stream.color)

    try:
        profile = pipe.start(cfg)
        print(">>> 采用默认降级模式启动成功。")
        return pipe, profile
    except Exception as e2:
        print(f"降级启动彻底失败: {e2}")
        return None, None



class RealSenseFrameGrabber:
    """
    RealSense 后台取帧线程 V2。

    关键变化：
    1. 相机线程中立刻把 RealSense frame 拷贝成 numpy 数组。
    2. 主线程不再持有 color_frame/depth_frame 这类 RealSense SDK 对象。
    3. 避免 Python 主线程长期持有 frame 导致 RealSense 内部帧池/队列被占住。
    4. 缓存区只保存最新一帧 numpy 图像，不使用无限队列。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.thread = None

        self.pipeline = None
        self.profile = None
        self.align = None
        self.intrinsics = None
        self.depth_scale = 0.001

        self.latest_color_image = None     # BGR numpy.ndarray
        self.latest_depth_image = None     # uint16 numpy.ndarray, aligned to color
        self.frame_index = 0
        self.last_frame_ts = 0.0
        self.last_poll_ts = 0.0
        self.last_debug_ts = 0.0
        self.no_frame_count = 0              # 总超时次数
        self.consecutive_no_frame_count = 0  # 连续超时次数，用于判断真掉流
        self.ok_frame_count = 0

        self.actual_w = 0
        self.actual_h = 0
        self.actual_fps = 0
        self.device_sn = None

        self.running = False
        self.last_error = None

    def start(self):
        self.thread = threading.Thread(
            target=self._run,
            name="RealSenseFrameGrabber",
            daemon=True,
        )
        self.thread.start()

        if not self.ready_event.wait(CAMERA_THREAD_START_TIMEOUT_S):
            print("相机线程启动超时。")
            return False

        with self.lock:
            if not self.running or self.intrinsics is None:
                if self.last_error:
                    print(f"相机线程启动失败: {self.last_error}")
                return False
            return True

    def _try_set_sensor_queue_size(self, profile):
        """尽量把 RealSense 传感器内部队列调小，失败不影响运行。"""
        try:
            dev = profile.get_device()
            for sensor in dev.query_sensors():
                try:
                    if sensor.supports(rs.option.frames_queue_size):
                        sensor.set_option(rs.option.frames_queue_size, REALSENSE_QUEUE_SIZE)
                except Exception:
                    pass
        except Exception:
            pass

    def _run(self):
        pipeline = None
        try:
            print("正在初始化相机线程...")
            ctx = rs.context()
            devs = ctx.query_devices()
            if len(devs) == 0:
                raise RuntimeError("未检测到 RealSense 相机")

            sn = devs[0].get_info(rs.camera_info.serial_number)
            print(f"检测到设备 SN: {sn}")

            pipeline, profile = start_pipeline(sn)
            if not pipeline or not profile:
                raise RuntimeError("RealSense pipeline 启动失败")

            self._try_set_sensor_queue_size(profile)

            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            actual_w, actual_h = color_stream.width(), color_stream.height()
            actual_fps = color_stream.fps()
            align = rs.align(rs.stream.color)
            intrinsics = color_stream.get_intrinsics()

            depth_scale = 0.001
            try:
                depth_sensor = profile.get_device().first_depth_sensor()
                depth_scale = float(depth_sensor.get_depth_scale())
            except Exception:
                pass

            with self.lock:
                self.pipeline = pipeline
                self.profile = profile
                self.align = align
                self.intrinsics = intrinsics
                self.depth_scale = depth_scale
                self.actual_w = actual_w
                self.actual_h = actual_h
                self.actual_fps = actual_fps
                self.device_sn = sn
                self.running = True
                self.last_error = None
                now = time.monotonic()
                self.last_frame_ts = now
                self.last_poll_ts = now
                self.last_debug_ts = now

            print("=" * 60)
            print("  YOLOv5n ONNX 实时目标检测")
            print(f"  分辨率: {actual_w}x{actual_h} @ {actual_fps}fps")
            print(f"  深度比例: {depth_scale}")
            print("  取帧模式: RealSense 后台线程，拷贝为 numpy，只缓存最新一帧")
            print("=" * 60)

            self.ready_event.set()

            while not self.stop_event.is_set():
                try:
                    now = time.monotonic()
                    with self.lock:
                        self.last_poll_ts = now

                    # 相机线程可以等待较长时间，因为它不会阻塞串口、YOLO 和主循环。
                    # 之前 30ms 轮询在 15fps 下容易产生大量 NO_FRAME 误报。
                    ok, frames = pipeline.try_wait_for_frames(CAMERA_THREAD_WAIT_TIMEOUT_MS)

                    if not ok:
                        now = time.monotonic()
                        with self.lock:
                            self.no_frame_count += 1
                            self.consecutive_no_frame_count += 1
                            last_frame_ts = self.last_frame_ts
                            last_debug_ts = self.last_debug_ts
                            last_error = self.last_error
                            consecutive_no_frame_count = self.consecutive_no_frame_count

                        age = now - last_frame_ts
                        if DEBUG_MODE and age >= CAMERA_REAL_STALL_S and now - last_debug_ts >= CAMERA_DEBUG_INTERVAL_S:
                            print(
                                "CAMTHREAD,REAL_STALL,"
                                f"age={age:.2f}s,total_no_frame={self.no_frame_count},"
                                f"consecutive_no_frame={consecutive_no_frame_count},last_error={last_error}"
                            )
                            with self.lock:
                                self.last_debug_ts = now
                        time.sleep(0.01)
                        continue

                    latest_frames = frames
                    for _ in range(FRAME_DRAIN_MAX):
                        ok2, frames2 = pipeline.try_wait_for_frames(0)
                        if not ok2:
                            break
                        latest_frames = frames2

                    aligned = align.process(latest_frames)
                    depth_frame = aligned.get_depth_frame()
                    color_frame = aligned.get_color_frame()

                    if not color_frame or not depth_frame:
                        continue

                    # 关键：这里立刻 copy 成 numpy，主线程不再持有 RealSense frame 对象。
                    color_image = np.asanyarray(color_frame.get_data()).copy()
                    if color_frame.profile.format() == rs.format.rgb8:
                        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                    depth_image = np.asanyarray(depth_frame.get_data()).copy()

                    # 显式丢掉 RealSense frame 引用，降低内部帧池被占用的概率。
                    del color_frame
                    del depth_frame
                    del aligned
                    del latest_frames
                    del frames

                    with self.lock:
                        self.latest_color_image = color_image
                        self.latest_depth_image = depth_image
                        self.frame_index += 1
                        self.last_frame_ts = time.monotonic()
                        self.last_error = None
                        self.consecutive_no_frame_count = 0
                        self.ok_frame_count += 1

                except Exception as exc:
                    with self.lock:
                        self.last_error = f"{type(exc).__name__}: {exc}"
                    if DEBUG_MODE:
                        print(f"REJDBG,相机线程取帧异常,{type(exc).__name__}:{exc}")
                    time.sleep(0.02)

        except Exception as exc:
            with self.lock:
                self.running = False
                self.last_error = f"{type(exc).__name__}: {exc}"
            if DEBUG_MODE:
                print(f"REJDBG,相机线程初始化异常,{type(exc).__name__}:{exc}")
            self.ready_event.set()

        finally:
            try:
                if pipeline:
                    pipeline.stop()
            except Exception as exc:
                if DEBUG_MODE:
                    print(f"REJDBG,停止相机pipeline异常,{type(exc).__name__}:{exc}")

            with self.lock:
                self.running = False
            self.ready_event.set()

    def get_latest_frames(self):
        """
        返回 color_image, depth_image, frame_index。
        返回的是 numpy 数组引用；主线程只读，不修改。
        """
        with self.lock:
            return self.latest_color_image, self.latest_depth_image, self.frame_index

    def get_intrinsics(self):
        with self.lock:
            return self.intrinsics

    def get_depth_scale(self):
        with self.lock:
            return self.depth_scale

    def get_last_frame_ts(self):
        with self.lock:
            return self.last_frame_ts

    def get_last_error(self):
        with self.lock:
            return self.last_error

    def get_stats(self):
        with self.lock:
            return {
                "running": self.running,
                "frame_index": self.frame_index,
                "last_frame_ts": self.last_frame_ts,
                "last_poll_ts": self.last_poll_ts,
                "no_frame_count": self.no_frame_count,
                "consecutive_no_frame_count": self.consecutive_no_frame_count,
                "ok_frame_count": self.ok_frame_count,
                "last_error": self.last_error,
            }

    def is_running(self):
        with self.lock:
            return self.running

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)

def preprocess(image):
    """将摄像头图像预处理为 YOLOv5 模型输入格式"""
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
    img = np.expand_dims(img, axis=0)    # 添加 batch 维度
    return img


def postprocess(output, img_w, img_h):
    """
    解析模型输出，返回检测结果列表
    输出格式: [1, 6300, 6] -> [x_center, y_center, w, h, conf, class]
    """
    detections = output[0]  # (6300, 6)
    results = []

    scale_x = img_w / INPUT_SIZE
    scale_y = img_h / INPUT_SIZE

    for det in detections:
        x_c, y_c, bw, bh, conf, cls = det
        if conf < CONF_THRESHOLD:
            continue

        # 从中心点坐标还原到左上/右下像素坐标
        x1 = int((x_c - bw / 2) * scale_x)
        y1 = int((y_c - bh / 2) * scale_y)
        x2 = int((x_c + bw / 2) * scale_x)
        y2 = int((y_c + bh / 2) * scale_y)

        x1 = max(0, min(x1, img_w - 1))
        y1 = max(0, min(y1, img_h - 1))
        x2 = max(0, min(x2, img_w - 1))
        y2 = max(0, min(y2, img_h - 1))

        results.append((x1, y1, x2, y2, conf))

    return results


def nms(detections, iou_threshold=0.45):
    """非极大值抑制，去除重复框"""
    if len(detections) == 0:
        return []

    boxes = np.array([[d[0], d[1], d[2], d[3]] for d in detections])
    scores = np.array([d[4] for d in detections])

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]

    keep = []
    # 按置信度从高到低迭代，抑制高重叠的候选框
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return [detections[i] for i in keep]


def map_display_to_depth(x, y, width, height, rotate_180=False):
    if rotate_180:
        return (width - 1 - x, height - 1 - y)
    return (x, y)


def is_bbox_near_edge(x1, y1, x2, y2, width, height, margin_px):
    if x1 <= margin_px or y1 <= margin_px:
        return True
    if x2 >= width - 1 - margin_px or y2 >= height - 1 - margin_px:
        return True
    return False


def is_low_apple_bbox(bbox, frame_size):
    _, _, _, y2 = bbox
    _, h = frame_size
    return y2 >= int(h * LOW_APPLE_BOTTOM_RATIO)


def is_bbox_near_edge_for_watch(x1, y1, x2, y2, width, height, margin_px, allow_bottom_edge=False):
    if not allow_bottom_edge:
        return is_bbox_near_edge(x1, y1, x2, y2, width, height, margin_px)
    if x1 <= margin_px or y1 <= margin_px:
        return True
    if x2 >= width - 1 - margin_px:
        return True
    return False


def get_hit_zone(arm_x):
    if arm_x < HIT_ZONE_X_MIN:
        return "EARLY"
    if arm_x > HIT_ZONE_X_MAX:
        return "LATE"
    return "GOOD"


def build_watch_hit_result(result):
    if result is None:
        return None, "UNKNOWN", "NO_TARGET"
    zone = get_hit_zone(result["x"])
    if is_arm_point_reachable(result["x"], result["y"], result["z"]):
        return dict(result), zone, "NORMAL"
    if not WATCH_STOP_ON_X_OUT_OF_RANGE:
        return None, zone, "FULL_RANGE_REQUIRED"
    if not is_arm_yz_reachable(result["y"], result["z"]):
        return None, zone, "YZ_OUT_OF_RANGE"
    if not is_watch_x_stop_window(result["x"]):
        return None, zone, "X_TOO_FAR"

    hit_result = dict(result)
    hit_result["x"] = clamp_arm_x_for_hit(result["x"])
    return hit_result, "GOOD", "X_CLAMPED_{}".format(zone)


def estimate_depth_m_from_bbox(depth_image, depth_scale, x1, y1, x2, y2, width, height, rotate_180=False):
    """从 bbox 区域内取中值深度 (米)，并返回有效采样比例。depth_image 为 aligned uint16 numpy 数组。"""
    if depth_image is None:
        return 0.0, 0.0

    depths = []
    total = 0
    bbox_w = max(1, x2 - x1)
    bbox_h = max(1, y2 - y1)
    step = max(2, min(6, min(bbox_w, bbox_h) // 6))
    dh, dw = depth_image.shape[:2]

    for y in range(y1, y2, step):
        for x in range(x1, x2, step):
            if 0 <= x < width and 0 <= y < height:
                dx, dy = map_display_to_depth(x, y, width, height, rotate_180)
                if 0 <= dx < dw and 0 <= dy < dh:
                    total += 1
                    raw = int(depth_image[dy, dx])
                    if raw > 0:
                        d = raw * depth_scale
                        if d > MIN_VALID_DEPTH_M:
                            depths.append(d)
    if not depths or total == 0:
        return 0.0, 0.0
    return float(np.median(depths)), float(len(depths)) / float(total)

def estimate_diameter_mm(x1, y1, x2, y2, depth_m, intrinsics):
    """根据 bbox 像素直径 + 深度 + 内参估算实际直径 (mm)"""
    bbox_w = max(1, x2 - x1)
    bbox_h = max(1, y2 - y1)
    diameter_px = min(bbox_w, bbox_h)
    focal_px = (intrinsics.fx + intrinsics.fy) * 0.5
    diameter_m = (diameter_px * depth_m) / focal_px
    return diameter_m * 1000.0


def classify_apple(diameter_mm):
    if diameter_mm >= BIG_MIN_MM:
        return "BIG"
    if diameter_mm <= SMALL_MAX_MM:
        return "SMALL"
    return "UNKNOWN"


def init_onnx_session():
    print("正在加载 YOLOv5n ONNX 模型...")
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(MODEL_PATH, sess_options=options)
    print(f"模型已加载: {MODEL_PATH}")
    print(f"输入: {session.get_inputs()[0].name} {session.get_inputs()[0].shape}")
    print(f"输出: {session.get_outputs()[0].name} {session.get_outputs()[0].shape}")
    return session


def init_camera():
    """
    启动 RealSense 后台取帧线程，并返回兼容原业务代码的 camera_state。
    """
    grabber = RealSenseFrameGrabber()
    if not grabber.start():
        grabber.stop()
        return None

    return {
        "grabber": grabber,
        "intrinsics": grabber.get_intrinsics(),
        "depth_scale": grabber.get_depth_scale(),
        "last_frame_ts": grabber.get_last_frame_ts(),
        "last_frame_index": -1,
        "last_warn_ts": 0.0,
    }


def restart_camera(state):
    """
    保留这个函数是为了兼容旧代码，但主流程不再自动调用它。
    真正需要人工恢复时，可以在调试阶段手动调用。
    """
    try:
        if state and state.get("grabber"):
            state["grabber"].stop()
    except Exception:
        pass
    return init_camera()


def get_latest_aligned_frames(camera_state):
    """
    从 RealSense 后台线程读取最新 numpy 帧。
    主线程不再持有 RealSense frame 对象。
    """
    if camera_state is None:
        return None, None

    grabber = camera_state.get("grabber")
    if grabber is None:
        return None, None

    color_image, depth_image, frame_index = grabber.get_latest_frames()
    if color_image is None or depth_image is None:
        return None, None

    last_frame_ts = grabber.get_last_frame_ts()
    if time.monotonic() - last_frame_ts > FRAME_MAX_AGE_S:
        return None, None

    camera_state["last_frame_ts"] = last_frame_ts
    camera_state["last_frame_index"] = frame_index
    return color_image, depth_image

def preview_once(camera_state, frame_fail_count):
    if not SHOW_WINDOW:
        return frame_fail_count, camera_state
    if camera_state is None:
        return frame_fail_count + 1, camera_state
    color_image, _ = get_latest_aligned_frames(camera_state)
    if color_image is None:
        return frame_fail_count + 1, camera_state
    frame_fail_count = 0

    image = color_image.copy()
    if ROTATE_180:
        image = cv2.rotate(image, cv2.ROTATE_180)

    cv2.imshow("YOLOv5n Detection", image)
    cv2.waitKey(1)
    return frame_fail_count, camera_state

def preview_latest_frame_only(camera_state):
    """仅刷新最新相机画面，不改变 frame_fail_count，不做 YOLO。用于 WATCH/SCAN/HIT 后保持窗口持续更新。"""
    if not SHOW_WINDOW:
        return False
    if camera_state is None:
        return False

    color_image, _ = get_latest_aligned_frames(camera_state)
    if color_image is None:
        return False

    image = color_image.copy()
    if ROTATE_180:
        image = cv2.rotate(image, cv2.ROTATE_180)

    cv2.imshow("YOLOv5n Detection", image)
    cv2.waitKey(1)
    return True

def window_closed():
    if not SHOW_WINDOW:
        return False
    try:
        return cv2.getWindowProperty("YOLOv5n Detection", cv2.WND_PROP_VISIBLE) < 1
    except Exception:
        return False


def find_best_target_once(session, camera_state, frame_fail_count, allow_x_out_of_range=False):
    """
    返回: (result, error_code, updated_fail_count, updated_camera_state)
    result 为 dict 或 None, error_code 为 str 或 None
    """
    global g_latest_cam_report

    intrinsics = camera_state["intrinsics"]
    depth_scale = camera_state.get("depth_scale", 0.001)

    last_color_image = None
    last_depth_image = None

    for _ in range(max(1, DETECT_EVERY_N_FRAMES)):
        color_image, depth_image = get_latest_aligned_frames(camera_state)
        if color_image is None or depth_image is None:
            frame_fail_count += 1
            continue
        last_color_image = color_image
        last_depth_image = depth_image
        frame_fail_count = 0

    if frame_fail_count >= 3 and DEBUG_MODE:
        print("REJDBG,相机短时间无新帧，继续等待下一帧")
        frame_fail_count = 0

    if last_color_image is None or last_depth_image is None:
        return None, "FRAME_TIMEOUT", frame_fail_count, camera_state

    image = last_color_image.copy()
    if ROTATE_180:
        image = cv2.rotate(image, cv2.ROTATE_180)

    h, w = image.shape[:2]

    input_tensor = preprocess(image)
    output = session.run(None, {session.get_inputs()[0].name: input_tensor})
    detections = postprocess(output[0], w, h)
    detections = nms(detections)

    display_frame = image.copy()
    nearest_target = None
    nearest_depth_m = None
    any_depth_valid = False

    for (x1, y1, x2, y2, conf) in detections:
        if SHOW_WINDOW:
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), DRAW_COLOR, 2)
            label = f"{conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(display_frame, (x1, y1 - th - 4), (x1 + tw, y1), DRAW_COLOR, -1)
            cv2.putText(display_frame, label, (x1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        z_m, valid_ratio = estimate_depth_m_from_bbox(
            last_depth_image, depth_scale, x1, y1, x2, y2, w, h, rotate_180=ROTATE_180
        )

        if z_m > 0:
            any_depth_valid = True
            diameter_mm = estimate_diameter_mm(x1, y1, x2, y2, z_m, intrinsics)
            size_label = classify_apple(diameter_mm)

            if SHOW_WINDOW:
                size_text = f"{size_label} {diameter_mm:.1f}mm"
                tx = x1
                ty = y2 + 20 if y2 + 20 < h - 5 else y1 - 10
                cv2.putText(display_frame, size_text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 255, 0), 1)

            if size_label == "UNKNOWN":
                if DEBUG_MODE:
                    print(
                        "REJDBG,尺寸未知,直径={:.1f}mm,阈值:SMALL<= {:.1f} / BIG>= {:.1f}".format(
                            diameter_mm, SMALL_MAX_MM, BIG_MIN_MM
                        )
                    )
                continue

            if nearest_depth_m is None or z_m < nearest_depth_m:
                nearest_depth_m = z_m
                depth_cx, depth_cy = map_display_to_depth(cx, cy, w, h, ROTATE_180)
                nearest_target = (depth_cx, depth_cy, z_m, size_label, diameter_mm, conf, (x1, y1, x2, y2), valid_ratio)
        else:
            if SHOW_WINDOW:
                size_text = "DEPTH--"
                tx = x1
                ty = y2 + 20 if y2 + 20 < h - 5 else y1 - 10
                cv2.putText(display_frame, size_text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 255, 0), 1)
            if DEBUG_MODE:
                print("REJDBG,深度无效,bbox=({},{},{},{}),conf={:.2f}".format(x1, y1, x2, y2, conf))

    if SHOW_WINDOW:
        cv2.putText(display_frame, f"Targets: {len(detections)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, DRAW_COLOR, 2)
        cv2.imshow("YOLOv5n Detection", display_frame)
        cv2.waitKey(1)

    if nearest_target is None:
        if detections and not any_depth_valid:
            if DEBUG_MODE:
                print("REJDBG,有检测但深度全无效,不会发送位置")
            return None, "DEPTH_INVALID", frame_fail_count, camera_state
        if DEBUG_MODE:
            print("REJDBG,未找到有效目标(可能未检测到或全部被过滤)")
        return None, None, frame_fail_count, camera_state

    nx, ny, nz_m, nlabel, nd_mm, nconf, nbbox, nvalid_ratio = nearest_target
    point3d = rs.rs2_deproject_pixel_to_point(intrinsics, [nx, ny], nz_m)
    cam_x, cam_y, cam_z = [p * 1000.0 for p in point3d]
    g_latest_cam_report = (cam_x, cam_y, cam_z, time.monotonic())

    if DEBUG_MODE:
        print(f"CAMDBG,{cam_x:.0f},{cam_y:.0f},{cam_z:.0f}")

    arm_point = get_grip_point_from_camera(
        cam_x,
        cam_y,
        cam_z,
        allow_x_out_of_range=allow_x_out_of_range,
    )
    if arm_point is None:
        if DEBUG_MODE:
            if not is_camera_target_grabbable(cam_x, cam_y, cam_z):
                print(
                    "REJDBG,相机坐标无效,cam=({:.0f},{:.0f},{:.0f}),要求有限数值且Z>0".format(
                        cam_x, cam_y, cam_z
                    )
                )
            else:
                arm_x_dbg, arm_y_dbg, arm_z_dbg = camera_to_arm(cam_x, cam_y, cam_z)
                print(
                    "REJDBG,机械臂不可达,arm=({:.0f},{:.0f},{:.0f}),范围X[{:.0f},{:.0f}] Y[{:.0f},{:.0f}] Z[{:.0f},{:.0f}]".format(
                        arm_x_dbg,
                        arm_y_dbg,
                        arm_z_dbg,
                        ARM_X_MIN,
                        ARM_X_MAX,
                        ARM_Y_MIN,
                        ARM_Y_MAX,
                        ARM_Z_MIN,
                        ARM_Z_MAX,
                    )
                )
        return None, None, frame_fail_count, camera_state

    arm_x, arm_y, arm_z = arm_point
    diameter_i = clamp_int(nd_mm, PI_DIAMETER_MIN_MM, PI_DIAMETER_MAX_MM)
    score_i = clamp_int(max(0.0, min(1.0, nconf)) * 100, PI_SCORE_MIN, PI_SCORE_MAX)

    result = {
        "type": nlabel,
        "x": arm_x,
        "y": arm_y,
        "z": arm_z,
        "diameter": diameter_i,
        "score": score_i,
        "bbox": nbbox,
        "valid_ratio": nvalid_ratio,
        "depth_mm": int(round(nz_m * 1000.0)),
        "frame_size": (w, h),
    }
    if DEBUG_MODE:
        print(f"ARMDBG,{arm_x},{arm_y},{arm_z}")
    return result, None, frame_fail_count, camera_state


def is_watch_candidate_ok(result):
    if result is None:
        return False
    if result["type"] not in ("BIG", "SMALL"):
        return False
    if not is_arm_yz_reachable(result["y"], result["z"]):
        return False
    if not is_arm_point_reachable(result["x"], result["y"], result["z"]):
        if not WATCH_STOP_ON_X_OUT_OF_RANGE:
            return False
        if not is_watch_x_stop_window(result["x"]):
            return False
    if WATCH_SEND_ANY_REACHABLE_ARM:
        return True

    min_score = WATCH_SCORE_MIN
    min_valid_ratio = DEPTH_VALID_RATIO_MIN
    edge_margin_px = EDGE_MARGIN_PX
    is_low_apple = LOW_APPLE_RELAX and is_low_apple_bbox(result["bbox"], result["frame_size"])
    if is_low_apple:
        min_score = LOW_APPLE_SCORE_MIN
        min_valid_ratio = LOW_APPLE_VALID_RATIO_MIN
        edge_margin_px = LOW_APPLE_EDGE_MARGIN_PX

    if result["score"] < min_score:
        return False
    if result["valid_ratio"] < min_valid_ratio:
        return False
    w, h = result["frame_size"]
    x1, y1, x2, y2 = result["bbox"]
    if is_bbox_near_edge_for_watch(x1, y1, x2, y2, w, h, edge_margin_px, allow_bottom_edge=is_low_apple):
        return False
    return True


def get_watch_reject_reason(result):
    if result is None:
        return "NO_TARGET"
    if result["type"] not in ("BIG", "SMALL"):
        return f"TYPE_{result['type']}"
    if not is_arm_yz_reachable(result["y"], result["z"]):
        return "YZ_OUT_OF_RANGE"
    if not is_arm_point_reachable(result["x"], result["y"], result["z"]):
        if not WATCH_STOP_ON_X_OUT_OF_RANGE:
            return "ARM_OUT_OF_RANGE"
        if not is_watch_x_stop_window(result["x"]):
            return "X_TOO_FAR"
        return "OK"
    if WATCH_SEND_ANY_REACHABLE_ARM:
        return "OK"

    min_score = WATCH_SCORE_MIN
    min_valid_ratio = DEPTH_VALID_RATIO_MIN
    edge_margin_px = EDGE_MARGIN_PX
    is_low_apple = LOW_APPLE_RELAX and is_low_apple_bbox(result["bbox"], result["frame_size"])
    if is_low_apple:
        min_score = LOW_APPLE_SCORE_MIN
        min_valid_ratio = LOW_APPLE_VALID_RATIO_MIN
        edge_margin_px = LOW_APPLE_EDGE_MARGIN_PX

    if result["score"] < min_score:
        return f"LOW_SCORE<{min_score}"
    if result["valid_ratio"] < min_valid_ratio:
        return "LOW_DEPTH_RATIO<{:.2f}".format(min_valid_ratio)
    w, h = result["frame_size"]
    x1, y1, x2, y2 = result["bbox"]
    if is_bbox_near_edge_for_watch(x1, y1, x2, y2, w, h, edge_margin_px, allow_bottom_edge=is_low_apple):
        return "NEAR_EDGE"
    return "OK"


def is_watch_candidate_stable(prev_result, curr_result):
    if prev_result is None or curr_result is None:
        return False
    if prev_result["type"] != curr_result["type"]:
        return False
    if abs(prev_result["depth_mm"] - curr_result["depth_mm"]) > DEPTH_JUMP_MAX_MM:
        return False
    if abs(prev_result["diameter"] - curr_result["diameter"]) > DIAMETER_JUMP_MAX_MM:
        return False
    if abs(prev_result["x"] - curr_result["x"]) > POS_JUMP_MAX_MM:
        return False
    if abs(prev_result["y"] - curr_result["y"]) > POS_JUMP_MAX_MM:
        return False
    if abs(prev_result["z"] - curr_result["z"]) > POS_JUMP_MAX_MM:
        return False
    return True


def get_watch_unstable_reason(prev_result, curr_result):
    if prev_result is None:
        return "FIRST_FRAME"
    if curr_result is None:
        return "NO_CURRENT"
    if prev_result["type"] != curr_result["type"]:
        return "TYPE_CHANGE"
    if abs(prev_result["depth_mm"] - curr_result["depth_mm"]) > DEPTH_JUMP_MAX_MM:
        return "DEPTH_JUMP"
    if abs(prev_result["diameter"] - curr_result["diameter"]) > DIAMETER_JUMP_MAX_MM:
        return "DIAMETER_JUMP"
    if abs(prev_result["x"] - curr_result["x"]) > POS_JUMP_MAX_MM:
        return "X_JUMP"
    if abs(prev_result["y"] - curr_result["y"]) > POS_JUMP_MAX_MM:
        return "Y_JUMP"
    if abs(prev_result["z"] - curr_result["z"]) > POS_JUMP_MAX_MM:
        return "Z_JUMP"
    return "STABLE"


def handle_command(
    line,
    session,
    camera_state,
    frame_fail_count,
    watch_mode,
    watch_seq,
    watch_tree_id,
    watch_view_id,
    watch_hit_sent,
    watch_candidate,
    watch_stable_count,
    canceled_scan_seqs,
    scan_in_progress,
    scan_seq,
    scan_deadline,
):
    parts = [p.strip() for p in line.strip().split(",") if p.strip()]
    if not parts:
        return (
            None,
            frame_fail_count,
            camera_state,
            watch_mode,
            watch_seq,
            watch_tree_id,
            watch_view_id,
            watch_hit_sent,
            watch_candidate,
            watch_stable_count,
            canceled_scan_seqs,
            scan_in_progress,
            scan_seq,
            scan_deadline,
        )

    cmd = parts[0].upper()
    if cmd == "PING" and len(parts) == 2:
        seq = parts[1]
        debug_event("PING", seq=seq)
        return (
            f"PONG,{seq}\n",
            frame_fail_count,
            camera_state,
            watch_mode,
            watch_seq,
            watch_tree_id,
            watch_view_id,
            watch_hit_sent,
            watch_candidate,
            watch_stable_count,
            canceled_scan_seqs,
            scan_in_progress,
            scan_seq,
            scan_deadline,
        )

    if cmd == "WATCH" and len(parts) >= 4:
        seq = parts[1]
        watch_mode = True
        watch_seq = seq
        watch_tree_id = parts[2]
        watch_view_id = parts[3]
        watch_hit_sent = False
        watch_candidate = None
        watch_stable_count = 0
        debug_event("WATCH_START", seq=seq, tree_id=watch_tree_id, view_id=watch_view_id)
        return (
            f"WATCHING,{seq}\n",
            frame_fail_count,
            camera_state,
            watch_mode,
            watch_seq,
            watch_tree_id,
            watch_view_id,
            watch_hit_sent,
            watch_candidate,
            watch_stable_count,
            canceled_scan_seqs,
            scan_in_progress,
            scan_seq,
            scan_deadline,
        )

    if cmd == "WATCH_STOP" and len(parts) == 2:
        seq = parts[1]
        if watch_mode and watch_seq == seq:
            watch_mode = False
            watch_hit_sent = False
            watch_candidate = None
            watch_stable_count = 0
            debug_event("WATCH_STOP", seq=seq, matched=1)
        else:
            debug_event("WATCH_STOP", seq=seq, matched=0, active_seq=watch_seq)
        return (
            None,
            frame_fail_count,
            camera_state,
            watch_mode,
            watch_seq,
            watch_tree_id,
            watch_view_id,
            watch_hit_sent,
            watch_candidate,
            watch_stable_count,
            canceled_scan_seqs,
            scan_in_progress,
            scan_seq,
            scan_deadline,
        )

    if cmd == "SCAN" and len(parts) >= 4:
        seq = parts[1]
        canceled_scan_seqs.discard(seq)
        watch_mode = False
        watch_hit_sent = False
        watch_candidate = None
        watch_stable_count = 0
        scan_in_progress = True
        scan_seq = seq
        scan_deadline = time.monotonic() + SCAN_TIMEOUT_S
        debug_event("SCAN_START", seq=seq, tree_id=parts[2], view_id=parts[3], timeout_s=SCAN_TIMEOUT_S)
        return (
            None,
            frame_fail_count,
            camera_state,
            watch_mode,
            watch_seq,
            watch_tree_id,
            watch_view_id,
            watch_hit_sent,
            watch_candidate,
            watch_stable_count,
            canceled_scan_seqs,
            scan_in_progress,
            scan_seq,
            scan_deadline,
        )
    if cmd == "SCAN_CANCEL" and len(parts) >= 2:
        seq = parts[1]
        canceled_scan_seqs.add(seq)
        was_active = scan_in_progress and scan_seq == seq
        if scan_in_progress and scan_seq == seq:
            scan_in_progress = False
            scan_seq = None
            scan_deadline = 0.0
        debug_event("SCAN_CANCEL", seq=seq, active=1 if was_active else 0)
        return (
            None,
            frame_fail_count,
            camera_state,
            watch_mode,
            watch_seq,
            watch_tree_id,
            watch_view_id,
            watch_hit_sent,
            watch_candidate,
            watch_stable_count,
            canceled_scan_seqs,
            scan_in_progress,
            scan_seq,
            scan_deadline,
        )
    debug_event("UNKNOWN_CMD", line=line)
    return (
        None,
        frame_fail_count,
        camera_state,
        watch_mode,
        watch_seq,
        watch_tree_id,
        watch_view_id,
        watch_hit_sent,
        watch_candidate,
        watch_stable_count,
        canceled_scan_seqs,
        scan_in_progress,
        scan_seq,
        scan_deadline,
    )


def send_latest_cam_report(ser, last_sent_detect_ts):
    if g_latest_cam_report is None:
        return last_sent_detect_ts

    cam_x, cam_y, cam_z, detected_ts = g_latest_cam_report
    if detected_ts <= last_sent_detect_ts:
        return last_sent_detect_ts

    reply = "CAM,cam=({:.0f},{:.0f},{:.0f})\n".format(cam_x, cam_y, cam_z)
    ser.write(reply.encode("utf-8"))
    if DEBUG_MODE:
        print(f"TX:{reply.strip()}")
    return detected_ts


def serial_loop():
    print("正在扫描系统中的串口设备...")
    port_name = find_ch340_port()
    if port_name is None:
        print("未检测到 CH340 设备，请检查 USB 物理连接。")
        sys.exit(1)

    print(f"检测到 CH340 设备，当前分配的端口为: {port_name}")
    print(f"正在尝试以 {SERIAL_BAUD_RATE} 波特率打开端口 {port_name}...")

    try:
        ser = serial.Serial(port_name, SERIAL_BAUD_RATE, timeout=SERIAL_READ_TIMEOUT_S)
    except serial.SerialException as e:
        if "Permission denied" in str(e):
            print(f"权限不足：无法打开设备节点 {port_name}。")
            print("请执行 `sudo usermod -aG dialout $USER` 后重启或注销当前会话，再重新运行此脚本。")
        else:
            print(f"串口通信发生错误: {e}")
        sys.exit(1)

    print("串口打开成功。等待 STM32 命令...\n")
    if DEBUG_MODE:
        print(
            "SERDBG,OPEN,PORT={port},BAUD={baud},BYTESIZE=8,PARITY=N,STOPBITS=1".format(
                port=port_name, baud=SERIAL_BAUD_RATE
            )
        )

    print(
        "运行模式: {}".format(
            "窗口预览模式" if SHOW_WINDOW else "无窗口后台模式"
        )
    )
    if not SHOW_WINDOW:
        print("OpenCV 窗口已禁用：适合 systemd 服务/无桌面环境运行。")

    session = init_onnx_session()
    camera_state = init_camera()
    if camera_state is None:
        print("相机初始化失败，程序退出。")
        ser.close()
        return

    if SHOW_WINDOW:
        cv2.namedWindow("YOLOv5n Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLOv5n Detection", 640, 480)

    frame_fail_count = 0
    last_preview_time = 0.0
    last_watch_time = 0.0
    last_watch_debug_time = 0.0
    last_cam_report_time = 0.0
    last_scan_wait_debug_time = 0.0
    last_sent_cam_detect_ts = 0.0
    watch_mode = False
    watch_seq = None
    watch_tree_id = None
    watch_view_id = None
    watch_hit_sent = False
    watch_candidate = None
    watch_stable_count = 0
    canceled_scan_seqs = set()
    scan_in_progress = False
    scan_seq = None
    scan_deadline = 0.0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                if SHOW_WINDOW and window_closed():
                    print("窗口已关闭，程序退出。")
                    break

                now_ts = time.monotonic()

                # 无论 idle / SCAN / WATCH / HIT 后等待，都定时刷新预览窗口。
                # 否则 WATCH 命中后 watch_hit_sent=True，主循环会一直留在 watch_mode 分支，
                # 原来的 idle 预览不会执行，窗口看起来就像“相机卡死”。
                if (SHOW_WINDOW and PREVIEW_WHEN_IDLE and
                        (not scan_in_progress) and
                        (not (watch_mode and watch_hit_sent)) and
                        now_ts - last_preview_time >= PREVIEW_MIN_INTERVAL_S):
                    preview_latest_frame_only(camera_state)
                    last_preview_time = now_ts

                if camera_state:
                    grabber = camera_state.get("grabber")
                    stats = grabber.get_stats() if grabber else {}
                    last_frame_ts = stats.get("last_frame_ts", now_ts) or now_ts
                    last_warn_ts = camera_state.get("last_warn_ts", 0.0)
                    age = now_ts - last_frame_ts
                    if age > CAMERA_STALL_TIMEOUT_S and now_ts - last_warn_ts > CAMERA_WARN_INTERVAL_S:
                        if DEBUG_MODE:
                            print(
                                "REJDBG,相机线程真正长时间未更新帧; "
                                f"age={age:.2f}s,running={stats.get('running')},"
                                f"frame_index={stats.get('frame_index')},"
                                f"total_no_frame={stats.get('no_frame_count')},"
                                f"consecutive_no_frame={stats.get('consecutive_no_frame_count')},"
                                f"last_error={stats.get('last_error')}"
                            )
                        camera_state["last_warn_ts"] = now_ts

                if (not scan_in_progress) and (not watch_mode) and (now_ts - last_cam_report_time >= CAM_REPORT_INTERVAL_S):
                    result, err, frame_fail_count, camera_state = find_best_target_once(
                        session, camera_state, frame_fail_count
                    )
                    last_cam_report_time = now_ts
                    if err not in ("FRAME_TIMEOUT", "DEPTH_INVALID"):
                        last_sent_cam_detect_ts = send_latest_cam_report(ser, last_sent_cam_detect_ts)

                if scan_in_progress:
                    if now_ts >= scan_deadline:
                        debug_event("SCAN_TIMEOUT_LOCAL", seq=scan_seq, timeout_s=SCAN_TIMEOUT_S)
                        reply = f"NONE,{scan_seq},NO_TARGET\n"
                        tx_line(ser, reply)
                        debug_event("SCAN_NONE", seq=scan_seq, reason="LOCAL_TIMEOUT")
                        scan_in_progress = False
                        scan_seq = None
                        scan_deadline = 0.0
                        continue

                    result, err, frame_fail_count, camera_state = find_best_target_once(
                        session, camera_state, frame_fail_count
                    )
                    if err in ("FRAME_TIMEOUT", "DEPTH_INVALID"):
                        if now_ts - last_scan_wait_debug_time >= SCAN_WAIT_DEBUG_INTERVAL_S:
                            debug_event("SCAN_WAIT_FRAME", seq=scan_seq, err=err)
                            last_scan_wait_debug_time = now_ts
                        continue
                    if err:
                        reply = f"NONE,{scan_seq},NO_TARGET\n"
                        tx_line(ser, reply)
                        debug_event("SCAN_SOFT_NONE", seq=scan_seq, err=err)
                        scan_in_progress = False
                        scan_seq = None
                        scan_deadline = 0.0
                        continue
                    if scan_seq in canceled_scan_seqs:
                        debug_event("SCAN_CANCELLED_DROP", seq=scan_seq)
                        canceled_scan_seqs.discard(scan_seq)
                        scan_in_progress = False
                        scan_seq = None
                        scan_deadline = 0.0
                        continue
                    if time.monotonic() >= scan_deadline:
                        debug_event("SCAN_TIMEOUT_LOCAL", seq=scan_seq, timeout_s=SCAN_TIMEOUT_S)
                        reply = f"NONE,{scan_seq},NO_TARGET\n"
                        tx_line(ser, reply)
                        debug_event("SCAN_NONE", seq=scan_seq, reason="LOCAL_TIMEOUT")
                        scan_in_progress = False
                        scan_seq = None
                        scan_deadline = 0.0
                        continue
                    if result is None:
                        reply = f"NONE,{scan_seq},NO_TARGET\n"
                        tx_line(ser, reply)
                        debug_event("SCAN_NONE", seq=scan_seq, reason="NO_TARGET")
                        scan_in_progress = False
                        scan_seq = None
                        scan_deadline = 0.0
                        continue
                    reply = (
                        f"FRUIT,{scan_seq},{result['type']},{result['x']},{result['y']},{result['z']},"
                        f"{result['diameter']},{result['score']}\n"
                    )
                    tx_line(ser, reply)
                    debug_result_event("SCAN_FRUIT", seq=scan_seq, result=result)
                    scan_in_progress = False
                    scan_seq = None
                    scan_deadline = 0.0
                    continue

                if watch_mode:
                    if now_ts - last_watch_time >= WATCH_MIN_INTERVAL_S and not watch_hit_sent:
                        result, err, frame_fail_count, camera_state = find_best_target_once(
                            session,
                            camera_state,
                            frame_fail_count,
                            allow_x_out_of_range=True,
                        )
                        last_watch_time = now_ts
                        if err:
                            if now_ts - last_watch_debug_time >= WATCH_DEBUG_INTERVAL_S:
                                debug_event("WATCH_SOFT_ERR", seq=watch_seq, err=err)
                                last_watch_debug_time = now_ts
                            watch_candidate = None
                            watch_stable_count = 0
                            continue
                        reject_reason = get_watch_reject_reason(result)
                        if result is None or reject_reason != "OK":
                            if now_ts - last_watch_debug_time >= WATCH_DEBUG_INTERVAL_S:
                                debug_result_event("WATCH_REJECT", seq=watch_seq, result=result, reason=reject_reason)
                                last_watch_debug_time = now_ts
                            watch_candidate = None
                            watch_stable_count = 0
                            continue

                        unstable_reason = get_watch_unstable_reason(watch_candidate, result)
                        if unstable_reason == "STABLE":
                            watch_stable_count += 1
                        else:
                            watch_stable_count = 1
                        watch_candidate = result
                        if now_ts - last_watch_debug_time >= WATCH_DEBUG_INTERVAL_S:
                            debug_result_event(
                                "WATCH_CANDIDATE",
                                seq=watch_seq,
                                result=result,
                                stable=watch_stable_count,
                                stable_need=WATCH_STABLE_FRAMES,
                                stability=unstable_reason,
                            )
                            last_watch_debug_time = now_ts

                        hit_result, zone, hit_mode = build_watch_hit_result(result)
                        if hit_result is None or zone != "GOOD":
                            watch_stable_count = 0
                            debug_result_event(
                                "WATCH_ZONE_REJECT",
                                seq=watch_seq,
                                result=result,
                                zone=zone,
                                mode=hit_mode,
                            )
                            continue

                        if watch_stable_count >= WATCH_STABLE_FRAMES:
                            reply = (
                                f"HIT,{watch_seq},{hit_result['type']},{hit_result['x']},{hit_result['y']},{hit_result['z']},"
                                f"{hit_result['diameter']},{hit_result['score']},{watch_stable_count},{zone}\n"
                            )
                            tx_line(ser, reply)
                            debug_result_event(
                                "HIT_SENT",
                                seq=watch_seq,
                                result=result,
                                hit_arm="{}/{}/{}".format(hit_result["x"], hit_result["y"], hit_result["z"]),
                                stable=watch_stable_count,
                                zone=zone,
                                mode=hit_mode,
                            )
                            watch_hit_sent = True
                    continue

                # 预览刷新已经在本轮循环前面统一执行，这里不再只在 idle 状态刷新。
                continue
            if DEBUG_MODE:
                print(f"SERDBG,RX_RAW,{len(raw)}B,{raw.hex()}")
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            if not line:
                continue

            if DEBUG_MODE:
                print(f"RX:{line}")

            (
                reply,
                frame_fail_count,
                camera_state,
                watch_mode,
                watch_seq,
                watch_tree_id,
                watch_view_id,
                watch_hit_sent,
                watch_candidate,
                watch_stable_count,
                canceled_scan_seqs,
                scan_in_progress,
                scan_seq,
                scan_deadline,
            ) = handle_command(
                line,
                session,
                camera_state,
                frame_fail_count,
                watch_mode,
                watch_seq,
                watch_tree_id,
                watch_view_id,
                watch_hit_sent,
                watch_candidate,
                watch_stable_count,
                canceled_scan_seqs,
                scan_in_progress,
                scan_seq,
                scan_deadline,
            )
            if reply:
                ser.write(reply.encode("utf-8"))
                if DEBUG_MODE:
                    print(f"TX:{reply.strip()}")

    except KeyboardInterrupt:
        print("\n监视已被用户中止 (Ctrl+C)。")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        if camera_state and camera_state.get("grabber"):
            try:
                camera_state["grabber"].stop()
            except Exception:
                pass
        if SHOW_WINDOW:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        print("串口已安全关闭。")


def main():
    serial_loop()


if __name__ == "__main__":
    main()
