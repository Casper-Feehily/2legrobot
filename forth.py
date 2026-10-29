"""LS-6A 原厂固件循迹：1号完整前进步态的衔接版。

主板中需已下载 0=立正、1=前进、3=左转、4=右转。
在 OpenMV IDE 打开整个文件运行；先预览，确认识线后改 ENABLE_MOTION=True。
直行时每次播放1号一次，周期之间不插入0号立正。
默认在完整前进结束后直接接转向；关闭 DIRECT_TURN_TRANSITION 可恢复立正过渡。
持续丢线仍等当前前进周期结束后立正。
三框至少两处可信纵向线才允许行走；横线污染的框不作为方向中心。
转向结束后，实际近处线已回正时，先前进一周期，减少重复原地转向。
使用已验证的 UART3 / 9600 / 8N1 接线；不修改主板固件。
本版按原始动作时间计时，没有动作完成回读；周期衔接会有短小间隔，
并非主板内无限循环。动作文件时长修改后应同步修改 ACTION_TIME_MS。
"""

import time

# 实测建议：先关闭运动调好画面、识线区域和黑色阈值，再校准中心与左右方向；
# 开启运动后，每次只调整一个参数，依次观察转向门槛、稳定性和动作衔接。
# 图像固定为 160×120；以下位置、宽高和偏差均为图像像素，时间均为毫秒。

# 运动与画面方向
ENABLE_MOTION = True  # False：仅预览、不发送动作；True：延时后先立正，再开始循迹。
BLACK_THRESHOLD = (0, 64)  # 黑色灰度范围，0 最黑、255 最白；通常保持下限 0，仅调上限。
# 上限调大可识别较浅的线，但易混入阴影；调小筛选更严格，但可能漏掉黑线。

H_MIRROR = False  # 水平镜像；用于匹配摄像头安装方向，切换后需重新核对左右判断。
V_FLIP = False  # 垂直翻转；先调正画面，再设置近、远 ROI 的位置。
REVERSE_TURNS = False  # True：交换左转 3 和右转 4；用于修正判断方向与实际转向相反。

# 目标中心与转向灵敏度
LINE_CENTER_X = 80  # 期望黑线所在的横坐标，可校准摄像头安装偏移；允许 1～158。
TURN_THRESHOLD = 12  # 加权偏差超过 ±此值才转向，等于边界仍直行；允许 1～79。
# 调小更敏感、易左右摆动；调大直行更稳，但可能转弯偏晚；不改变单次转向角度。
POST_TURN_CENTER_THRESHOLD = 12  # 转向结束后，近处线距中心不超过此像素数则先前进一步。
# 必须至少两框可信且近处实际识线；仅在转向后生效，前进一步后恢复加权判断。
# 调大可更早结束原地转向，但允许的偏差也更大；调小对正更严格，可能需要更多次转向。

# 黑线识别与干扰过滤
ROIS = ((0, 95, 160, 20), (0, 65, 160, 20), (0, 35, 160, 20))  # 近、中、远。
# 每项为 (x, y, 宽, 高)，y 越大越靠下；三框应互不重叠且都在 160×120 内。
# 一个框被横线遮挡时使用另外两个；不能只凭一个框或历史方向盲走。
# 调整位置以适应摄像头俯角和赛道；加大区域可覆盖更多画面，也可能增加干扰和耗时。
MIN_PIXELS = 20  # 黑块像素数量及外接矩形面积的筛选下限；调大滤噪，调小保留细线。
MIN_BLOB_WIDTH = 2  # 黑块最小宽度；调大可滤细小干扰，但容易漏掉远处较细的线。
MAX_BLOB_WIDTH = 65  # 黑块最大宽度；调小排除大片黑影，调大容纳宽线或弯道。
MIN_BLOB_HEIGHT = 4  # 黑块最小高度；调大滤掉矮小噪点，但线刚进入 ROI 时容易漏检。
RECOVERY_STRIP_HEIGHT = 4  # 宽黑块干扰且不足两框时，重查该框上下各此高度的窄带。

MAX_CENTER_JUMP = 60  # 同一帧近、远黑线中心允许的最大横向差，允许 1～160。
# 以完整近远纵向间距折算；中框和窄带按实际间距缩小限值，避免拼接不相关黑块。


MAX_FRAME_MS = 150  # 单帧处理耗时及相邻判断间隔的上限，必须大于 0。
# 单帧超时按丢线处理，判断间隔超时则重置稳定累计；太小易中断，太大可能接受旧画面。

# 主板动作时长：资料原始 XML 的各帧时间合计；不是 7 号前进一步的时长。
# 动作号：0=立正，1=完整前进步态，3=左转，4=右转。
# 必须与主板实际动作文件时长一致，修改动作文件后同步修改；缩短这里不会让动作加速，
# 只会让程序更早发送下一指令，可能干扰尚未完成的动作；本程序没有动作完成回读。
ACTION_TIME_MS = {0: 1000, 1: 1300, 3: 750, 4: 750}
CYCLE_MARGIN_MS = (
    70  # 前进时长之外的额外余量；调大衔接更保守、间隔更长，调小可能过早发令。
)
STRAIGHT_MARGIN_MS = 40  # 三框确认直道时的前进余量；只减少间隔，不缩短步态本身。
DIRECT_TURN_TRANSITION = True  # True：完整前进后直接转向；False：先执行立正过渡。
TURN_SETTLE_MS = (
    100  # 转向完成后的站稳等待；此期间重新识线，等待结束且判断稳定后才发令。
)
STAND_SETTLE_MS = (
    100  # 立正完成后的站稳等待；同时积累识线结果，避免结束后再额外等一次。
)
DECISION_GRACE_MS = (
    200  # 前进周期结束且无稳定方向时的额外确认时间；有稳定转向时不等待。
)
# 调大可容忍短暂丢线或方向变化，但持续丢线后立正也会更晚；不会中途打断当前步态。

# 方向判断稳定性：连续帧数和持续时间必须同时满足；丢线立即清空累计。
STABLE_FRAMES = 2  # 连续相同方向的最少帧数，至少为 1；调大抗抖，调小反应快。
STABLE_MS = 40  # 同一方向至少持续的时间；调大抗抖，调小反应快，须同时满足帧数条件。
# 直道判定依次限制：近处偏差、近远横向差、中点到近远连线的横向偏差（像素）。
STRAIGHT_ENTER_LIMITS = (8, 8, 4)
STRAIGHT_EXIT_LIMITS = (12, 14, 8)  # 退出阈值较宽，避免在边界来回切换模式。
STRAIGHT_STABLE_FRAMES = 3  # 只有三框均可信且无横线/重检，才累计直道确认。
STRAIGHT_STABLE_MS = 100  # 进入直道须同时满足帧数与持续时间；有干扰立即退出快模式。
START_DELAY_MS = 3000  # 开启运动后、首次发送立正前的准备时间；按实际摆放所需时间调整。
ACTION_NAMES = {
    0: "STAND",
    1: "FORWARD",
    3: "LEFT",
    4: "RIGHT",
}  # 动作号与显示名称；非灵敏度参数。
# 此映射也用于检查动作号；仅修改名称不能改变主板动作，动作号应与主板预存动作一致。


def validate_config():
    if not 0 <= BLACK_THRESHOLD[0] < BLACK_THRESHOLD[1] <= 255:
        raise ValueError("Invalid BLACK_THRESHOLD")
    if not 1 <= LINE_CENTER_X <= 158 or not 1 <= TURN_THRESHOLD <= 79:
        raise ValueError("Invalid center or turn threshold")
    if not 1 <= POST_TURN_CENTER_THRESHOLD <= 79:
        raise ValueError("Invalid post-turn center threshold")
    if len(ROIS) != 3:
        raise ValueError("Use three ROIs: near, middle, far")
    for x, y, w, h in ROIS:
        if min(x, y) < 0 or min(w, h) <= 0 or x + w > 160 or y + h > 120:
            raise ValueError("ROI must fit 160x120")
    for near, far in zip(ROIS, ROIS[1:]):
        if far[1] + far[3] > near[1]:
            raise ValueError("ROIs must be ordered near to far without overlap")
    if not 1 <= RECOVERY_STRIP_HEIGHT <= min(r[3] // 2 for r in ROIS):
        raise ValueError("Invalid recovery strip height")
    if not 1 <= MIN_BLOB_WIDTH <= MAX_BLOB_WIDTH <= 160:
        raise ValueError("Invalid blob widths")
    if MIN_PIXELS < 1 or not 1 <= MIN_BLOB_HEIGHT <= 120:
        raise ValueError("Invalid blob filters")
    if not 1 <= MAX_CENTER_JUMP <= 160 or MAX_FRAME_MS <= 0:
        raise ValueError("Invalid frame filters")
    if any(ACTION_TIME_MS.get(a, 0) <= 0 for a in ACTION_NAMES):
        raise ValueError("Set positive action durations")
    if not 0 <= STRAIGHT_MARGIN_MS <= CYCLE_MARGIN_MS:
        raise ValueError("Invalid straight margin")
    if len(STRAIGHT_ENTER_LIMITS) != 3 or len(STRAIGHT_EXIT_LIMITS) != 3:
        raise ValueError("Set three straight thresholds")
    if any(not 0 <= a <= b <= 160 for a, b in zip(STRAIGHT_ENTER_LIMITS, STRAIGHT_EXIT_LIMITS)):
        raise ValueError("Invalid straight hysteresis")
    if STRAIGHT_STABLE_FRAMES < 1 or STRAIGHT_STABLE_MS < 0 or START_DELAY_MS < 0:
        raise ValueError("Invalid confirmation or startup timing")
    if (
        min(
            CYCLE_MARGIN_MS,
            TURN_SETTLE_MS,
            STAND_SETTLE_MS,
            DECISION_GRACE_MS,
            STABLE_MS,
        )
        < 0
        or STABLE_FRAMES < 1
    ):
        raise ValueError("Invalid timing settings")


def action_packet(action_id):
    if action_id not in ACTION_NAMES:
        raise ValueError("Use action 0, 1, 3 or 4")
    return bytes((0x55, 0x55, 0x05, 0x06, action_id, 0x01, 0x00))


def roi_y(index):
    return ROIS[index][1] + ROIS[index][3] / 2


def compatible_points(a, b):
    # 同一帧内方向约束：距离越近，允许的中心差越小。
    return a[1] != b[1] and abs(a[0] - b[0]) * (roi_y(0) - roi_y(2)) <= (
        MAX_CENTER_JUMP * abs(a[1] - b[1])
    )


def project_x(a, b, y):
    return a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1])


class TrackObservation:
    """一帧的实际观测与补算坐标；补算不增加可信框数量。"""

    def __init__(self, points=(None, None, None), wide=(False, False, False),
                 recovered=(False, False, False)):
        self.points = tuple(points)
        self.wide = tuple(wide)
        self.recovered = tuple(recovered)
        self.near_x, self.far_x = None, None
        valid = [p for p in self.points if p is not None]
        if len(valid) < 2:
            return
        if any(not (0 <= p[0] < 160 and 0 <= p[1] < 120) for p in valid):
            return
        for i, a in enumerate(valid):
            if any(not compatible_points(a, b) for b in valid[i + 1:]):
                return
        near = self.points[0][0] if self.points[0] else project_x(valid[0], valid[-1], roi_y(0))
        far = self.points[2][0] if self.points[2] else project_x(valid[0], valid[-1], roi_y(2))
        if 0 <= near < 160 and 0 <= far < 160 and abs(near - far) <= MAX_CENTER_JUMP:
            self.near_x, self.far_x = near, far


def find_candidates(img, roi, strip=False):
    # 窄带只有几行，像素下限需同步缩小，否则合法细线也会被过滤。
    minimum = min(MIN_PIXELS, MIN_BLOB_WIDTH * roi[3]) if strip else MIN_PIXELS
    blobs = img.find_blobs(
        [BLACK_THRESHOLD], roi=roi, pixels_threshold=minimum,
        area_threshold=minimum, merge=True,
    )
    height = roi[3] if strip else MIN_BLOB_HEIGHT
    return (
        [(b.cx, b.cy) for b in blobs
         if MIN_BLOB_WIDTH <= b.w <= MAX_BLOB_WIDTH and b.h >= height],
        any(b.w > MAX_BLOB_WIDTH for b in blobs),
    )


def pick_point(candidates, known):
    valid = [p for p in known if p is not None]
    reference = valid[-1][0] if valid else LINE_CENTER_X
    candidates = [p for p in candidates if all(compatible_points(p, q) for q in valid)]
    return min(candidates, key=lambda p: abs(p[0] - reference)) if candidates else None


def detect_line(img):
    points, wide, recovered = [], [], [False, False, False]
    for roi in ROIS:
        candidates, interference = find_candidates(img, roi)
        points.append(pick_point(candidates, points))
        wide.append(interference)

    if sum(p is not None for p in points) < 2:
        for index, (x, y, w, h) in enumerate(ROIS):
            if points[index] is not None or not wide[index]:
                continue
            edges = []
            for top in (y, y + h - RECOVERY_STRIP_HEIGHT):
                candidates, _ = find_candidates(
                    img, (x, top, w, RECOVERY_STRIP_HEIGHT), strip=True
                )
                point = pick_point(candidates, points + edges)
                if point is not None:
                    edges.append(point)
            if edges:
                points[index] = (sum(p[0] for p in edges) / len(edges),
                                 sum(p[1] for p in edges) / len(edges))
                recovered[index] = True
    return TrackObservation(points, wide, recovered)


def draw_tracking(img, track):
    # 所有检测和重检结束后才绘制，避免边框及文字污染窄带识别。
    for roi, point, wide in zip(ROIS, track.points, track.wide):
        img.draw_rectangle(roi, color=200 if wide else 100)
        if point is not None:
            img.draw_cross((int(point[0]), int(point[1])), color=255)
    img.draw_line((LINE_CENTER_X, 30, LINE_CENTER_X, 119), color=180)
    flags = "".join("R" if r else "X" if w else "+" if p else "-"
                    for p, w, r in zip(track.points, track.wide, track.recovered))
    img.draw_string((0, 20), "roi:%d/3 %s" % (
        sum(p is not None for p in track.points), flags), color=255)


def choose_action(near_x, far_x, after_turn=False):
    if near_x is None or far_x is None:
        return None, None
    # 近处偏差为主，远处用于提前判断弯道；单位均为像素。
    # 0.65 为近处权重，0.35 为远处权重；调整时建议两者之和保持为 1。
    # 增大近处权重更关注脚前位置；增大远处权重有助提前判弯，也更受远处误识别影响。
    error = 0.65 * (near_x - LINE_CENTER_X) + 0.35 * (far_x - LINE_CENTER_X)
    # 已完成一次转向时，近处对正就先前进，避免远处仍是弯道而反复原地转向。
    # error 仍保留加权偏差供画面显示；只有此分支按近处偏差判断能否恢复前进。
    if after_turn and abs(near_x - LINE_CENTER_X) <= POST_TURN_CENTER_THRESHOLD:
        action = 1
    elif error < -TURN_THRESHOLD:
        action = 3
    elif error > TURN_THRESHOLD:
        action = 4
    else:
        action = 1
    if REVERSE_TURNS and action != 1:
        action = 7 - action  # 3和4交换
    return action, error


class LineFilter:
    """只返回近期连续稳定的方向；丢线立即使当前判断无效。"""

    def __init__(self):
        self.candidate = None
        self.count = 0
        self.first_seen = None
        self.last_seen = None

    def update(self, action, now):
        interrupted = (
            self.last_seen is not None
            and time.ticks_diff(now, self.last_seen) > MAX_FRAME_MS
        )
        self.last_seen = now
        if action is None:
            self.candidate, self.count, self.first_seen = None, 0, None
            return None
        if interrupted or action != self.candidate:
            self.candidate, self.count, self.first_seen = action, 1, now
        else:
            self.count += 1
        if (
            self.count >= STABLE_FRAMES
            and time.ticks_diff(now, self.first_seen) >= STABLE_MS
        ):
            return action
        return None


class RoadModeFilter:
    """直道须持续确认；弯道、遮挡和丢线立即撤销快速衔接。"""

    def __init__(self):
        self.mode = "CURVE"
        self.count = 0
        self.first_seen = None
        self.last_seen = None

    def update(self, track, now):
        if self.last_seen is not None and time.ticks_diff(now, self.last_seen) > MAX_FRAME_MS:
            self.mode, self.count, self.first_seen = "CURVE", 0, None
        self.last_seen = now
        if track is None or track.near_x is None:
            mode = "LOST"
        elif any(track.wide) or any(track.recovered):
            mode = "CROSS"
        elif any(p is None for p in track.points):
            mode = "CURVE"
        else:
            near, middle, far = track.points
            measures = (abs(near[0] - LINE_CENTER_X), abs(near[0] - far[0]),
                        abs(middle[0] - project_x(near, far, middle[1])))
            if self.mode == "STRAIGHT" and all(
                value <= limit for value, limit in zip(measures, STRAIGHT_EXIT_LIMITS)
            ):
                return self.mode
            if all(value <= limit for value, limit in zip(measures, STRAIGHT_ENTER_LIMITS)):
                if self.count == 0:
                    self.first_seen = now
                self.count += 1
                self.mode = "CURVE"
                if (self.count >= STRAIGHT_STABLE_FRAMES
                        and time.ticks_diff(now, self.first_seen) >= STRAIGHT_STABLE_MS):
                    self.mode = "STRAIGHT"
                return self.mode
            mode = "CURVE"
        self.mode, self.count, self.first_seen = mode, 0, None
        return self.mode


class GaitController:
    def __init__(self):
        self.state = "IDLE"
        self.sent_at = None
        self.duration = 0
        self.last_interval_ms = 0

    def elapsed(self, now):
        return 0 if self.sent_at is None else time.ticks_diff(now, self.sent_at)

    def next_action(self, stable_action, now, road_mode="CURVE"):
        ready_after = self.duration
        if self.state == "WALK" and stable_action == 1 and road_mode == "STRAIGHT":
            ready_after = ACTION_TIME_MS[1] + STRAIGHT_MARGIN_MS
        if self.state != "IDLE" and self.elapsed(now) < ready_after:
            return None
        if self.state == "WALK":
            if stable_action == 1:
                return 1  # 继续完整步态，不调用立正
            if stable_action in (3, 4) and DIRECT_TURN_TRANSITION:
                return stable_action  # 当前完整步态已结束，不再插入一秒立正
            if (
                stable_action is None
                and self.elapsed(now) < self.duration + DECISION_GRACE_MS
            ):
                return None
            return 0  # 关闭直接衔接时换方向，或持续丢线：周期结束后立正一次
        # 动作完整执行后，在额外站稳时间内累计新判断；站稳时间结束前不会发令。
        if stable_action in (1, 3, 4):
            return stable_action
        return None

    def mark_sent(self, action, now):
        self.last_interval_ms = self.elapsed(now)
        self.sent_at = now
        if action == 1:
            self.state = "WALK"
            self.duration = ACTION_TIME_MS[1] + CYCLE_MARGIN_MS
        elif action == 0:
            self.state = "STAND"
            self.duration = ACTION_TIME_MS[0] + STAND_SETTLE_MS
        else:
            self.state = "TURN"
            self.duration = ACTION_TIME_MS[action] + TURN_SETTLE_MS

    def discard_observation(self, frame_started):
        # 只丢弃立正/转向动作尚未执行完时开始采集的画面；站稳等待可与识线累计重叠。
        if self.state == "STAND":
            action_duration = self.duration - STAND_SETTLE_MS
        elif self.state == "TURN":
            action_duration = self.duration - TURN_SETTLE_MS
        else:
            return False
        return self.elapsed(frame_started) < action_duration


def run():
    validate_config()
    import sensor

    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(
        sensor.QQVGA
    )  # 固定 160×120；更改分辨率需同步调整 ROI、中心和像素门槛。
    sensor.set_hmirror(H_MIRROR)
    sensor.set_vflip(V_FLIP)
    sensor.skip_frames(time=1500)  # 上电后等待 2000 ms，让相机自动参数先稳定。
    # 等待后关闭自动增益、白平衡和曝光，减少识线亮度变化；换光照后需重新检查黑色阈值。
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)

    uart = None
    controller = GaitController()
    if ENABLE_MOTION:
        from pyb import UART

        # UART3、9600 波特率、8 数据位、无校验、1 停止位；须与接线及主板串口配置一致。
        # timeout=100、timeout_char=10 为 UART 超时配置，单位 ms；不控制行走速度或转向角度。
        uart = UART(3, 9600, bits=8, parity=None, stop=1, timeout=100, timeout_char=10)
        print("MOTION: stand first, then follow line. Prepare for 3 seconds.")
        time.sleep_ms(START_DELAY_MS)
        packet = action_packet(0)
        if uart.write(packet) != len(packet):
            raise OSError("UART short write; check robot before restarting")
        controller.mark_sent(0, time.ticks_ms())
    else:
        print("PREVIEW only. Check roi:3/3 and LEFT/RIGHT/FORWARD.")

    direction = LineFilter()
    road = RoadModeFilter()
    while True:
        started = time.ticks_ms()
        img = sensor.snapshot()
        track = detect_line(img)
        now = time.ticks_ms()
        if time.ticks_diff(now, started) > MAX_FRAME_MS:
            track = TrackObservation()
        near_x, far_x = track.near_x, track.far_x
        action, error = choose_action(
            near_x, far_x,
            after_turn=controller.state == "TURN" and track.points[0] is not None,
        )
        discard = controller.discard_observation(started)
        observed = None if discard else action
        road_mode = road.update(None if discard else track, now)
        stable_action = direction.update(observed, now)
        if uart is not None:
            next_action = controller.next_action(stable_action, now, road_mode)
            if next_action is not None:
                packet = action_packet(next_action)
                if uart.write(packet) != len(packet):
                    raise OSError("UART short write; check robot before restarting")
                controller.mark_sent(next_action, time.ticks_ms())
                # 直行衔接保留视觉累计；新立正/转向指令清空累计，动作完成后重新观察。
                if next_action != 1:
                    direction = LineFilter()
                    road = RoadModeFilter()
                print(
                    "TX %s gap=%dms %s: %s"
                    % (ACTION_NAMES[next_action], controller.last_interval_ms,
                       road_mode, " ".join("%02X" % b for b in packet))
                )

        mode = "PREVIEW" if uart is None else controller.state
        plan = "NO LINE" if action is None else ACTION_NAMES[action]
        draw_tracking(img, track)
        img.draw_string((0, 30), "%s dt:%dms" % (
            "WAIT" if discard else road_mode, controller.last_interval_ms), color=255)
        img.draw_string((0, 0), "%s %s" % (mode, plan), color=255)
        if error is None:
            img.draw_string((0, 10), "Need 2 valid ROIs", color=255)
        else:
            img.draw_string(
                (0, 10), "N:%d F:%d E:%d" % (int(near_x), int(far_x), int(error)), color=255
            )


if __name__ == "__main__":
    run()
