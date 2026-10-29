"""LS-6A 原厂固件循迹 第五版（v5.1）：三框 + 直道沿用第四版 + 弯道"成组转向"。

v5.1 相对 v5 的改动（实测反馈：直线过度转向、弯道后段抽搐）：
A. 直道彻底回滚到第四版逻辑：RoadModeFilter 判为 STRAIGHT 时走 decide_straight()，
   即 forth.py 的 choose_action 逐行复刻（0.65 近 + 0.35 远、单一门槛 12px、一拍一判）。
   v5 把弯道的连发/迟滞/曲率前馈也套在直道上，是直线过度转向的直接原因。
B. 连发改为"悲观起步"：TURN_GAIN_INIT_PX 加大到 30 且前 TURN_BURST_WARMUP 组只发 1 拍，
   先用真实数据校准再放大步数。v5 的 18px 是猜的，一开局就 3 拍连发很容易转过头。
C. 发令间隔整体放松：前进->转向 120→220ms，组内两拍 60→90ms。抽搐最可能来自
   新指令撞进上一拍还没跑完的动作（无动作完成回读），这一档是给这一项留的余量。
D. 安静窗 220→300ms：转向后重心回摆比预想的久，回摆帧会把偏差放大成"再转一次"。
E. 新增 STAND_BEFORE_TURN 开关（默认 False）。置 True 就是"每组转向第一拍前先立正"，
   用来单独验证"取消立正是不是抽搐原因"，不要同时改别的参数。

v5 相对第四版（forth.py）的原始思路：
1. 转向从"一帧一决策"改成"一次决策连发 k 拍"，组内不看画面，组末安静后再核对效果。
2. k 用自适应增益估计"单拍转向能消掉多少像素偏差"，k = |偏差| / 增益。
3. 一秒立正降级为"只在持续丢线和转向无效时用"，正常换向只留很短的间隔。
4. 弯道转向方向只由【近框 + 中框】决定，远框只判"弯道是否还在这侧延续"。
5. 弯道死区带迟滞：TURN_ENTER_PX(14) > TURN_EXIT_PX(10)。
6. 三个保护：转过头不许立刻反向修、连续转向拍数上限、转向无效升级立正。

调参顺序：先 ENABLE_MOTION=False 看 HUD；直线不动任何弯道参数；
弯道先看 reason 是 C-TURN 还是 C-POST，再动 TURN_QUIET_MS / TURN_CHAIN_GAP_MS，
最后才动 TURN_GAIN_INIT_PX 与 TURN_BURST_MAX。
"""

import time

# ---------------- 运动与画面方向 ----------------
ENABLE_MOTION = True  # False 只预览不发动作；True 延时后先立正再循迹
BLACK_THRESHOLD = (0, 64)  # 黑色灰度上限；阴影重就调小到 50 左右
H_MIRROR = False
V_FLIP = False
REVERSE_TURNS = False  # True 交换 3/4

# ---------------- 识线几何 ----------------
LINE_CENTER_X = 80
ROIS = ((0, 95, 160, 20), (0, 65, 160, 20), (0, 35, 160, 20))  # 近、中、远
MIN_PIXELS = 20
MIN_BLOB_WIDTH = 2
MAX_BLOB_WIDTH = 65
MIN_BLOB_HEIGHT = 4
RECOVERY_STRIP_HEIGHT = 4
MAX_CENTER_JUMP = 60
MAX_FRAME_MS = 150

# ---------------- 动作时长与发令间隔 ----------------
ACTION_TIME_MS = {0: 1000, 1: 1300, 3: 750, 4: 750}
CYCLE_MARGIN_MS = 70  # 前进之间的安全余量
STRAIGHT_MARGIN_MS = 40  # 三框确认直道时，把间隔再压短这么多（第四版已验证）
FWD_TO_TURN_GAP_MS = 220  # 完整步态结束到转向之间的间隔（v5 是 120，实测偏紧）
TURN_CHAIN_GAP_MS = 90  # 同一组转向里，两拍之间的间隔（v5 是 60，实测偏紧）
TURN_QUIET_MS = 300  # 转向结束后不采信画面的时间（v5 是 220，回摆可能更久）
STAND_SETTLE_MS = 100
STAND_QUIET_MS = 120
DECISION_GRACE_MS = 200
VERIFY_TIMEOUT_MS = 1200  # 安静窗口后一直没拿到有效画面，放弃效果核对
START_DELAY_MS = 3000
STAND_BEFORE_TURN = False  # A/B 开关：True 则每组转向第一拍前先立正（复现第四版前的做法）

# ---------------- 方向稳定性 ----------------
STABLE_FRAMES = 2
STABLE_MS = 40
STRAIGHT_ENTER_LIMITS = (8, 8, 4)  # 近框偏差 / 近远横向差 / 中点到连线残差
STRAIGHT_EXIT_LIMITS = (12, 14, 8)
STRAIGHT_STABLE_FRAMES = 3
STRAIGHT_STABLE_MS = 100

# ---------------- 转向策略：直道（第四版原样） ----------------
STRAIGHT_WEIGHTS = (0.65, 0.0, 0.35)  # 近、中、远；第四版是"近 0.65 + 远 0.35"
TURN_THRESHOLD = 12  # 第四版门槛，直道不要动它
POST_TURN_CENTER_PX = 12  # 第四版：刚转完向、近框进这条带就先走一步

# ---------------- 转向策略：弯道（第五版新增） ----------------
CURVE_WEIGHTS = (0.70, 0.30, 0.00)  # 弯道不用远框算转向量，避免杠杆放大
TURN_ENTER_PX = 14  # 弯道开始转的门槛
TURN_ENTER_CURVE_PX = 11  # 弯道仍在同侧延续时，允许更小偏差继续加转
TURN_EXIT_PX = 10  # 刚转完向时，偏差落进这条带就认为对正，先走一步
OVERSHOOT_PX = 12  # 转过头：反向偏差超过它就不许立刻反向修，先走一步
CURVE_FEEDFORWARD_PX = 10  # 近远横向差超过它，认为弯道还在同侧延续

TURN_BURST_MAX = 3  # 一次决策最多连发几拍转向
TURN_BURST_WARMUP = 2  # 前这么多组只发 1 拍，先校准增益再放大步数
TURN_GAIN_INIT_PX = 30.0  # 单拍转向消偏量的初始猜测（宁大勿小，过冲比多判一拍难受）
TURN_GAIN_MIN_PX = 6.0
TURN_GAIN_MAX_PX = 45.0
TURN_GAIN_ALPHA = 0.35  # 自适应学习率
MAX_CONSECUTIVE_TURNS = 4  # 连续转向拍数上限（中间没有前进），超过强制走一步
TURN_NO_EFFECT_PX = 3.0  # 一整组转向的改善量小于它，算"没转过去"
TURN_NO_EFFECT_LIMIT = 2  # 连续这么多组无效就立正复位

ACTION_NAMES = {0: "STAND", 1: "FORWARD", 3: "LEFT", 4: "RIGHT"}


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def validate_config():
    if not 0 <= BLACK_THRESHOLD[0] < BLACK_THRESHOLD[1] <= 255:
        raise ValueError("Invalid BLACK_THRESHOLD")
    if not 1 <= LINE_CENTER_X <= 158:
        raise ValueError("Invalid LINE_CENTER_X")
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
    if any(
        not 0 <= a <= b <= 160
        for a, b in zip(STRAIGHT_ENTER_LIMITS, STRAIGHT_EXIT_LIMITS)
    ):
        raise ValueError("Invalid straight hysteresis")
    if len(STRAIGHT_WEIGHTS) != 3 or min(STRAIGHT_WEIGHTS) < 0:
        raise ValueError("Set three non-negative straight weights")
    if not 0.98 <= sum(STRAIGHT_WEIGHTS) <= 1.02:
        raise ValueError("Straight weights should sum to 1")
    if not 1 <= TURN_THRESHOLD <= 79 or not 1 <= POST_TURN_CENTER_PX <= 79:
        raise ValueError("Invalid straight thresholds")
    if len(CURVE_WEIGHTS) != 3 or min(CURVE_WEIGHTS) < 0:
        raise ValueError("Set three non-negative curve weights")
    if not 0.98 <= sum(CURVE_WEIGHTS) <= 1.02:
        raise ValueError("Curve weights should sum to 1")
    if not 1 <= TURN_EXIT_PX <= TURN_ENTER_PX <= 79:
        raise ValueError("Need 1<=EXIT<=ENTER<=79")
    if not TURN_EXIT_PX < TURN_ENTER_CURVE_PX <= TURN_ENTER_PX:
        # 曲率前馈门槛必须比对正带宽，否则刚转完永远走 POST-TURN 分支，前馈形同虚设。
        raise ValueError("Need EXIT<CURVE_ENTER<=ENTER")
    if not 0 <= OVERSHOOT_PX <= 79 or CURVE_FEEDFORWARD_PX < 0:
        raise ValueError("Invalid overshoot or curve limits")
    if TURN_BURST_MAX < 1 or MAX_CONSECUTIVE_TURNS < 1 or TURN_BURST_WARMUP < 0:
        raise ValueError("Invalid burst limits")
    if not TURN_GAIN_MIN_PX < TURN_GAIN_INIT_PX < TURN_GAIN_MAX_PX:
        raise ValueError("TURN_GAIN_INIT_PX must be inside its clamp range")
    if not 0.0 < TURN_GAIN_ALPHA <= 1.0 or TURN_NO_EFFECT_LIMIT < 1:
        raise ValueError("Invalid gain tuning")
    if (
        min(
            CYCLE_MARGIN_MS,
            FWD_TO_TURN_GAP_MS,
            TURN_CHAIN_GAP_MS,
            TURN_QUIET_MS,
            STAND_SETTLE_MS,
            STAND_QUIET_MS,
            DECISION_GRACE_MS,
            STABLE_MS,
            STRAIGHT_STABLE_MS,
            START_DELAY_MS,
        )
        < 0
        or STABLE_FRAMES < 1
        or STRAIGHT_STABLE_FRAMES < 1
    ):
        raise ValueError("Invalid timing settings")


def action_packet(action_id):
    if action_id not in ACTION_NAMES:
        raise ValueError("Use action 0, 1, 3 or 4")
    return bytes((0x55, 0x55, 0x05, 0x06, action_id, 0x01, 0x00))


def roi_y(index):
    return ROIS[index][1] + ROIS[index][3] / 2


def compatible_points(a, b):
    # 同一帧内方向约束：两点 y 差越小，容许的横向差越小。
    return a[1] != b[1] and abs(a[0] - b[0]) * (roi_y(0) - roi_y(2)) <= (
        MAX_CENTER_JUMP * abs(a[1] - b[1])
    )


def project_x(a, b, y):
    return a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1])


class TrackObservation:
    """一帧观测；缺框的位置由另外两框连线投影补出，补出不等于可信框。"""

    def __init__(
        self,
        points=(None, None, None),
        wide=(False, False, False),
        recovered=(False, False, False),
    ):
        self.points = tuple(points)
        self.wide = tuple(wide)
        self.recovered = tuple(recovered)
        self.near_x = None
        self.mid_x = None
        self.far_x = None
        self.near_real = self.points[0] is not None
        valid = [p for p in self.points if p is not None]
        self.count = len(valid)
        if len(valid) < 2:
            return
        if any(not (0 <= p[0] < 160 and 0 <= p[1] < 120) for p in valid):
            return
        for i, a in enumerate(valid):
            if any(not compatible_points(a, b) for b in valid[i + 1 :]):
                return
        first, last = valid[0], valid[-1]
        near = self.points[0][0] if self.points[0] else project_x(first, last, roi_y(0))
        mid = self.points[1][0] if self.points[1] else project_x(first, last, roi_y(1))
        far = self.points[2][0] if self.points[2] else project_x(first, last, roi_y(2))
        if 0 <= near < 160 and 0 <= far < 160 and abs(near - far) <= MAX_CENTER_JUMP:
            self.near_x, self.mid_x, self.far_x = near, mid, far


def find_candidates(img, roi, strip=False):
    minimum = min(MIN_PIXELS, MIN_BLOB_WIDTH * roi[3]) if strip else MIN_PIXELS
    blobs = img.find_blobs(
        [BLACK_THRESHOLD],
        roi=roi,
        pixels_threshold=minimum,
        area_threshold=minimum,
        merge=True,
    )
    height = roi[3] if strip else MIN_BLOB_HEIGHT
    return (
        [
            (b.cx, b.cy)
            for b in blobs
            if MIN_BLOB_WIDTH <= b.w <= MAX_BLOB_WIDTH and b.h >= height
        ],
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
                points[index] = (
                    sum(p[0] for p in edges) / len(edges),
                    sum(p[1] for p in edges) / len(edges),
                )
                recovered[index] = True
    return TrackObservation(points, wide, recovered)


def draw_tracking(img, track):
    # 检测和重检全部结束后才绘制，避免边框文字污染窄带。
    for roi, point, wide in zip(ROIS, track.points, track.wide):
        img.draw_rectangle(roi, color=200 if wide else 100)
        if point is not None:
            img.draw_cross((int(point[0]), int(point[1])), color=255)
    img.draw_line((LINE_CENTER_X, 30, LINE_CENTER_X, 119), color=180)
    flags = "".join(
        "R" if r else "X" if w else "+" if p else "-"
        for p, w, r in zip(track.points, track.wide, track.recovered)
    )
    img.draw_string(
        (0, 20), "roi:%d/3 %s" % (sum(p is not None for p in track.points), flags), color=255
    )


class TrackGeometry:
    """把三框坐标转成决策用的几个数；三框来自同一帧，彼此没有时间差。"""

    def __init__(self, track):
        self.valid = False
        self.near_real = False
        self.near_err = 0.0
        self.mid_err = 0.0
        self.far_err = 0.0
        self.straight_error = 0.0  # 直道判定偏差：第四版权重（近 0.65 + 远 0.35）
        self.curve_error = 0.0  # 弯道判定偏差：第五版权重（近 0.70 + 中 0.30）
        self.far_gap = 0.0  # 近远横向差，近似"曲率方向"
        self.curve_dir = 0  # -1 线往左弯，+1 往右弯，0 看不出弯
        if track is None or track.near_x is None:
            return
        self.near_real = track.near_real
        self.near_err = track.near_x - LINE_CENTER_X
        self.mid_err = track.mid_x - LINE_CENTER_X
        self.far_err = track.far_x - LINE_CENTER_X
        self.straight_error = (
            STRAIGHT_WEIGHTS[0] * self.near_err
            + STRAIGHT_WEIGHTS[1] * self.mid_err
            + STRAIGHT_WEIGHTS[2] * self.far_err
        )
        self.curve_error = (
            CURVE_WEIGHTS[0] * self.near_err
            + CURVE_WEIGHTS[1] * self.mid_err
            + CURVE_WEIGHTS[2] * self.far_err
        )
        self.far_gap = self.far_err - self.near_err
        if self.far_gap < -CURVE_FEEDFORWARD_PX:
            self.curve_dir = -1
        elif self.far_gap > CURVE_FEEDFORWARD_PX:
            self.curve_dir = 1
        self.valid = True


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
    """直道须持续确认；弯道、遮挡、丢线立即撤销快速衔接。"""

    def __init__(self):
        self.mode = "CURVE"
        self.count = 0
        self.first_seen = None
        self.last_seen = None

    def update(self, track, now):
        if (
            self.last_seen is not None
            and time.ticks_diff(now, self.last_seen) > MAX_FRAME_MS
        ):
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
            measures = (
                abs(near[0] - LINE_CENTER_X),
                abs(near[0] - far[0]),
                abs(middle[0] - project_x(near, far, middle[1])),
            )
            if self.mode == "STRAIGHT" and all(
                value <= limit for value, limit in zip(measures, STRAIGHT_EXIT_LIMITS)
            ):
                return self.mode
            if all(value <= limit for value, limit in zip(measures, STRAIGHT_ENTER_LIMITS)):
                if self.count == 0:
                    self.first_seen = now
                self.count += 1
                self.mode = "CURVE"
                if (
                    self.count >= STRAIGHT_STABLE_FRAMES
                    and time.ticks_diff(now, self.first_seen) >= STRAIGHT_STABLE_MS
                ):
                    self.mode = "STRAIGHT"
                return self.mode
            mode = "CURVE"
        self.mode, self.count, self.first_seen = mode, 0, None
        return self.mode


class TurnBurst:
    """一组转向计划：方向、总共几拍、还剩几拍、转之前的偏差。"""

    def __init__(self, action, count, error_before, learn=True):
        self.action = action
        self.direction = -1 if action == 3 else 1
        self.count = count
        self.remaining = count
        self.error_before = error_before
        self.learn = learn  # False：临时单拍（直道），不参与增益学习


class SteerPolicy:
    """把一帧几何量翻译成一个动作：直道走第四版逻辑，弯道走第五版成组逻辑。"""

    def __init__(self):
        self.gain = TURN_GAIN_INIT_PX
        self.last_dir = 0  # 上一次发给主板的是哪边转；0 表示最近一次是前进/立正
        self.consec_turns = 0  # 中间没有前进的连续转向拍数
        self.no_effect = 0
        self.verified = 0  # 已核对过多少组转向（用于 warmup）
        self.overshot = 0  # 转过头次数；调大 TURN_GAIN_INIT_PX 的依据

    def action_for(self, direction):
        action = 3 if direction < 0 else 4
        if REVERSE_TURNS:
            action = 7 - action
        return action

    def on_sent(self, action):
        if action == 1 or action == 0:
            self.last_dir = 0
            self.consec_turns = 0
            if action == 0:
                self.no_effect = 0
        else:
            self.last_dir = -1 if action == 3 else 1
            self.consec_turns += 1

    def decide(self, geo, straight):
        """返回 (action, reason)；action=None 表示这一帧不作判断。"""
        if geo is None or not geo.valid:
            return None, "LOST"
        if straight:
            return self.decide_straight(geo)
        return self.decide_curve(geo)

    def decide_straight(self, geo):
        """第四版 choose_action 的逐行复刻：直道行为必须与 forth.py 完全一致。

        这里坚决不掺弯道那套：不连发、不加迟滞、不看曲率前馈、不做转过头保护。
        第四版直道本来就是好的，任何"顺手优化"都会变成直线过度转向。
        """
        if self.last_dir != 0 and geo.near_real and abs(geo.near_err) <= POST_TURN_CENTER_PX:
            return 1, "S-POST"
        e = geo.straight_error  # 近 0.65 + 远 0.35
        if e < -TURN_THRESHOLD:
            return self.action_for(-1), "S-TURN"
        if e > TURN_THRESHOLD:
            return self.action_for(1), "S-TURN"
        return 1, "S-LINE"

    def decide_curve(self, geo):
        e = geo.curve_error  # 近 0.70 + 中 0.30，远框不参与转向量
        ae = abs(e)
        near_ae = abs(geo.near_err)

        # 保护 1：原地连续转太多拍，脚位已经歪了，先走一步重新建立姿态。
        if self.consec_turns >= MAX_CONSECUTIVE_TURNS:
            return 1, "C-LIMIT"
        # 保护 2：连转两组都没把偏差消下去，说明不是角度不够，是姿态/识别有问题。
        if self.no_effect >= TURN_NO_EFFECT_LIMIT:
            return 0, "C-NOEFF"
        # 刚转完向：近框已经落进对正带，就先走一步，别在原地继续磨。
        if self.last_dir != 0 and near_ae <= TURN_EXIT_PX:
            return 1, "C-POST"
        # 转过头：偏差已经翻到另一侧且不小，这时反向修必然来回摆，先走一步。
        if self.last_dir != 0 and e * self.last_dir < 0 and ae > OVERSHOOT_PX:
            return 1, "C-OVER"

        # 迟滞死区：弯道仍同侧延续时放宽，平时用较宽门槛压住换向抖动。
        enter = TURN_ENTER_PX
        if self.last_dir != 0 and geo.curve_dir == self.last_dir:
            enter = TURN_ENTER_CURVE_PX
        if ae <= enter:
            return 1, "C-LINE" if ae <= TURN_EXIT_PX else "C-DEAD"
        return self.action_for(-1 if e < 0 else 1), "C-TURN"

    def burst_for(self, geo, action):
        """算这一组连发几拍：偏差 / 单拍增益，再按可信度与上限收窄。"""
        if geo is None or not geo.valid:
            return 1
        count = int(abs(geo.curve_error) / self.gain + 0.5)
        if count < 1:
            count = 1
        if count > TURN_BURST_MAX:
            count = TURN_BURST_MAX
        # 前几组先只发一拍：增益还是猜的，先拿真实数据校准，宁可多判一次也别过冲。
        if self.verified < TURN_BURST_WARMUP:
            count = 1
        direction = -1 if action == 3 else 1
        # 曲率方向和偏差方向矛盾（多半是横线/噪点），只转一拍边看边改。
        if geo.curve_dir != 0 and geo.curve_dir != direction:
            count = 1
        # 近框是推算出来的，没有真看到脚前的线，也只转一拍。
        if not geo.near_real:
            count = 1
        room = MAX_CONSECUTIVE_TURNS - self.consec_turns
        if room < 1:
            room = 1
        if count > room:
            count = room
        return count

    def note_burst_result(self, burst, geo):
        """安静窗口后拿到的第一帧：核对这一组转到底有没有用，并更新增益。"""
        self.verified += 1
        progress = (burst.error_before - geo.curve_error) * burst.direction
        per_packet = progress / burst.count
        if per_packet > TURN_GAIN_MIN_PX * 0.5:
            self.gain = clamp(
                self.gain * (1.0 - TURN_GAIN_ALPHA) + per_packet * TURN_GAIN_ALPHA,
                TURN_GAIN_MIN_PX,
                TURN_GAIN_MAX_PX,
            )
        if geo.curve_error * burst.direction < -TURN_EXIT_PX:
            self.overshot += 1  # 转过了：说明单拍消偏量被低估，增益该调大
        if (
            progress < TURN_NO_EFFECT_PX * burst.count
            and abs(geo.curve_error) > TURN_EXIT_PX
        ):
            self.no_effect += 1
        else:
            self.no_effect = 0


class GaitController:
    def __init__(self, steer):
        self.steer = steer
        self.state = "IDLE"
        self.sent_at = None
        self.duration = 0
        self.quiet_ms = 0  # 发令后这段时间内采集的画面不采信
        self.burst = None  # 正在执行的转向组
        self.pending_verify = None  # 待核对效果的已结束转向组
        self.last_interval_ms = 0

    def elapsed(self, now):
        return 0 if self.sent_at is None else time.ticks_diff(now, self.sent_at)

    def next_action(self, stable_action, geo, now, road_mode):
        if self.state != "IDLE":
            ready_after = self.duration
            if self.state == "WALK":
                if stable_action == 1 and road_mode == "STRAIGHT":
                    # 三框确认直道：把衔接间隔压短，走得更连贯。
                    ready_after = ACTION_TIME_MS[1] + STRAIGHT_MARGIN_MS
                elif stable_action in (3, 4):
                    # 前进后要转向：比常规间隔再多等一点，让上一拍步态收干净。
                    ready_after = ACTION_TIME_MS[1] + FWD_TO_TURN_GAP_MS
            if self.elapsed(now) < ready_after:
                return None

        # 1) 组内后续转向拍：不再看画面，按计划连发。
        if self.burst is not None and self.burst.remaining > 0:
            return self.burst.action

        # 2) 上一组转向的安静窗口已过，用稳定后的第一帧核对效果。
        if self.pending_verify is not None:
            if geo is not None and geo.valid:
                self.steer.note_burst_result(self.pending_verify, geo)
                self.pending_verify = None
            elif self.elapsed(now) > self.duration + VERIFY_TIMEOUT_MS:
                self.pending_verify = None

        if stable_action is None:
            if self.state in ("WALK", "TURN") and (
                self.elapsed(now) >= self.duration + DECISION_GRACE_MS
            ):
                return 0  # 持续看不到线：立正一次，复位姿态后重新观察
            return None
        if stable_action == 0:
            return 0
        if stable_action in (3, 4):
            # A/B 开关：想验证"取消立正是不是抽搐的原因"时把它打开，
            # 这里会在每组第一拍转向之前先发一次立正，其余参数都不用动。
            if STAND_BEFORE_TURN and self.state != "STAND":
                return 0
            # 只有弯道才连发；直道的第四版逻辑是一拍一判，不允许成组。
            if self.burst is None and road_mode != "STRAIGHT":
                count = self.steer.burst_for(geo, stable_action)
                error_before = (
                    geo.curve_error if geo is not None and geo.valid else 0.0
                )
                self.burst = TurnBurst(stable_action, count, error_before)
        return stable_action

    def mark_sent(self, action, now):
        self.last_interval_ms = self.elapsed(now)
        self.sent_at = now
        self.steer.on_sent(action)
        if action == 1:
            self.state = "WALK"
            self.duration = ACTION_TIME_MS[1] + CYCLE_MARGIN_MS
            self.quiet_ms = 0
        elif action == 0:
            self.state = "STAND"
            self.duration = ACTION_TIME_MS[0] + STAND_SETTLE_MS
            self.quiet_ms = ACTION_TIME_MS[0] + STAND_QUIET_MS
            self.burst = None
            self.pending_verify = None
        else:
            self.state = "TURN"
            if self.burst is None:
                # 直道单拍（第四版逻辑）：不学习增益，也不做效果核对。
                self.burst = TurnBurst(action, 1, 0.0, False)
            self.burst.remaining -= 1
            if self.burst.remaining > 0:
                # 组内：只留发令间隔，不插立正，也不采信回摆画面。
                self.duration = ACTION_TIME_MS[action] + TURN_CHAIN_GAP_MS
                self.quiet_ms = ACTION_TIME_MS[action] + TURN_CHAIN_GAP_MS
            else:
                self.duration = ACTION_TIME_MS[action] + TURN_QUIET_MS
                self.quiet_ms = self.duration
                self.pending_verify = self.burst if self.burst.learn else None
                self.burst = None

    def discard_observation(self, frame_started):
        # 只丢弃"机械还在回摆/动作还没跑完"时开始采集的画面。
        return self.elapsed(frame_started) < self.quiet_ms


def run():
    validate_config()
    import sensor

    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_hmirror(H_MIRROR)
    sensor.set_vflip(V_FLIP)
    sensor.skip_frames(time=1500)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)

    steer = SteerPolicy()
    controller = GaitController(steer)
    uart = None
    if ENABLE_MOTION:
        from pyb import UART

        uart = UART(3, 9600, bits=8, parity=None, stop=1, timeout=100, timeout_char=10)
        print("MOTION: stand first, then follow line. Prepare for 3 seconds.")
        time.sleep_ms(START_DELAY_MS)
        packet = action_packet(0)
        if uart.write(packet) != len(packet):
            raise OSError("UART short write; check robot before restarting")
        controller.mark_sent(0, time.ticks_ms())
    else:
        print("PREVIEW only. Check roi:3/3 and the reason string.")

    direction = LineFilter()
    road = RoadModeFilter()
    reason = "INIT"
    straight = False
    while True:
        started = time.ticks_ms()
        img = sensor.snapshot()
        track = detect_line(img)
        now = time.ticks_ms()
        if time.ticks_diff(now, started) > MAX_FRAME_MS:
            track = TrackObservation()
        geo = TrackGeometry(track)
        discard = controller.discard_observation(started)
        # 先用这一帧的画面判直道/弯道，再按模式选决策逻辑：
        # STRAIGHT 走第四版原逻辑，其余（CURVE/CROSS/LOST）走第五版弯道逻辑。
        road_mode = road.update(None if discard else track, now)
        straight = road_mode == "STRAIGHT"
        decision, new_reason = steer.decide(geo, straight)
        if not discard:
            reason = new_reason
        observed = None if discard else decision
        stable_action = direction.update(observed, now)
        if uart is not None:
            next_action = controller.next_action(stable_action, geo, now, road_mode)
            if next_action is not None:
                packet = action_packet(next_action)
                if uart.write(packet) != len(packet):
                    raise OSError("UART short write; check robot before restarting")
                group_size = controller.burst.count if controller.burst else 1
                controller.mark_sent(next_action, time.ticks_ms())
                if next_action != 1:
                    direction = LineFilter()
                    road = RoadModeFilter()
                print(
                    "TX %s x%d gap=%dms %s %s gain=%.1f c=%d over=%d"
                    % (
                        ACTION_NAMES[next_action],
                        group_size,
                        controller.last_interval_ms,
                        road_mode,
                        reason,
                        steer.gain,
                        steer.consec_turns,
                        steer.overshot,
                    )
                )

        mode = "PREVIEW" if uart is None else controller.state
        draw_tracking(img, track)
        img.draw_string((0, 0), "%s %s" % (mode, reason), color=255)
        if geo.valid:
            img.draw_string(
                (0, 10),
                "N:%d M:%d F:%d" % (int(geo.near_err), int(geo.mid_err), int(geo.far_err)),
                color=255,
            )
        else:
            img.draw_string((0, 10), "Need 2 valid ROIs", color=255)
        img.draw_string(
            (0, 30),
            "%s%s g%.0f c%d" % (
                road_mode[:1], "4" if straight else "5", steer.gain, steer.consec_turns),
            color=255,
        )


if __name__ == "__main__":
    run()
