"""LS-6A 原厂固件循迹：1号完整前进步态的衔接版。

主板中需已下载 0=立正、1=前进、3=左转、4=右转。
在 OpenMV IDE 打开整个文件运行；先预览，确认识线后改 ENABLE_MOTION=True。
直行时每次播放1号一次，周期之间不插入0号立正。
准备转向或持续丢线时，等当前周期结束，先立正，再重新判断方向。
使用已验证的 UART3 / 9600 / 8N1 接线；不修改主板固件。
本版按原始动作时间计时，没有动作完成回读；周期衔接会有短小间隔，
并非主板内无限循环。动作文件时长修改后应同步修改 ACTION_TIME_MS。
"""

import time

ENABLE_MOTION = False  # 在这份完整程序里改为True才会运动
BLACK_THRESHOLD = (0, 64)
H_MIRROR = False
V_FLIP = False
REVERSE_TURNS = False
LINE_CENTER_X = 80
TURN_THRESHOLD = 12
ROIS = ((0, 95, 160, 20), (0, 35, 160, 20))
MIN_PIXELS = 20
MIN_BLOB_WIDTH = 2
MAX_BLOB_WIDTH = 65
MIN_BLOB_HEIGHT = 4
MAX_CENTER_JUMP = 60
MAX_FRAME_MS = 150

# 资料原始XML的各帧时间合计；不是7号前进一步的时长。
ACTION_TIME_MS = {0: 1000, 1: 1300, 3: 750, 4: 750}
CYCLE_MARGIN_MS = 80  # 防止新动作在上一周期完成前到达；不是立正停顿
TURN_SETTLE_MS = 180
STAND_SETTLE_MS = 150
DECISION_GRACE_MS = 200  # 周期结束后，给短暂丢线/方向变化一点确认时间
STABLE_FRAMES = 3
STABLE_MS = 100
START_DELAY_MS = 3000
ACTION_NAMES = {0: "STAND", 1: "FORWARD", 3: "LEFT", 4: "RIGHT"}


def validate_config():
    if not 0 <= BLACK_THRESHOLD[0] < BLACK_THRESHOLD[1] <= 255:
        raise ValueError("Invalid BLACK_THRESHOLD")
    if not 1 <= LINE_CENTER_X <= 158 or not 1 <= TURN_THRESHOLD <= 79:
        raise ValueError("Invalid center or turn threshold")
    if len(ROIS) != 2 or ROIS[0][1] <= ROIS[1][1]:
        raise ValueError("Use near ROI first, far ROI second")
    for x, y, w, h in ROIS:
        if min(x, y) < 0 or min(w, h) <= 0 or x+w > 160 or y+h > 120:
            raise ValueError("ROI must fit 160x120")
    if not 1 <= MIN_BLOB_WIDTH <= MAX_BLOB_WIDTH <= 160:
        raise ValueError("Invalid blob widths")
    if MIN_PIXELS < 1 or not 1 <= MIN_BLOB_HEIGHT <= 120:
        raise ValueError("Invalid blob filters")
    if not 1 <= MAX_CENTER_JUMP <= 160 or MAX_FRAME_MS <= 0:
        raise ValueError("Invalid frame filters")
    if any(ACTION_TIME_MS.get(a, 0) <= 0 for a in ACTION_NAMES):
        raise ValueError("Set positive action durations")
    if min(CYCLE_MARGIN_MS, TURN_SETTLE_MS, STAND_SETTLE_MS,
           DECISION_GRACE_MS, STABLE_MS) < 0 or STABLE_FRAMES < 1:
        raise ValueError("Invalid timing settings")


def action_packet(action_id):
    if action_id not in ACTION_NAMES:
        raise ValueError("Use action 0, 1, 3 or 4")
    return bytes((0x55, 0x55, 0x05, 0x06, action_id, 0x01, 0x00))


def detect_line(img):
    selected = []
    reference = LINE_CENTER_X
    for roi in ROIS:
        blobs = img.find_blobs(
            [BLACK_THRESHOLD],
            roi=roi,
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_PIXELS,
            merge=True
        )
        candidates = [b for b in blobs
                      if MIN_BLOB_WIDTH <= b.w <= MAX_BLOB_WIDTH
                      and b.h >= MIN_BLOB_HEIGHT]
        blob = min(candidates, key=lambda b: abs(b.cx - reference)) if candidates else None
        if blob is not None and selected and selected[0] is not None:
            if abs(blob.cx - reference) > MAX_CENTER_JUMP:
                blob = None
        selected.append(blob)
        if blob is not None:
            reference = blob.cx

    for roi in ROIS:
        img.draw_rectangle(roi, color=100)
    for blob in selected:
        if blob is not None:
            img.draw_rectangle(blob.rect, color=255)
            img.draw_cross((blob.cx, blob.cy), color=255)
    img.draw_line((LINE_CENTER_X, 30, LINE_CENTER_X, 119), color=180)
    img.draw_string((0, 20), "roi:%d/2" % sum(b is not None for b in selected), color=255)
    if any(blob is None for blob in selected):
        return None, None
    return selected[0].cx, selected[1].cx


def choose_action(near_x, far_x):
    if near_x is None or far_x is None:
        return None, None
    # 近处偏差为主，远处用于提前判断弯道；单位均为像素。
    error = 0.65 * (near_x - LINE_CENTER_X) + 0.35 * (far_x - LINE_CENTER_X)
    if error < -TURN_THRESHOLD:
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
        interrupted = self.last_seen is not None and time.ticks_diff(now, self.last_seen) > MAX_FRAME_MS
        self.last_seen = now
        if action is None:
            self.candidate, self.count, self.first_seen = None, 0, None
            return None
        if interrupted or action != self.candidate:
            self.candidate, self.count, self.first_seen = action, 1, now
        else:
            self.count += 1
        if self.count >= STABLE_FRAMES and time.ticks_diff(now, self.first_seen) >= STABLE_MS:
            return action
        return None


class GaitController:
    def __init__(self):
        self.state = "IDLE"
        self.sent_at = None
        self.duration = 0

    def elapsed(self, now):
        return 0 if self.sent_at is None else time.ticks_diff(now, self.sent_at)

    def next_action(self, stable_action, now):
        if self.state != "IDLE" and self.elapsed(now) < self.duration:
            return None
        if self.state == "WALK":
            if stable_action == 1:
                return 1  # 继续完整步态，不调用立正
            if stable_action is None and self.elapsed(now) < self.duration + DECISION_GRACE_MS:
                return None
            return 0  # 换方向或持续丢线，仅在周期结束后立正一次
        # 立正和转向执行期间不使用视觉结果；结束后由新帧重新累计。
        if stable_action in (1, 3, 4):
            return stable_action
        return None

    def mark_sent(self, action, now):
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
        # 直行时持续识线，为下个周期作准备；立正/转向期间的画面不参与决策。
        return self.state in ("STAND", "TURN") and self.elapsed(frame_started) < self.duration


def run():
    validate_config()
    import sensor
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_hmirror(H_MIRROR)
    sensor.set_vflip(V_FLIP)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)

    uart = None
    controller = GaitController()
    if ENABLE_MOTION:
        from pyb import UART
        uart = UART(3, 9600, bits=8, parity=None, stop=1,
                    timeout=100, timeout_char=10)
        print("MOTION: stand first, then follow line. Prepare for 3 seconds.")
        time.sleep_ms(START_DELAY_MS)
        packet = action_packet(0)
        if uart.write(packet) != len(packet):
            raise OSError("UART short write; check robot before restarting")
        controller.mark_sent(0, time.ticks_ms())
    else:
        print("PREVIEW only. Check roi:2/2 and LEFT/RIGHT/FORWARD.")

    direction = LineFilter()
    while True:
        started = time.ticks_ms()
        img = sensor.snapshot()
        near_x, far_x = detect_line(img)
        now = time.ticks_ms()
        if time.ticks_diff(now, started) > MAX_FRAME_MS:
            near_x, far_x = None, None
        action, error = choose_action(near_x, far_x)
        observed = None if controller.discard_observation(started) else action
        stable_action = direction.update(observed, now)
        if uart is not None:
            next_action = controller.next_action(stable_action, now)
            if next_action is not None:
                packet = action_packet(next_action)
                if uart.write(packet) != len(packet):
                    raise OSError("UART short write; check robot before restarting")
                controller.mark_sent(next_action, time.ticks_ms())
                # 直行衔接保留视觉累计；站稳/转向后必须重新观察。
                if next_action != 1:
                    direction = LineFilter()
                print("TX %s: %s" % (ACTION_NAMES[next_action], " ".join("%02X" % b for b in packet)))

        mode = "PREVIEW" if uart is None else controller.state
        plan = "NO LINE" if action is None else ACTION_NAMES[action]
        img.draw_string((0, 0), "%s %s" % (mode, plan), color=255)
        if error is None:
            img.draw_string((0, 10), "Need both ROIs", color=255)
        else:
            img.draw_string((0, 10), "N:%d F:%d E:%d" % (near_x, far_x, int(error)), color=255)


if __name__ == "__main__":
    run()