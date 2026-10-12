"""Mac のカメラで手のジェスチャーを認識し、Gazebo のロボットを操縦する(フェーズ B)。

    python gesture_teleop.py              # 手動操縦モード: /cmd_vel を送信
    python gesture_teleop.py --nav-mode   # 自律移動中の監視モード: 握りこぶしで非常停止のみ
    python gesture_teleop.py --dry-run    # ROS に接続せず認識結果だけ表示

AI: MediaPipe Gesture Recognizer(ローカル推論、無料)
"""
import argparse
import os
import time
import urllib.request

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

from control_logic import Smoother, gesture_to_command

MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/gesture_recognizer/'
             'gesture_recognizer/float16/latest/gesture_recognizer.task')
MODEL_PATH = os.path.join(os.path.dirname(__file__), 'models', 'gesture_recognizer.task')
SEND_HZ = 10


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        os.makedirs(os.path.dirname(MODEL_PATH), exist_ok=True)
        print('downloading gesture model ...')
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    return MODEL_PATH


def create_recognizer():
    options = vision.GestureRecognizerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=ensure_model()),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1)
    return vision.GestureRecognizer.create_from_options(options)


def draw_overlay(frame, cmd, v, w, gesture, score, mode):
    h, wd = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (wd, 70), (0, 0, 0), -1)
    # cv2.putText は日本語を描けないため英字で表示
    color = (0, 0, 255) if cmd.emergency_stop else (0, 255, 0)
    cv2.putText(frame, f'[{mode}] gesture: {gesture or "-"} ({score:.2f})',
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f'cmd: v={v:+.2f} m/s  w={w:+.2f} rad/s',
                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    # 操舵の不感帯を表示
    for x in (0.42, 0.58):
        cv2.line(frame, (int(wd * x), 70), (int(wd * x), h), (200, 200, 200), 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--nav-mode', action='store_true',
                    help='/cmd_vel は送らず、握りこぶしで自律移動を停止するだけ')
    ap.add_argument('--dry-run', action='store_true', help='ROS に接続しない')
    args = ap.parse_args()

    link = None
    if not args.dry_run:
        from ros_link import RosLink
        link = RosLink(args.host, args.port)
        print(f'connected to rosbridge {args.host}:{args.port}')

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit('camera not available (システム設定 > プライバシー > カメラ を確認)')

    recognizer = create_recognizer()
    smoother = Smoother()
    mode = 'NAV-WATCH' if args.nav_mode else 'TELEOP'
    last_send = 0.0
    stop_latched = False   # 非常停止を 1 回だけ送るためのフラグ
    t0 = time.monotonic()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)   # 鏡像にして直感的な操作にする
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts = int((time.monotonic() - t0) * 1000)
            result = recognizer.recognize_for_video(image, ts)

            gesture, score, hand_x = None, 0.0, 0.5
            if result.gestures:
                top = result.gestures[0][0]
                gesture, score = top.category_name, top.score
                lm = result.hand_landmarks[0]
                hand_x = sum(p.x for p in lm) / len(lm)
                for p in lm:
                    cv2.circle(frame, (int(p.x * frame.shape[1]), int(p.y * frame.shape[0])),
                               3, (255, 200, 0), -1)

            cmd = gesture_to_command(gesture, hand_x, score)
            v, w = smoother.update(cmd)
            if args.nav_mode:
                v, w = 0.0, 0.0

            now = time.monotonic()
            if link and now - last_send >= 1.0 / SEND_HZ:
                last_send = now
                if cmd.emergency_stop and not stop_latched:
                    link.send_plan({'action': 'stop'})
                    print('EMERGENCY STOP sent')
                stop_latched = cmd.emergency_stop
                if not args.nav_mode:
                    link.send_twist(v, w)

            draw_overlay(frame, cmd, v, w, gesture, score, mode)
            cv2.imshow('gesture teleop (q: quit)', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        if link:
            if not args.nav_mode:
                link.send_twist(0.0, 0.0)
            time.sleep(0.2)
            link.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
