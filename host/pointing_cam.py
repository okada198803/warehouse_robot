"""Mac のカメラで指差しを認識し、指した荷物置き場を llm_commander.py に渡す。

    python pointing_cam.py                 # 認識。同じ方向を 1 秒保持すると確定する
    python pointing_cam.py --calibrate     # 起動直後から校正モードで表示(キー操作は常に有効)
    python pointing_cam.py --skeleton-only # カメラ映像を出さず、手の骨格だけを黒背景に描く

操作(ウィンドウを選択してキー入力)
    n / c / s   北・中央・南の荷物を指して押す。2 秒間の指差しの中央値をその方向の校正値にする
    w           校正値を pointing_calib.json に保存
    r           初期値に戻す
    q           終了

確定した荷物 id は /tmp/pai_pointing.json に書く(llm_commander.py --cargo が「あれ」の解決に使う)。
表示: 既定は、カメラ映像(鏡像)に手の骨格と判定を重ねる(gesture_teleop.py の非常停止と同じ作り)。映像は保存しない。
プライバシー: カメラは手元に向け、顔が映らない位置に置く。顔や部屋を出したくないときは --skeleton-only。
AI: MediaPipe Gesture Recognizer の手のランドマーク(ローカル推論、無料)
"""
import argparse
import time

import cv2
import mediapipe as mp
import numpy as np

from gesture_teleop import create_recognizer
from pointing import (CALIB_FILE, DEFAULT_CALIB, Calibration, PointingSelector, clear_selection,
                      is_pointing, pointing_feature, write_selection)

W, H = 960, 540
LABELS = {'cargo_north': 'NORTH', 'cargo_center': 'CENTER', 'cargo_south': 'SOUTH'}
KEYS = {ord('n'): 'cargo_north', ord('c'): 'cargo_center', ord('s'): 'cargo_south'}
SAMPLE_SEC = 2.0
CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
               (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18),
               (18, 19), (19, 20), (0, 17)]
U_SCALE = 0.28   # u = 1 を画面幅の何割に対応させるか(定規の表示用)


def u_to_x(u):
    return int(W / 2 + u * W * U_SCALE)


def draw(canvas, lm, selector, calib, now, u, sampling, current):
    # 手の骨格(鏡像)。canvas はカメラ映像(既定)か黒背景(--skeleton-only)
    if lm is not None:
        pts = [(int(p.x * W), int(p.y * H)) for p in lm]
        for a, b in CONNECTIONS:
            cv2.line(canvas, pts[a], pts[b], (120, 120, 120), 2)
        for p in pts:
            cv2.circle(canvas, p, 4, (255, 200, 0), -1)
    # 定規: 校正済みの 3 方向と現在の u
    y = H - 90
    cv2.line(canvas, (0, y), (W, y), (80, 80, 80), 2)
    for cid, c in calib.centers.items():
        x = u_to_x(c)
        hot = selector.candidate == cid
        color = (0, 255, 0) if hot else (200, 200, 200)
        cv2.line(canvas, (x, y - 25), (x, y + 25), color, 4 if hot else 2)
        cv2.putText(canvas, LABELS.get(cid, cid), (x - 40, y + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    if u is not None:
        cv2.circle(canvas, (min(max(u_to_x(u), 0), W - 1), y), 9, (0, 200, 255), -1)
    # 状態
    pointing = u is not None
    cv2.putText(canvas, f'pointing: {"yes" if pointing else "no"}  state: {selector.status}',
                (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    prog = selector.progress(now)
    cv2.rectangle(canvas, (20, 55), (20 + int(300 * prog), 70), (0, 255, 0), -1)
    cv2.rectangle(canvas, (20, 55), (320, 70), (150, 150, 150), 1)
    if current:
        cv2.putText(canvas, f'SELECTED: {LABELS.get(current, current)}', (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
    if sampling:
        left = max(sampling['until'] - now, 0.0)
        cv2.putText(canvas, f'CALIBRATING {LABELS[sampling["id"]]}: point now ({left:.1f}s, {len(sampling["vals"])} samples)',
                    (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
    cv2.putText(canvas, 'n/c/s: calibrate  w: save  r: reset  q: quit', (20, H - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--skeleton-only', action='store_true', help='カメラ映像を出さず、手の骨格だけを黒背景に描く')
    ap.add_argument('--calib', default=CALIB_FILE)
    ap.add_argument('--dwell', type=float, default=1.0, help='同じ方向をこの秒数保持したら確定')
    ap.add_argument('--ttl', type=float, default=10.0, help='確定した荷物が有効な秒数')
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit('camera not available (システム設定 > プライバシー > カメラ を確認)')
    calib = Calibration.load(args.calib)
    selector = PointingSelector(calib, dwell=args.dwell, ttl=args.ttl)
    recognizer = create_recognizer()
    sampling = None
    t0 = time.monotonic()
    clear_selection()
    print('校正値:', {k: round(v, 2) for k, v in calib.centers.items()})
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)   # 鏡像(操作者の左がカメラ映像の左)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            now = time.monotonic() - t0
            result = recognizer.recognize_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), int(now * 1000))
            lm = result.hand_landmarks[0] if result.hand_landmarks else None
            u = pointing_feature(lm) if lm is not None and is_pointing(lm) else None

            if sampling and u is not None:
                sampling['vals'].append(u)
            if sampling and now >= sampling['until']:
                if len(sampling['vals']) >= 10:
                    vals = sorted(sampling['vals'])
                    calib.centers[sampling['id']] = vals[len(vals) // 2]
                    print(f'校正: {sampling["id"]} = {calib.centers[sampling["id"]]:.2f}')
                else:
                    print('校正失敗: 指差しのサンプルが足りません(人差し指だけを伸ばして保持)')
                sampling = None

            if sampling is None:
                confirmed = selector.update(u, now)
                if confirmed:
                    write_selection(confirmed)
                    print(f'指差し確定: {confirmed}')
            if args.skeleton_only:
                canvas = np.zeros((H, W, 3), dtype=np.uint8)
            else:
                canvas = cv2.resize(frame, (W, H))   # 骨格の座標(正規化 × W, H)と合わせる
            draw(canvas, lm, selector, calib, now, u, sampling, selector.current(now))
            cv2.imshow('pointing (q: quit)', canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key in KEYS and sampling is None:
                sampling = {'id': KEYS[key], 'until': now + SAMPLE_SEC, 'vals': []}
            elif key == ord('w'):
                calib.save(args.calib)
                print(f'保存: {args.calib}')
            elif key == ord('r'):
                calib.centers = dict(DEFAULT_CALIB)
    finally:
        clear_selection()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
