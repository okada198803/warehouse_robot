import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from control_logic import MAX_ANGULAR, MAX_LINEAR, Smoother, gesture_to_command, steer_from_x  # noqa


def test_no_hand_stops():
    c = gesture_to_command(None, 0.5, 0.0)
    assert (c.linear, c.angular) == (0.0, 0.0)


def test_low_score_stops():
    assert gesture_to_command('Open_Palm', 0.5, 0.3).linear == 0.0


def test_open_palm_forward_center():
    c = gesture_to_command('Open_Palm', 0.5, 0.9)
    assert c.linear == MAX_LINEAR and c.angular == 0.0


def test_steer_direction():
    assert steer_from_x(0.0) == MAX_ANGULAR       # 手が左端 → 左旋回(+)
    assert steer_from_x(1.0) == -MAX_ANGULAR
    assert steer_from_x(0.55) == 0.0              # 不感帯


def test_fist_is_emergency():
    assert gesture_to_command('Closed_Fist', 0.5, 0.9).emergency_stop


def test_smoother_stops_immediately():
    s = Smoother()
    for _ in range(10):
        s.update(gesture_to_command('Open_Palm', 0.5, 0.9))
    assert s.update(gesture_to_command(None, 0.5, 0.0)) == (0.0, 0.0)
