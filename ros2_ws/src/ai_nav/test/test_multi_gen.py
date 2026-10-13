import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ai_nav.multi_gen import (ROBOTS, bridge_yaml, namespaced_sdf,  # noqa: E402
                              relative_nav2_params, tf_remaps)

PKG = os.path.join(os.path.dirname(__file__), '..')
MODEL = os.path.join(PKG, '..', 'nav_dev', 'models', 'LidarRobo4', 'model.sdf')
PARAMS = os.path.join(PKG, 'params', 'nav2_params.yaml')


def test_sdf_topics_namespaced():
    out = namespaced_sdf(open(MODEL, encoding='utf-8').read(), 'r1')
    for t in ('/r1/scan', '/r1/odom', '/r1/odom/tf', '/r1/cmd_vel', '/r1/joint_states'):
        assert f'>{t}<' in out
    assert '<topic>scan</topic>' not in out


def test_sdf_unexpected_raises():
    try:
        namespaced_sdf('<sdf/>', 'r1')
    except ValueError:
        return
    raise AssertionError('ValueError expected')


def test_bridge_yaml_parses_and_has_each_namespace():
    data = yaml.safe_load(bridge_yaml(list(ROBOTS)))
    names = [d['ros_topic_name'] for d in data]
    assert names.count('/clock') == 1
    for ns in ROBOTS:
        assert f'/{ns}/scan' in names and f'/{ns}/cmd_vel' in names
    assert len(names) == len(set(names))
    assert all(d['gz_topic_name'] == d['ros_topic_name'] for d in data)


def test_tf_remaps():
    assert tf_remaps(['r1']) == [('/r1/odom/tf', '/r1/tf')]


def test_relative_params_only_topics_change():
    src = open(PARAMS, encoding='utf-8').read()
    out = relative_nav2_params(src)
    a, b = src.splitlines(), out.splitlines()
    assert len(a) == len(b)
    diff = [(x, y) for x, y in zip(a, b) if x != y]
    assert len(diff) == 3
    assert all(('/scan' in x or '/odom' in x) for x, _ in diff)
    yaml.safe_load(out)


def test_robot_poses_distinct():
    xy = [(x, y) for x, y, _ in ROBOTS.values()]
    assert len(set(xy)) == len(xy)
