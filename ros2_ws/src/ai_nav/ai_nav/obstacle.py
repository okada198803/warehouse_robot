"""デモ用: Gazebo の通路にパレット(障害物)を置く / 撤去する。

    ros2 run ai_nav obstacle place south_aisle
    ros2 run ai_nav obstacle remove south_aisle
    ros2 run ai_nav obstacle clear            # 全通路から撤去

地図(Nav2)には反映しない。ロボットは LiDAR で見つけて初めて障害物を知る。
"""
import argparse
import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from ai_nav.plan_logic import load_places

WORLD = 'warehouse'
SIZE = (0.5, 1.1, 0.45)   # x, y, z [m]。幅 0.9 m の通路を完全に塞ぐ


def pallet_sdf(name):
    sx, sy, sz = SIZE
    color = '0.15 0.35 0.75 1'
    return (f'<sdf version="1.6"><model name="{name}"><static>true</static><link name="link">'
            f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f'</collision><visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box>'
            f'</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
            f'</material></visual></link></model></sdf>')


def place(aisle, p):
    name = f'pallet_{aisle}'
    subprocess.run(['ros2', 'run', 'ros_ign_gazebo', 'create', '-world', WORLD,
                    '-name', name, '-file', write_sdf(name),
                    '-x', str(p['x']), '-y', str(p['y']), '-z', str(SIZE[2] / 2)], check=True)
    print(f'placed {name} at ({p["x"]}, {p["y"]})')


def remove(aisle):
    name = f'pallet_{aisle}'
    subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/remove',
                    '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '3000', '--req', f'name: "{name}" type: MODEL'], check=False)
    print(f'removed {name}')


def main():
    default = os.path.join(get_package_share_directory('ai_nav'),
                           'config', 'places_warehouse.yaml')
    ap = argparse.ArgumentParser()
    ap.add_argument('command', choices=['place', 'remove', 'clear'])
    ap.add_argument('aisle', nargs='?')
    ap.add_argument('--places', default=default)
    args = ap.parse_args()

    places = load_places(args.places)
    aisles = [k for k, v in places.items() if v.get('kind') == 'aisle']
    if args.command == 'clear':
        for a in aisles:
            remove(a)
        return
    if args.aisle not in aisles:
        raise SystemExit(f'aisle must be one of {aisles}')
    if args.command == 'place':
        place(args.aisle, places[args.aisle])
    else:
        remove(args.aisle)


if __name__ == '__main__':
    main()


def write_sdf(name):
    path = f'/tmp/{name}.sdf'
    with open(path, 'w') as f:
        f.write(pallet_sdf(name))
    return path
