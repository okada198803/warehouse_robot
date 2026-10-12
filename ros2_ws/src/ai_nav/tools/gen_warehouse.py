"""倉庫ワールドを 1 つの壁定義から生成する。

出力:
  worlds/warehouse.sdf   Gazebo 用ワールド(壁・棚を静的モデルとして配置)
  maps/warehouse.pgm/.yaml  Nav2 用の占有格子地図(同じ壁定義をラスタライズ)

同じ定義から両方を作るため、SLAM を行わなくても Gazebo と地図が必ず一致する。
    python3 tools/gen_warehouse.py
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
TEMPLATE = os.path.join(PKG, '..', 'nav_dev', 'models', 'worlds', 'nav_slam.sdf')

WORLD_NAME = 'warehouse'
SIZE_X, SIZE_Y = 7.0, 5.0       # 床面(m)
RES = 0.05                      # 地図解像度(m/px)
MARGIN = 0.25                   # 地図の外側余白(m)

# (名前, 中心x, 中心y, 幅x, 奥行y, 高さ, 色RGB)
WALL = (0.55, 0.55, 0.6)
RACK = (0.85, 0.55, 0.2)
GATE = (0.4, 0.4, 0.45)
BOXES = [
    ('wall_south', 3.5, -0.05, 7.2, 0.1, 0.5, WALL),
    ('wall_north', 3.5, 5.05, 7.2, 0.1, 0.5, WALL),
    ('wall_west', -0.05, 2.5, 0.1, 5.2, 0.5, WALL),
    ('wall_east', 7.05, 2.5, 0.1, 5.2, 0.5, WALL),
    # 中央の棚 2 列。間が中央通路(幅 0.8 m)
    ('rack_south', 3.5, 1.8, 3.0, 0.6, 1.0, RACK),
    ('rack_north', 3.5, 3.2, 3.0, 0.6, 1.0, RACK),
    # 南北の通路を幅 0.9 m に絞る仕切り(パレット 1 枚で塞がる幅)
    ('gate_south', 3.5, 0.3, 0.2, 0.6, 0.5, GATE),
    ('gate_north', 3.5, 4.7, 0.2, 0.6, 0.5, GATE),
]


def box_model(name, cx, cy, sx, sy, sz, rgb):
    color = f'{rgb[0]} {rgb[1]} {rgb[2]} 1'
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {sz / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def write_world():
    with open(TEMPLATE, encoding='utf-8') as f:
        sdf = f.read()
    sdf = sdf.replace('<world name="empty">', f'<world name="{WORLD_NAME}">')
    sdf = sdf.replace('<cast_shadows>true</cast_shadows>', '<cast_shadows>false</cast_shadows>')
    # GUI の初期視点を倉庫全体の俯瞰に
    sdf = sdf.replace('<camera_pose>-6 0 6 0 0.5 0</camera_pose>',
                      '<camera_pose>3.5 -3.5 7.5 0 0.95 1.57</camera_pose>')
    models = ''.join(box_model(*b) for b in BOXES)
    sdf = sdf.replace('  </world>', models + '\n  </world>')
    out = os.path.join(PKG, 'worlds', f'{WORLD_NAME}.sdf')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(sdf)
    return out


def write_map():
    ox = oy = -MARGIN
    w = int(round((SIZE_X + 2 * MARGIN) / RES))
    h = int(round((SIZE_Y + 2 * MARGIN) / RES))
    grid = [[254] * w for _ in range(h)]      # 254 = free
    for _, cx, cy, sx, sy, _sz, _c in BOXES:
        x0, x1 = cx - sx / 2, cx + sx / 2
        y0, y1 = cy - sy / 2, cy + sy / 2
        for r in range(h):
            y = oy + (h - 1 - r + 0.5) * RES      # 画素中心の y(画像は上が +y)
            if not y0 - RES / 2 <= y <= y1 + RES / 2:
                continue
            for c in range(w):
                x = ox + (c + 0.5) * RES
                if x0 - RES / 2 <= x <= x1 + RES / 2:
                    grid[r][c] = 0                # 0 = occupied
    # 外壁の外側は未知領域
    for r in range(h):
        y = oy + (h - 1 - r + 0.5) * RES
        for c in range(w):
            x = ox + (c + 0.5) * RES
            if (x < -0.1 or x > SIZE_X + 0.1 or y < -0.1 or y > SIZE_Y + 0.1) and grid[r][c]:
                grid[r][c] = 205                  # 205 = unknown
    pgm = os.path.join(PKG, 'maps', f'{WORLD_NAME}.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(bytes(v for row in grid for v in row))
    with open(os.path.join(PKG, 'maps', f'{WORLD_NAME}.yaml'), 'w') as f:
        f.write(f'image: {WORLD_NAME}.pgm\nmode: trinary\nresolution: {RES}\n'
                f'origin: [{ox}, {oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm


if __name__ == '__main__':
    print(write_world())
    print(write_map())
