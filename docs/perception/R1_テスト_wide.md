# 認識層(M8)の学習とテスト

- 学習: 1400 件(scan_warehouse_s1.jsonl, scan_warehouse_s2.jsonl, scan_warehouse_s4.jsonl)、障害物あり 598
- テスト: 800 件(scan_warehouse_wide_s1.jsonl, scan_warehouse_wide_s2.jsonl, scan_warehouse_wide_s3.jsonl)、障害物あり 349
- 規則: 確認帯の点数 ≥ 16。学習: 格子の点数 192 + 相対位置 4 → 全結合 32 → 1

| 判定 | 正解率 | 障害物あり→あり | あり→なし(見逃し) | なし→あり(誤登録) | なし→なし |
|---|---|---|---|---|---|
| 規則(点数 ≥ 16) | 73.6% | 159 | 190 | 21 | 430 |
| 規則 + LiDAR 位置の補正 | 75.9% | 162 | 187 | 6 | 445 |
| 学習 | 84.8% | 325 | 24 | 98 | 353 |

## 内訳(テスト)

| ワールド | 通路 | 物体 | 件数 | 規則の誤り | 補正した規則の誤り | 学習の誤り |
|---|---|---|---|---|---|---|
| warehouse_wide | middle_aisle | none | 54 | 7 | 0 | 3 |
| warehouse_wide | middle_aisle | pallet | 38 | 8 | 8 | 2 |
| warehouse_wide | middle_aisle | robot | 31 | 15 | 15 | 6 |
| warehouse_wide | north_aisle | none | 160 | 2 | 1 | 3 |
| warehouse_wide | north_aisle | pallet | 97 | 38 | 29 | 6 |
| warehouse_wide | north_aisle | robot | 91 | 56 | 55 | 9 |
| warehouse_wide | south_aisle | none | 168 | 4 | 4 | 72 |
| warehouse_wide | south_aisle | pallet | 73 | 21 | 21 | 7 |
| warehouse_wide | south_aisle | robot | 88 | 60 | 60 | 14 |

## 障害物までの距離別(テスト。障害物なしは全て含む)

| 範囲 | 件数 | 規則 見逃し / 誤登録 | 補正した規則 見逃し / 誤登録 | 学習 見逃し / 誤登録 |
|---|---|---|---|---|
| 障害物なし(帯の外を含む) | 451 | 0 / 21 | 0 / 6 | 0 / 98 |
| 障害物あり 1.0 m 未満 | 57 | 15 / 0 | 13 / 0 | 5 / 0 |
| 障害物あり 1.0〜2.0 m | 215 | 111 / 0 | 109 / 0 | 12 / 0 |
| 障害物あり 2.0 m 以上 | 77 | 64 / 0 | 65 / 0 | 7 / 0 |

- 障害物なし(または帯の外)の点数: 元の規則で 16 以上 21 件、補正した規則で 6 件
- 障害物ありの点数: 元の規則で 16 未満 190 件、補正した規則で 187 件
