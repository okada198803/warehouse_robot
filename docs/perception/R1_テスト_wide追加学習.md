# 認識層(M8)の学習とテスト

- 学習: 2000 件(scan_warehouse_s1.jsonl, scan_warehouse_s2.jsonl, scan_warehouse_s4.jsonl, scan_warehouse_wide_s1.jsonl, scan_warehouse_wide_s2.jsonl)、障害物あり 858
- テスト: 200 件(scan_warehouse_wide_s3.jsonl)、障害物あり 89
- 規則: 確認帯の点数 ≥ 16。学習: 格子の点数 192 + 相対位置 4 → 全結合 32 → 1

| 判定 | 正解率 | 障害物あり→あり | あり→なし(見逃し) | なし→あり(誤登録) | なし→なし |
|---|---|---|---|---|---|
| 規則(点数 ≥ 16) | 72.0% | 35 | 54 | 2 | 109 |
| 規則 + LiDAR 位置の補正 | 75.5% | 40 | 49 | 0 | 111 |
| 学習 | 94.5% | 83 | 6 | 5 | 106 |

## 内訳(テスト)

| ワールド | 通路 | 物体 | 件数 | 規則の誤り | 補正した規則の誤り | 学習の誤り |
|---|---|---|---|---|---|---|
| warehouse_wide | middle_aisle | none | 12 | 2 | 0 | 2 |
| warehouse_wide | middle_aisle | pallet | 9 | 0 | 1 | 0 |
| warehouse_wide | middle_aisle | robot | 9 | 6 | 6 | 1 |
| warehouse_wide | north_aisle | none | 40 | 0 | 0 | 0 |
| warehouse_wide | north_aisle | pallet | 19 | 8 | 5 | 1 |
| warehouse_wide | north_aisle | robot | 19 | 15 | 14 | 4 |
| warehouse_wide | south_aisle | none | 43 | 0 | 0 | 0 |
| warehouse_wide | south_aisle | pallet | 18 | 5 | 4 | 0 |
| warehouse_wide | south_aisle | robot | 31 | 20 | 19 | 3 |

## 障害物までの距離別(テスト。障害物なしは全て含む)

| 範囲 | 件数 | 規則 見逃し / 誤登録 | 補正した規則 見逃し / 誤登録 | 学習 見逃し / 誤登録 |
|---|---|---|---|---|
| 障害物なし(帯の外を含む) | 111 | 0 / 2 | 0 / 0 | 0 / 5 |
| 障害物あり 1.0 m 未満 | 14 | 6 / 0 | 3 / 0 | 0 / 0 |
| 障害物あり 1.0〜2.0 m | 50 | 26 / 0 | 25 / 0 | 3 / 0 |
| 障害物あり 2.0 m 以上 | 25 | 22 / 0 | 21 / 0 | 3 / 0 |

- 障害物なし(または帯の外)の点数: 元の規則で 16 以上 2 件、補正した規則で 0 件
- 障害物ありの点数: 元の規則で 16 未満 54 件、補正した規則で 49 件
