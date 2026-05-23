# rb300_nav

ROS2 Humble 用ナビゲーションパッケージ。SLAM Toolbox を使った地図生成機能を提供します。

---

## 概要

`rb300_nav` は、ロボット `RB300` のナビゲーションを担う ROS2 パッケージです。
現在は **SLAM Toolbox (`online_async_node`)** によるリアルタイム地図生成をサポートしています。

- **購読トピック**: `/scan`（LiDAR）、`/odom`（オドメトリ）
- **発行 TF**: `map` → `odom`
- **モード**: mapping（地図生成）

---

## 前提条件

- ROS2 Humble
- `slam_toolbox` パッケージがインストール済みであること
  ```bash
  sudo apt install ros-humble-slam-toolbox
  ```
- 以下の TF ツリーが既に配信されていること
  - `odom` → `base_link`
  - `base_link` → `laser`（または LiDAR のフレーム）

---

## ファイル構成

```
rb300_nav/
├── CMakeLists.txt              # ament_cmake ビルド設定
├── package.xml                 # パッケージ依存関係
├── config/
│   └── slam_toolbox.yaml       # SLAM Toolbox パラメータ
├── launch/
│   └── slam.launch.py          # 地図生成ランチファイル
└── README.md                   # このファイル
```

---

## ビルド

```bash
cd ~/ros2_ws
# パッケージを src に配置済みの場合
colcon build --packages-select rb300_nav
source install/setup.bash
```

---

## 使い方

### 地図生成（SLAM）

```bash
ros2 launch rb300_nav slam.launch.py
```

- `/scan` と `/odom` を元に、リアルタイムで地図を構築します。
- 可視化したい場合は別途 RViz2 を起動し、`/map` トピックを表示してください。

### 地図の保存（コマンドライン）

SLAM 実行中に、以下のサービスコールで地図を `.pgm` / `.yaml` として保存できます。

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/username/maps/my_map'}}"
```

> 拡張子 `.yaml` は自動で付与されます。

---

## パラメータ

`config/slam_toolbox.yaml` で主な設定を変更できます。

| パラメータ | デフォルト | 説明 |
|------------|-----------|------|
| `map_frame` | `map` | 地図のフレームID |
| `odom_frame` | `odom` | オドメトリのフレームID |
| `base_frame` | `base_link` | ロボットベースのフレームID |
| `scan_topic` | `/scan` | LiDAR スキャントピック |
| `resolution` | `0.05` | 地図の解像度 [m/pixel] |
| `max_laser_range` | `20.0` | LiDAR の最大有効距離 [m] |

---

## 今後の予定

- [ ] 地図保存ボタン連携（WebUI）
- [ ] 地図選択・定位（AMCL + Map Server）
- [ ] 経路計画・ナビゲーション（Nav2）
