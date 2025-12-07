# My Robot Package (ROS 2)

自分用のロボット制御パッケージ。

## 環境
- WSL2 (Ubuntu 22.04)
- ROS 2 Humble

## よく使う独自コマンド (エイリアス)
- `cb`: ビルドして環境再読み込み (colcon build)
- `cs`: ソースコードフォルダへ移動
- `cw`: ワークスペースのルートへ移動

## 実行方法
**ジョイスティック側:**
```bash
ros2 run my_robot_pkg joy_controller

**モーター側:**
```bash
```
ros2 run my_robot_pkg dynamixel_driver

メモ
USBデバイスを繋ぐときは usbipd attach ... を忘れない
ファイルを編集したら必ず cb する
