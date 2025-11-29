# localization_node
Geminiから二次元自己位置推定ノードを学ぶ

## 前提
Ubuntu24.04とROS2 jazzyで動作確認

## ビルド
```sh-session
colcon build --packages-select localization_node
```

## 実行
```sh-session
ros2 launch localization_node bringup.launch.py 
```

