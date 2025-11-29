# ROS 2での二次元自己位置推定ノードの作成ステップ

C++ノードを作成する場合、主に以下の要素を考慮する必要があります。

1. 必要な入力データ自己位置推定を行うためには、通常、以下のデータが必要です。

    |データ|ROS 2トピックの型（一例）|説明|
    |--|--|--|
    |マップ|nav_msgs/msg/OccupancyGrid|環境の静的な地図情報。|
    |レーザースキャン|sensor_msgs/msg/LaserScan|ロボットの周辺にある障害物までの距離データ。|
    |オドメトリ|nav_msgs/msg/Odometry|ロボットの車輪エンコーダなどから得られる相対的な移動量（位置と姿勢）。|TF (Transform)|tf2_msgs/msg/TFMessage|座標系間の変換情報（例: base_linkからlaserへの変換）。|

2. ノードの設計と実装
C++ノードでは、以下の処理を実装します。

    - サブスクライバーの作成: 上記の入力データ（マップ、スキャン、オドメトリ）を受信するサブスクライバーを作成します。
    - フィルタリングアルゴリズムの実装: 自己位置推定の核となるアルゴリズムを実装します。最も一般的なのは、パーティクルフィルター（AMCLが使用しているもの）です。
    - パーティクルフィルターの概要:
        1. 初期化: マップ上の初期位置の候補（パーティクル）をランダムにばらまきます。
        2. 予測 (Prediction): オドメトリ情報に基づいて、各パーティクルを移動させます。
        3. 更新 (Update): レーザースキャンデータと地図を比較し、現実の環境と一致するパーティクルに高い重みを与えます。
        4. リサンプリング (Resampling): 重みに応じてパーティクルを再生成し、次に備えます。

    - パブリッシャーの作成: 推定されたロボットの位置と姿勢を**geometry_msgs/msg/PoseWithCovarianceStamped**（またはAMCLと同様にTF）としてパブリッシュします。
    - TFブロードキャスター: 推定結果に基づき、世界座標系（map）からオドメトリ座標系（odom）への変換（$T_{map \rightarrow odom}$）をブロードキャストします。

3. C++でのセットアップ
ROS 2 C++ノードを作成する際の一般的な手順です。

    1. ワークスペースとパッケージの作成:
    ```Bash
    ros2 pkg create --build-type ament_cmake localization_node --dependencies rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros
    ```

    2. ソースコードの記述: localization_node/src/localization_node.cppなどにC++コードを記述し、サブスクライバーやパーティクルフィルターのロジックを実装します。

    3. CMakeLists.txtの編集: 実行可能ファイルをビルドするように設定します。

    4. ビルドと実行: colcon buildでビルドし、ros2 run localization_node localization_nodeで実行します。