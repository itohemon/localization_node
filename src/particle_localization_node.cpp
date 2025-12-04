#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <random>
#include <cmath>
#include <algorithm>

// パーティクル構造体
struct Particle
{
    double x, y, theta; // 姿勢 (Pose)
    double weight;      // 重み
};

class ParticleLocalizationNode : public rclcpp::Node
{
public:
    ParticleLocalizationNode() : Node("particle_localization_node"), random_engine(std::random_device{}())
    {
        // パラメータの宣言と取得
        this->declare_parameter("num_particles", 100);
        num_particles_ = this->get_parameter("num_particles").as_int();

        // 📌 サブスクライバーの設定
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&ParticleLocalizationNode::mapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ParticleLocalizationNode::odomCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ParticleLocalizationNode::scanCallback, this, std::placeholders::_1));

        // 📢 パブリッシャーの設定
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 1);
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 1);

        // TFブロードキャスターの設定
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // パーティクルの初期化
        initializeParticles();
    }

private:
    int num_particles_;
    std::vector<Particle> particles_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::default_random_engine random_engine;

    // --- コールバック関数 ---

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Map received. Map size: %d x %d", msg->info.width, msg->info.height);
        map_data_ = msg;
        // マップ全体に初期パーティクルを再分布させるロジックをここに追加しても良い
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!last_odom_)
        {
            last_odom_ = msg;
            return;
        }

        // 運動モデル（Prediction）の実行
        motionUpdate(msg);

        // 次の処理のためにオドメトリを保存
        last_odom_ = msg;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!map_data_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for map data...");
            return;
        }

        // 観測モデル（Update）の実行
        measurementUpdate(msg);

        // リサンプリングの実行
        resampleParticles();

        // 結果のパブリッシュ
        publishResults();
    }

    // --- パーティクルフィルタのコアロジック ---

    void initializeParticles()
    {
        particles_.clear();
        double initial_weight = 1.0 / num_particles_;
        std::uniform_real_distribution<> x_dist(-1.0, 1.0); // 例として(-1m, 1m)で初期化
        std::uniform_real_distribution<> y_dist(-1.0, 1.0);
        std::uniform_real_distribution<> theta_dist(0.0, 2.0 * M_PI);

        for (int i = 0; i < num_particles_; ++i)
        {
            particles_.push_back({x_dist(random_engine),
                                  y_dist(random_engine),
                                  theta_dist(random_engine),
                                  initial_weight});
        }
        RCLCPP_INFO(this->get_logger(), "Particles initialized: %d", num_particles_);
    }

    // クォータニオンからヨー角[rad]を抽出するヘルパー関数
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q); // geometry_msgs::msg::Quaternionをtf2::Quaternionに変換

        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // ロール、ピッチ、ヨーを抽出

        return yaw;
    }

    // 予測ステップ：オドメトリ情報に基づいてパーティクルを移動させる
    void motionUpdate(const nav_msgs::msg::Odometry::SharedPtr current_odom)
    {
        if (!last_odom_)
            return;

        // 1. オドメトリの絶対姿勢を取得
        double current_x = current_odom->pose.pose.position.x;
        double current_y = current_odom->pose.pose.position.y;
        double current_theta = get_yaw_from_quaternion(current_odom->pose.pose.orientation);

        double last_x = last_odom_->pose.pose.position.x;
        double last_y = last_odom_->pose.pose.position.y;
        double last_theta = get_yaw_from_quaternion(last_odom_->pose.pose.orientation);

        // 2. odom座標系における移動差分(delta_dist, delta_theta)を計算
        // ここで計算されるのは、ロボットのローカル座標系ではなく、odom座標系における移動
        double delta_x_odom = current_x - last_x;
        double delta_y_odom = current_y - last_y;
        double delta_theta = current_theta - last_theta;

        // 角度差分を[-pi, pi]に正規化(回転が360度を超えても正しく扱うため)
        while (delta_theta > M_PI)
            delta_theta -= 2.0 * M_PI;
        while (delta_theta < -M_PI)
            delta_theta += 2.0 * M_PI;

        // odom座標系ではなく、ロボットのローカル座標系における移動量(delta_forward)を計算
        // これは、パーティクルを移動させるための「真の移動指令」に近い
        double delta_dist = std::hypot(delta_x_odom, delta_y_odom);

        // ローカル座標系での前方移動量
        // odom座標系での移動ベクトルを、last_thetaで逆回転してローカル座標系に変換
        double delta_forward = std::cos(last_theta) * delta_x_odom + std::sin(last_theta) * delta_y_odom;
        double delta_sideways = -std::sin(last_theta) * delta_x_odom + std::cos(last_theta) * delta_y_odom;

        // 3. ノイズモデル(Motion Model)を適用してパーティクルを移動
        std::normal_distribution<> forward_noise_dist(0.0, 0.05); // 前方移動ノイズ
        std::normal_distribution<> turn_noise_dist(0.0, 0.01);    // 回転ノイズ

        for (auto &p : particles_)
        {
            // ノイズを加えた移動量
            double noisy_delta_forward = delta_forward + forward_noise_dist(random_engine);
            double noisy_delta_theta = delta_theta + turn_noise_dist(random_engine);

            // ローカル座標系の移動量を、パーティクルの姿勢(p.theta)に基づいてマップ座標系に変換
            p.x += noisy_delta_forward * std::cos(p.theta);
            p.y += noisy_delta_forward * std::sin(p.theta);
            p.theta += noisy_delta_theta;

            // 角度を [-pi, pi] に正規化
            p.theta = std::fmod(p.theta + M_PI, 2.0 * M_PI);
            if (p.theta < 0)
                p.theta += 2.0 * M_PI;
            p.theta -= M_PI;
        }
    }

    // ワールド座標 (メートル) をマップのグリッドセル座標 (インデックス) に変換
    bool worldToMap(double world_x, double world_y, int &map_x, int &map_y,
                    const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
    {
        if (!map)
            return false;

        // マップの原点 (origin) からの相対位置
        double relative_x = world_x - map->info.origin.position.x;
        double relative_y = world_y - map->info.origin.position.y;

        // 解像度 (resolution) を使ってセルインデックスに変換
        map_x = static_cast<int>(std::floor(relative_x / map->info.resolution));
        map_y = static_cast<int>(std::floor(relative_y / map->info.resolution));

        // マップの境界チェック
        return map_x >= 0 && map_x < (int)map->info.width &&
               map_y >= 0 && map_y < (int)map->info.height;
    }

    // マップのグリッドセル値を取得する
    int getMapValue(int map_x, int map_y, const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
    {
        if (!map || map_x < 0 || map_x >= (int)map->info.width ||
            map_y < 0 || map_y >= (int)map->info.height)
        {
            return -1; // 範囲外
        }
        // 一次元配列のインデックス計算: index = y * width + x
        return map->data[map_y * map->info.width + map_x];
    }

    // 更新ステップ：レーザースキャンとマップの一致度に基づいて重みを計算
    void measurementUpdate(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!map_data_)
            return;

        double total_weight = 0.0;

        // 観測ノイズモデル（標準偏差）：一致しなかった場合の尤度のガウス分布
        std::normal_distribution<> measurement_noise_dist(0.0, 0.2);

        // レーザーの原点から本体までのオフセット (TFが必要だが、ここでは簡易的にゼロとする)
        // const double laser_offset_x = 0.0;
        // const double laser_offset_y = 0.0;

        // 尤度を計算するビームの間隔を定義 (計算負荷軽減のため)
        const int step = 5; // 5ビームごとにチェックする

        for (auto &p : particles_)
        {
            double log_likelihood_sum = 0.0; // 対数尤度の和

            for (size_t i = 0; i < scan->ranges.size(); i += step)
            {
                double range = scan->ranges[i];

                // 範囲外や不明な値はスキップ
                if (std::isinf(range) || std::isnan(range) || range > scan->range_max)
                {
                    continue;
                }

                // 1. パーティクルの姿勢に基づいたビームの絶対角度を計算
                double angle = p.theta + scan->angle_min + i * scan->angle_increment;

                // 2. ビームの終点座標 (ワールド座標) を計算
                double end_x = p.x + range * std::cos(angle);
                double end_y = p.y + range * std::sin(angle);

                // 3. マップグリッドセルインデックスに変換
                int map_x, map_y;
                if (!worldToMap(end_x, end_y, map_x, map_y, map_data_))
                {
                    // マップの範囲外なら、この観測は無視
                    continue;
                }

                // 4. マップ値を取得
                int map_value = getMapValue(map_x, map_y, map_data_);

                // 5. 尤度の評価 (簡易モデル)
                // マップ値は通常 0 (空き) から 100 (占有) の値を取る
                double match_score;
                if (map_value > 70)
                {
                    // 障害物に当たっているセルと一致 -> 高い尤度
                    match_score = 0.9;
                }
                else if (map_value < 10)
                {
                    // 空きセルと一致 -> 低い尤度
                    match_score = 0.05;
                }
                else
                {
                    // 不明なセルや中間値 -> 中程度の尤度
                    match_score = 0.3;
                }

                // 対数尤度を追加 (積の計算を和に変換し、数値安定性を高める)
                // log(p_i) = log(match_score)
                log_likelihood_sum += std::log(match_score);
            }

            // 6. 重みの更新
            // log尤度の和を指数関数に戻して、パーティクルの重みに乗算
            p.weight *= std::exp(log_likelihood_sum);
            total_weight += p.weight;
        }

        // 7. 重みの正規化
        if (total_weight > 0.0)
        {
            for (auto &p : particles_)
            {
                p.weight /= total_weight;
            }
        }
    }

    // リサンプリングステップ：重みに応じてパーティクルを再抽出
    void resampleParticles()
    {
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);

        // 累積分布の作成
        std::vector<double> cumulative_weights;
        double total_weight = 0.0;
        for (const auto &p : particles_)
        {
            total_weight += p.weight;
            cumulative_weights.push_back(total_weight);
        }

        // **重みがゼロの状況を避けるガード**
        if (total_weight == 0.0)
        {
            // 全ての重みがゼロの場合、パーティクルを再初期化するか、均等重みにリセットする。
            // ここでは簡易的に処理をスキップ
            RCLCPP_WARN(this->get_logger(), "Total weight is zero. Resampling skipped.");
            return;
        }

        // ルーレット選択（またはLow Variance Sampling）の実施
        std::uniform_real_distribution<> weight_dist(0.0, total_weight);

        for (int i = 0; i < num_particles_; ++i)
        {
            double random_val = weight_dist(random_engine);

            // ランダム値がどのパーティクルの累積重みに該当するかを二分探索で高速に探す
            // std::lower_bound は、random_val以上の値を持つ最初の要素を指すイテレータを返す
            auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), random_val);
            size_t index = std::distance(cumulative_weights.begin(), it);

            // 選択されたパーティクルを新しいリストに追加
            if (index < particles_.size())
            {
                new_particles.push_back(particles_[index]);
            }
            else
            {
                // エラー時のフォールバック (通常は起こらないはず)
                new_particles.push_back(particles_[0]);
            }
            // 重みを均等にリセット（重要）
            new_particles.back().weight = 1.0 / num_particles_;
        }

        particles_ = new_particles;
    }

    // --- 結果のパブリッシュ ---

    void publishResults()
    {
        if (particles_.empty())
            return;

        // 重み付き平均による推定姿勢の計算
        double pose_x = 0.0, pose_y = 0.0, pose_theta_sin = 0.0, pose_theta_cos = 0.0;
        double total_weight = 0.0;

        for (const auto &p : particles_)
        {
            pose_x += p.x * p.weight;
            pose_y += p.y * p.weight;
            pose_theta_sin += std::sin(p.theta) * p.weight;
            pose_theta_cos += std::cos(p.theta) * p.weight;
            total_weight += p.weight;
        }

        // 推定姿勢の正規化
        if (total_weight == 0.0)
            return;
        pose_x /= total_weight;
        pose_y /= total_weight;
        double pose_theta = std::atan2(pose_theta_sin, pose_theta_cos);

        // 1. 推定姿勢のパブリッシュ
        geometry_msgs::msg::PoseWithCovarianceStamped estimated_pose;
        estimated_pose.header.frame_id = "map";
        estimated_pose.header.stamp = this->now();
        estimated_pose.pose.pose.position.x = pose_x;
        estimated_pose.pose.pose.position.y = pose_y;

        tf2::Quaternion q;
        q.setRPY(0, 0, pose_theta);
        estimated_pose.pose.pose.orientation = tf2::toMsg(q);
        pose_pub_->publish(estimated_pose);

        // 2. パーティクル位置のパブリッシュ (RVizで確認用)
        geometry_msgs::msg::PoseArray particle_array;
        particle_array.header.frame_id = "map";
        particle_array.header.stamp = this->now();
        for (const auto &p : particles_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            tf2::Quaternion q_p;
            q_p.setRPY(0, 0, p.theta);
            pose.orientation = tf2::toMsg(q_p);
            particle_array.poses.push_back(pose);
        }
        particles_pub_->publish(particle_array);

        // 3. TF (map -> odom) のブロードキャスト
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom"; // odomの親はmapになる
        transform_stamped.transform.translation.x = pose_x;
        transform_stamped.transform.translation.y = pose_y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation = estimated_pose.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
