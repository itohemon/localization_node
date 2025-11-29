#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
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

    // 予測ステップ：オドメトリ情報に基づいてパーティクルを移動させる
    void motionUpdate(const nav_msgs::msg::Odometry::SharedPtr current_odom)
    {
        // オドメトリの差分計算 (dx, dy, dtheta)
        // 複雑なオドメトリモデルの代わりに、ここでは簡易な差分を使用
        double dx = current_odom->pose.pose.position.x - last_odom_->pose.pose.position.x;
        double dy = current_odom->pose.pose.position.y - last_odom_->pose.pose.position.y;
        // 角度差分はQuaternionを変換して計算が必要（省略）
        // double dtheta = ... ;

        // ここでは便宜上、簡単な移動とランダムノイズを適用
        double dtheta = 0.05; // 簡易的な角度変化の仮定
        double dist = std::hypot(dx, dy);

        // ノイズモデル
        std::normal_distribution<> motion_noise_dist(0.0, 0.05); // 標準偏差 0.05m/rad

        for (auto &p : particles_)
        {
            // 現在の姿勢 (p.x, p.y, p.theta) にノイズを加えて移動
            // 新しい姿勢 = 古い姿勢 + (移動量 * ノイズ)
            double noise_x = motion_noise_dist(random_engine);
            double noise_y = motion_noise_dist(random_engine);
            double noise_theta = motion_noise_dist(random_engine);

            p.x += (dist + noise_x) * std::cos(p.theta);
            p.y += (dist + noise_y) * std::sin(p.theta);
            p.theta += (dtheta + noise_theta);

            // 角度を [-pi, pi] に正規化
            p.theta = std::fmod(p.theta + M_PI, 2.0 * M_PI);
            if (p.theta < 0)
                p.theta += 2.0 * M_PI;
            p.theta -= M_PI;
        }
    }

    // 更新ステップ：レーザースキャンとマップの一致度に基づいて重みを計算
    void measurementUpdate(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        double total_weight = 0.0;

        // 観測ノイズモデル（標準偏差）
        std::normal_distribution<> measurement_noise_dist(0.0, 0.2);

        for (auto &p : particles_)
        {
            double likelihood = 1.0;

            // 📌 重要: ここに観測モデルのロジックを実装する
            // 1. 各パーティクルpの位置から、スキャンビームの終端座標 (x_scan, y_scan) を計算する。
            // 2. マップデータ (map_data_) 上の (x_scan, y_scan) の占有確率（Occupancy Grid Value）を取得する。
            // 3. 占有確率が高い（障害物に当たっている）ほど、そのパーティクルは実測スキャンと一致していると見なす。

            // 例: 簡易的な実装（ここではマップとのチェックを省略し、観測ノイズのみを考慮）
            // 実際のAMCLでは、ビームエンドポイントモデルや尤度場モデルを使用します。

            // 全スキャン点の平均尤度を計算 (ここはスタブ)
            double avg_match_score = 0.0;
            for (size_t i = 0; i < scan->ranges.size(); ++i)
            {
                // ... 観測モデルの実装 ...
                // avg_match_score += score_from_map_check;
            }

            // 簡易尤度の計算
            // scoreが高いほど尤度が高くなるようにする
            likelihood = std::exp(avg_match_score * (-0.5)); // 例

            p.weight *= likelihood;
            total_weight += p.weight;
        }

        // 重みの正規化
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

        // ルーレット選択（またはLow Variance Sampling）の実施
        std::uniform_real_distribution<> weight_dist(0.0, total_weight);

        for (int i = 0; i < num_particles_; ++i)
        {
            double random_val = weight_dist(random_engine);

            // ランダム値がどのパーティクルの累積重みに該当するかを探す
            auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), random_val);
            size_t index = std::distance(cumulative_weights.begin(), it);

            if (index < particles_.size())
            {
                // 重みが大きいパーティクルをコピーして再生成
                new_particles.push_back(particles_[index]);
                new_particles.back().weight = 1.0 / num_particles_; // 重みを均等にリセット
            }
            else
            {
                // エラー時のフォールバック
                new_particles.push_back(particles_[0]);
                new_particles.back().weight = 1.0 / num_particles_;
            }
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
