#include "localizer/localizer.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// デフォルトコンストラクタ
// パラメータの宣言と取得
// Subscriber，Publisherの設定
// frame idの設定
// パーティクルクラウドのメモリの確保
// odometryのモデルの初期化
Localizer::Localizer() : Node("teamb_localizer")
{ 
    // パラメータの宣言
    this->declare_parameter("hz", 10);

    // パラメータの取得
    this->get_parameter("hz", hz_);

    // Subscriberの設定
    // <subscriber名> = this->create_subscription<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable(), std::bind(&<class名>::<コールバック関数名>, this, std::placeholders::_1));
    // std::bindを使ってsubするコールバック関数，std::placeholdersを使ってその関数内での引数を指定する
    // std::placeholdersで指定する引数は大体1番目のもの（コールバック関数の引数が1つであるため）
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable(), std::bind(&Localizer::map_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1).reliable(), std::bind(&Localizer::odom_callback, this, std::placeholders::_1));
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(1).reliable(), std::bind(&Localizer::laser_callback, this, std::placeholders::_1));

    // Publisherの設定
    // <publisher名> = this->create_publisher<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable());
    pub_estimated_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/e_pose", rclcpp::QoS(1).reliable());
    pub_particle_cloud_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/p_cloud", rclcpp::QoS(1).reliable());

    // frame idの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";

    // パーティクルクラウドのメモリの確保
    particle_cloud_msg_.poses.reserve(max_particle_num_);

    // ！odometryのモデルの初期化
    OdomModel odom_model_(0.0, 0.0, 0.0, 0.0);

    // fw_var_per_fw_ = 0.0;
    // fw_var_per_rot_ = 0.0;
    // rot_var_per_fw_ = 0.0;
    // rot_var_per_rot_ = 0.0;

    // odom_model_.pose.orientation.z = 0.0; // 回転のz成分
    // odom_model_.pose.orientation.w = 1.0; // 初期角度0°のための式合わせ
    // odom_model_.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); も可
    // odom_model_.pose.orientation.z = 0.0; だけでは4次元数の条件x^2 + y^2 + z^2 + w^2 = 1を満たすと限らない為不適
}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    flag_odom_ = true; // マップのmsg受け取りフラグ
}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{   
    prev_odom_ = last_odom_; // 1つ前のオドメトリを保存
    last_odom_ = *msg;       // 最新のオドメトリを保存
    flag_map_ = true; // オドメトリのmsg受け取りフラグ

    double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    double distance_moved = std::sqrt(dx * dx + dy * dy); // ロボットが動いた距離
    
    if (distance_moved > 0.01) { // 1cm以上
        flag_move_ = true; // 機体動いた
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    // ！Nodeで定義したsub_odom_はmsgでなくSubscriptionなので,直接poseにアクセスできない
    // → コールバック関数でodomのmsgデータ取得した後、他の関数からsub_odom_でそのデータを使えposeにアクセスできるように設定する


    // (オドメトリのノイズを考慮した補正)
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_laser_ = true; // レーザーのmsg受け取りフラグ
}

// hz_を返す関数
int Localizer::getOdomFreq() // (constを付けることでhz_の変更がないことを保証出来る)
{
    return hz_;
}

// ロボットとパーティクルの推定位置の初期化
void Localizer::initialize()
{
    // 推定位置の初期化
    Pose estimated_pose_(init_x_, init_y_, init_yaw_);

    // estimated_pose_.x_ = init_x_;
    // estimated_pose_.y_ = init_y_;
    // estimated_pose_.yaw_ = init_yaw_;
    // tf2::Quaternion quat(estimated_pose_.x, estimated_pose_.y, estimated_pose.orientation.z, estimated_pose.orientation.w);
    // tf2::Matrix3x3(quat).getEulerYPR(estimate_pose.yaw_, pitch, roll);  // yawはラジアン単位で取得
    
    // パーティクルの初期化
    particles_.clear();

    for (int i = 0; i < particle_num_; i++) {
        // 初期位置近傍に標準偏差を誤差としたパーティクルを配置
        double particle_weight;
        Particle particle(init_x_ + norm_rv(0, init_x_dev_), init_y_ + norm_rv(0, init_y_dev_), init_yaw_ + norm_rv(0, init_yaw_dev_), particle_weight); // Particleとは一つのパーティクル作成のための構造体(x, y, yaw, weight入り)
        
        // particle.x = init_x_ + norm_rv(0, init_x_dev_);
        // particle.y = init_y_ + norm_rv(0, init_y_dev_);
        // particle.yaw = init_yaw_ + norm_rv(0, init_yaw_dev_);

        // パーティクルの重みの初期化とリストに追加
        reset_weight(particle);
    }
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_) // map,odom,laserの値を取得できたとき
    {
        // tfのブロードキャスト(他のノードに送信すること)、ros1ならtf::でros2ならtf2_ros::
        static tf2_ros::TransformBroadcaster br; // tfの座標変換情報をブロードキャストするためのオブジェクト
        tf2_ros::Transform transform; // Transformとはtfの座標変換を表すクラスでロボットの平行移動を表すtf::Vector3と回転情報を表すtf::Quaternionを持つ
        std::transform.setOrigin(tf2_ros::Vector3(estimated_pose_.x, estimated_pose_.y, 0.0) ); // 位置を設定
        tf2_ros::Quaternion q;
        q.setRPY(0, 0, estimated_pose_.yaw); // 回転情報を入力(Roll, Pitch, Yaw)の順
        transform.setRotation(q); // 回転を設定
        br.sendTransform(tf2_ros::StampedTransform(std::transform, ros::Time::now(), "world", "robot")); // (座標系, タイムスタンプとして使う時刻, 親座標系の名前, 今使っている座標系の名前)  

        flag_broadcast_ = true; // tfブロードキャストした

    //     // 初回のみ TransformBroadcaster を初期化
    // if (!tf_broadcaster_) {
    //     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // }

    // // 変換情報を格納する TransformStamped メッセージ
    // geometry_msgs::msg::TransformStamped transform;

    // // ここで transform の座標情報を設定
    // transform.header.stamp = this->now();
    // transform.header.frame_id = "map";
    // transform.child_frame_id = "base_link";
    // transform.transform.translation.x = 1.0;
    // transform.transform.translation.y = 2.0;
    // transform.transform.translation.z = 0.0;
    // transform.transform.rotation.x = 0.0;
    // transform.transform.rotation.y = 0.0;
    // transform.transform.rotation.z = 0.0;
    // transform.transform.rotation.w = 1.0;

    // // 座標変換をブロードキャスト
    // tf_broadcaster_->sendTransform(transform);

        // 位置推定
        localize();
        
        // パブリッシュ
        publish_estimated_pose(); // 推定位置
        publish_particles(); // パーティクルクラウド
    }
}

// 適切な角度(-M_PI ~ M_PI)を返す
double Localizer::normalize_angle(double angle)
{
    while (angle < -M_PI && M_PI < angle) { // 適切な範囲外のとき
        if (angle < -M_PI) {
            angle += 2 * M_PI;
        } else 
            angle -= 2 * M_PI;
    }
    return angle;
}

// ランダム変数生成関数（正規分布に従った）
double Localizer::norm_rv(const double mean, const double stddev) // 平均と標準偏差
{
    static std::default_random_engine generator; // 乱数生成
    std::normal_distribution<double> distribution(mean, stddev); // 正規分布生成器の初期化

    return distribution(generator);
}

// パーティクルの重みの初期化しリストに追加(正規分布に基づいて初期値設定、正規化→初期化？)
void Localizer::reset_weight()
{
    double another_weight = 1.0 / particle_num_; // 初期重み1.0を均等配分
    particles_.push_back(another_weight); // hppで定義済みのリストに追加
}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
// map座標系からみたbase_link座標系の位置と姿勢，odom座標系からみたbase_link座標系の位置と姿勢から計算
void Localizer::broadcast_odom_state()
{
    if(flag_broadcast_) // tfをブロードキャストしたとき
    {
        // TF Broadcasterの実体化
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        // broadcasterの初期化
        odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系(地図の静的な絶対座標)からみたbase_link座標系(ロボットの中心基準で動く座標)の位置と姿勢の取得
        geometry_msgs::msg::PoseStamped map_to_base;
        map_to_base.header.stamp = this->now(); // 現在時刻をセット
        map_to_base.header.frame_id = "map"; // "map" 座標系
        map_to_base.pose = estimated_pose_msg_.pose; // 推定位置をコピー

        // odom座標系(ロボットの動きを元にした相対座標)からみたbase_link座標系(ロボットの中心基準で動く座標)の位置と姿勢の取得
        geometry_msgs::msg::PoseStamped odom_to_base;
        odom_to_base.header.stamp = this->now(); // 現在時刻をセット
        odom_to_base.header.frame_id = "odom"; // "odom" 座標系、？座標系の名前若干統一されてるか不安
        odom_to_base.pose = last_odom_.pose.pose; // 最新のオドメトリをコピー

        // map座標系(地図の静的な絶対座標)からみたodom座標系(ロボットの動きを元にした相対座標)の位置と姿勢を計算（回転行列を使った単純な座標変換）
        double dx = map_to_base.pose.position.x - odom_to_base.pose.position.x;
        double dy = map_to_base.pose.position.y - odom_to_base.pose.position.y;
        double dyaw = map_to_base.pose.orientation.z - odom_to_base.pose.orientation.z;

        // (サンプルコードに記述済み)
        tf2::Quaternion map_to_odom_quat;
        map_to_odom_quat.setRPY(0, 0, dyaw);

        // yawからquaternionを作成(サンプルコードに記述済み)
        tf2::Quaternion map_to_odom_quat;

        // odom座標系odomの位置姿勢情報格納するための変数(サンプルコードに記述済み)
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納
        odom_state.header.stamp = this->now();
        RCLCPP_INFO(this->get_logger(), "Current time: %ld", this->now().nanoseconds()); // デバック用

        // 親フレーム・子フレームの指定(サンプルコードに記述済み)
        odom_state.header.frame_id = map_.header.frame_id; // 親フレーム
        odom_state.child_frame_id  = last_odom_.header.frame_id; // 子フレーム

        // map座標系からみたodom座標系の原点位置と方向の格納
        odom_state.transform.translation.x = dx;
        odom_state.transform.translation.y = dy;
        odom_state.transform.translation.z = 0.0;
        odom_state.transform.rotation = tf2::toMsg(map_to_odom_quat);

        // tf情報をbroadcast(座標系の設定)
        odom_state_broadcaster->sendTransform(odom_state);
    }

}

// 自己位置推定
// 動作更新と観測更新を行う
void Localizer::localize()
{
    motion_update();
    observation_update();
}

// 動作更新
// ロボットの微小移動量を計算し，パーティクルの位置をノイズを加えて更新
void Localizer::motion_update()
{
    if(flag_odom_)
    {   
        // ロボットの微小移動量計算
        double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
        double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
        double dyaw = last_odom_.pose.pose.position.z - prev_odom_.pose.pose.position.z; // 回転角度yawの差ってこの回転方向成分z?

        // オドメトリの標準座標
        odom_model_.set_dev(std::sqrt(dx * dx + dy * dy), std::abs(dyaw));

        // ノイズ取得
        double fx_noise = odom_model_.get_fw_noise();
        double rot_noise = odom_model_.get_rot_noise();

        // パーティクルの位置を更新
        for (auto& particle : particles_)
        {
            // ノイズを加える#hiraiwa
            double dx_add_noise = dx + norm_rv(0, fx_noise);
            double dy_add_noise = dy + norm_rv(0, fx_noise);
            double dyaw_add_noize = dyaw + norm_rv(0, rot_noise);
            double ddx,ddy,ddyaw;
             ddx += dx_add_noise;
             ddy += dy_add_noise;
             ddyaw += dyaw_add_noize;

            // 位置更新
            particle = Particle(ddx, ddy, ddyaw, particle.weight());//#hiraiwa#

        }
    }
}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    if (flag_laser_)
    {
        for(auto& particle : particles_)
        {
            // パーティクル1つのレーザ(1本?)における平均尤度を算出、重みを更新
            alpha_th_ = calc_marginal_likelihood(); // 周辺尤度を算出
            particle.weight *= alpha_th_; //！重みに尤度をかける
        }

        // ここからはそれぞれの関数内でforでparticles_回す
        // 正規化
        normalize_belief(); // 正規化 

        // 位置推定
        estimate_pose();

        // リサンプリング
        resampling(alpha_th_);
    }
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{
    double marginal_likelihood = 0.0; //周辺尤度の平均
    double yudo_ = 0.0; // likelihood()関数の戻り値として得られる各レーザの尤度
    double sum_yudo_ = 0.0; //上記の合計
    int laser_number = 0; // センサの本数

    for (int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        yudo_ = Particle::likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
        sum_yudo_ += yudo_; 
        laser_number++;
    }
    
    marginal_likelihood = sum_yudo_ / laser_number; // 全レーザの合計尤度をレーザで割って平均化
    return marginal_likelihood;
}

// 推定位置の決定☆
// 算出方法は複数ある（平均，加重平均，中央値など...）
// 加重平均
void Localizer::estimate_pose()
{
    double sum_x = 0.0, sum_y = 0.0, sum_yaw = 0.0;
    double weight_sum = 0.0;
    
    for (const auto& particle : particles_)
    {
        sum_x += particle.x * particle.weight;
        sum_y += particle.y * particle.weight;
        sum_yaw += particle.yaw * particle.weight;
        weight_sum += particle.weight;
    }
    estimated_pose_.x = sum_x;
    estimated_pose_.y = sum_y;
    estimated_pose_.yaw = sum_yaw;
}

// 重みの正規化(0から1の間、重要度重み、正規化→初期化？)
void Localizer::normalize_belief()
{
    double sum_weights = 0.0;
    for (auto& particle : particles_)
    {
        sum_weights += particle.weight; // 重みの合計を計算
    }

    if (sum_weights > 0.0)
    {
        for (auto& particle : particles_)
        {
            particle.weight() /= sum_weights; //重要度重み算出
        }
    }
}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting()
{

}

// リサンプリング（系統サンプリング）☆
// 周辺尤度に応じてパーティクルをリサンプリング
void Localizer::resampling(const double alpha)
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    int size = particles_.size();

    // particle数の動的変更(AMCL特有のサンプリング、尤度が高い時は粒子減らし尤度が低い時は粒子増やす)
    if (alpha > 0.80) // alpha(尤度)高い
    {
        particle_num_ -= (particle_num_) / 3; // 粒子減らす
    }

    else if (alpha < 0.30) // 尤度低い
    {
        particle_num_ += (particle_num_) / 3; // 粒子増やす
    }

    // サンプリングするパーティクルのインデックスを保持
    indexes.reserve(particle_num_); // インデックスの数をパーティクルの数に合わせる
    int index = 0;
    for (int i=0; i<particle_num_; i++)
    {
        index++;
        indexes.push_back(index); // indexをhppで定義済みのリストに追加
    }

    // リサンプリング
    next_particles_.reserve(particle_num_); // パーティクル数の保持
    std::move(next_particles_); // パーティクルを移動

    // 重みを初期化しhppで定義済みのリストに追加
    for (auto& particle : next_particles_) { // next_patricles_の中身のparticleごとにfor回す
        reset_weight(particle); // ！この関数の引数にnext_particles_のようなリストは来れない ↑
    }
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{
    // 位置推定結果をパブリッシュする
    estimated_pose_msg_.header.stamp = this->now();
    estimated_pose_msg_.header.frame_id = "world";
    estimated_pose_msg_.position.x = estimated_pose_.x;
    estimated_pose_msg_.position.y = estimated_pose_.y;
    estimated_pose_msg_.orientation = tf2_ros::createQuaternionMsgFromYaw(estimated_pose_.yaw);
    
    // <publisher名>->publish(<変数名>);
    pub_estimated_pose_->publish(estimated_pose_msg_);
}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    is_visible_ = true; // パーティクルクラウドをパブリッシュした

    particle_cloud_msg_.header.stamp = this->now();
    particle_cloud_msg_.poses.clear(); // 前回までのパーティクルデータの削除
    for (const auto& particle : particles_)
    {
        Pose pose;
        pose.position.x = particle.position.x;
        pose.position.y = particle.position.y;
        // ！toMsg = tf2::Quaternion → geometry_msgs::msg::Quaternion
        pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, particle.position.yaw));

        particle_cloud_msg_.poses.push_back(pose); // hppで定義済みのリストに追加
    }

    // <publisher名>->publish(<変数名>);
    pub_particle_cloud_->publish(particle_cloud_msg_);
}

// 327辺りと411 estimated_pose_
// 171辺りと262辺りと280辺りと322辺りと421辺り particle

// 421辺り pose
// 127辺り tf関連の関数(サイトのを写した)
// 307 他のcppの関数を持ってきたとき関数無いのエラー
// 393 pose.sppで定義した関数として使えているのか