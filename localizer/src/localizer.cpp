#include "localizer/localizer.hpp"

// デフォルトコンストラクタ
// パラメータの宣言と取得
// Subscriber，Publisherの設定
// frame idの設定
// パーティクルクラウドのメモリの確保
// odometryのモデルの初期化
Localizer::Localizer() : Node("team_localizer")
{ 
    // パラメータの宣言
    this->declare_parameter("hz", 10);

    // パラメータの取得
    this->get_parameter("hz", hz_);

    // timer
    // timer_ = this->create_wall_timer(<周期間隔>, std::bind(&<class名>::<callback関数名, this)); // 引数ないコールバック関数のときはこの続きの引数の数指定は要らない, walltimerを使うときは引数を指定してはいけない
    timer_ = this->create_wall_timer(10ms, std::bind(&Localizer::timer_callback,this));

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

    // odometryのモデルの初期化
    sub_odom_.pose.pose.position.x = 0.0; // (0,0)
    sub_odom_.pose.pose.position.y = 0.0;
    sub_odom_.pose.pose.orientation.x = 0.0; // 回転のx成分
    sub_odom_.pose.pose.orientation.y = 0.0; // 回転のy成分
    sub_odom_.pose.pose.orientation.z = 0.0; // 回転のz成分
    sub_odom_.pose.pose.orientation.w = 1.0; // 初期角度0°のための式合わせ
    // sub_odom_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); も可
    // sub_odom_.pose.pose.orientation.z = 0.0; だけでは4次元数の条件x^2 + y^2 + z^2 + w^2 = 1を満たすと限らない為不適
}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *mag;
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

    // ここでオドメトリのノイズを考慮した補正を入れるのもアリ
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
    estimated_pose_.x = init_x_;
    estimated_pose_.y = init_y_;
    estimated_pose_.yaw = init_yaw_; 
    
    // パーティクルの初期化
    particles_.clear();

    for (int i = 0; i < particle_num_; i++) {
        Particle particle; // Particleとは一つのパーティクル作成のための構造体(x, y, yaw, weight)

        // 初期位置近傍にパーティクルを配置
        particle.x = init_x_ + norm_rv(0, init_x_dev_); // 標準偏差を誤差とした位置
        particle.y = init_y_ + norm_rv(0, init_y_dev_);
        particle.yaw = init_yaw + norm_rv(0, init_yaw_dev_);

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
        // tfのブロードキャスト(他のノードに送信すること)
        static tf::TransformBroadcaster br; // tfの座標変換情報をブロードキャストするためのオブジェクト
        tf::Transform transform; // Transformとはtfの座標変換を表すクラスでロボットの平行移動を表すtf::Vector3と回転情報を表すtf::Quaternionを持つ
        transform.setOrigin(tf::Vector3(estimated_pose_.x, estimated_pose.y, 0.0) ); // 位置を設定
        tf::Quaternion q;
        q.setRPY(0, 0, estimated_pose_.yaw); // 回転情報を入力(Roll, Pitch, Yaw)の順
        transform.setRotation(q); // 回転を設定
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot")); // (座標系, タイムスタンプとして使う時刻, 親座標系の名前, 今使っている座標系の名前)  

        flag_broadcast_ = true; // tfブロードキャストした

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

// パーティクルの重みの初期化(正規分布に基づいて初期値設定、正規化→初期化？)
void Localizer::reset_weight(Particle &particle)
{
    particle.weight = 1.0 / particle_num_; // 初期重み1.0を均等配分
    particles_.push_back(particle); // hppで定義済みのリストに追加
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
        geometry_msgs::msg::PoseStamped map_to_base = estimated_pose_;

        // odom座標系(ロボットの動きを元にした相対座標)からみたbase_link座標系(ロボットの中心基準で動く座標)の位置と姿勢の取得
        geometry_msgs::msg::PoseStamped odom_to_base = last_odom_;

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
        double dx = prev_odom_.x - last_odom_.x;
        double dy = prev_odom_.y - last_odom_.y;
        double dyaw = prev_odom_.yaw - last_odom.yaw;

        // オドメトリの標準座標
        odom_model_.set_dev(std::sqrt(dx * dx + dy * dy), std::abs(dyaw));

        // ノイズ取得
        double fx_noise = odom_model_.get_fw_noise();
        double rot_noise = odom_model_.get_rot_noise();

        // パーティクルの位置を更新
        for (auto& particle : particles_)
        {
            // ノイズを加える
            double dx_add_noise = dx + norm_rv(0, fw_noise);
            double dy_add_noise = dy + norm_rv(0, fw_noise);
            double dyaw_add_noize = dyaw + norm_rv(0, rot_noise);

            // 位置更新
            particle.x += dx_add_noise;
            particle.y += dy_add_noize;
            particle.yaw += dyaw_add_noize;
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
            // パーティクル1つのレーザ1本における平均尤度を算出、重みを更新
            const double alpha;
            alpha = calc_marginal_likelihood(); // 周辺尤度を算出
            particle.weight *= alpha; //！重みに尤度をかける
        }

        // ここからはそれぞれの関数内でforでparticles_回す
        // 正規化
        normalize_belief(); // 正規化 

        // 位置推定
        estimate_pose();

        // リサンプリング
        resampling(const double alpha);
    }
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{
    double marginal_likelihood = 1.0;
    double likelihood_ = likelihood(map_, laser_, sensor_noise_ratio, laser_step, ignore_angle_range_list); // 各レーザの尤度を求める
    // 全レーザの合計尤度をレーザで割って平均化
    return margical_likelihood;
}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
void Localizer::estimate_pose()
{


    double sum_x = 0.0, sum_y = 0.0, sum_yaw = 0.0;
    for (const auto& particle : particles_)
    {
        sum_x += particle.x * particle.weight;
        sum_y += particle.y * particle.weight;
        sum_yaw += particle.yaw * particle.weight;
    }
    estimated_pose_.x = sum_x;
    estimated_pose_.y = sum_y;
    estimated_pose_.yaw = sum_yaw;
}

// 重みの正規化(0から1の間、重要度重み、正規化→初期化？)
void Localizer::normalize_belief()
{


    double sum_weights = 0.0;
    for (const auto& particle : particles_)
    {
        sum_weights += particle.weight;
    }

    if (sum_weights > 0.0)
    {
        for (auto& particle : particles_)
        {
            particle.weight /= sum_weights;
        }
    }
}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting()
{

}

// リサンプリング（系統サンプリング）
// 周辺尤度に応じてパーティクルをリサンプリング
void Localizer::resampling(const double alpha)
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    int size = particles_.size();

    // particle数の動的変更

    // サンプリングするパーティクルのインデックスを保持

    // リサンプリング

    // 重みを初期化
    reset_weight();
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{
        // 位置推定結果をパブリッシュする
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = estimated_pose_.x;
        pose_msg.pose.position.y = estimated_pose_.y;
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(estimated_pose_.yaw);

        pose_pub_.publish(pose_msg);
}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    if(is_visible_)
    {
        is_visible_ = true; // パーティクルクラウドをパブリッシュした
    }


    particle_cloud_msg_.header.stamp = this->now();
    particle_cloud_msg_.poses.clear();
    for (const auto& particle : particles_)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = particle.x;
        pose.position.y = particle.y;
        pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, particle.yaw));
        particle_cloud_msg_.poses.push_back(pose);
    }
    pub_particle_cloud_->publish(particle_cloud_msg_);
}