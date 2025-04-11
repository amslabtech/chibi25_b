#include "localizer/localizer.hpp"
#include <tf2/LinearMath/Quaternion.h> // tf2::Quaternion用
#include <tf2/LinearMath/Matrix3x3.h> // getRPY用
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // fromMsg用

// デフォルトコンストラクタ
// パラメータの宣言と取得
// configのyamlファイルから値を取得するため
// ☆これよりデバッグの際に毎回ビルドしなくてもyamlファイルの値を書き換えるだけでよくなる
// Subscriber，Publisherの設定
// frame idの設定
// パーティクルクラウドのメモリの確保
// odometryのモデルの初期化
Localizer::Localizer() : Node("teamb_localizer")
{ 
    // パラメータの宣言
    this->declare_parameter("hz", 10);
    this->declare_parameter("particle_num", 100);
    this->declare_parameter("max_particle_num", 200);
    this->declare_parameter("min_particle_num", 50);
    this->declare_parameter("move_dist_th", 0.1);
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_yaw", 0.0);
    this->declare_parameter("init_x_dev", 1.0);
    this->declare_parameter("init_y_dev", 1.0);
    this->declare_parameter("init_yaw_dev", 0.5);
    this->declare_parameter("alpha_th", 0.01);
    // this->declare_parameter("marginal_likelihood_th_", 0.05);
    this->declare_parameter("reset_count_limit", 5);
    this->declare_parameter("expansion_x_dev", 1.0);
    this->declare_parameter("expansion_y_dev", 1.0);
    this->declare_parameter("expansion_yaw_dev", 0.5);
    this->declare_parameter("laser_step", 5);
    this->declare_parameter("sensor_noise_ratio", 0.1);
    this->declare_parameter("ff_", 0.0);
    this->declare_parameter("fr_", 0.0);
    this->declare_parameter("rf_", 0.0);
    this->declare_parameter("rr_", 0.0);

    // パラメータの取得
    this->get_parameter("hz", hz_);
    this->get_parameter("particle_num", particle_num_);
    this->get_parameter("max_particle_num", max_particle_num_);
    this->get_parameter("min_particle_num", min_particle_num_);
    this->get_parameter("move_dist_th", move_dist_th_);
    this->get_parameter("init_x", init_x_);
    this->get_parameter("init_y", init_y_);
    this->get_parameter("init_yaw", init_yaw_);
    this->get_parameter("init_x_dev", init_x_dev_);
    this->get_parameter("init_y_dev", init_y_dev_);
    this->get_parameter("init_yaw_dev", init_yaw_dev_);
    this->get_parameter("alpha_th", alpha_th_);
    // this->get_parameter("marginal_likelihood_th_", marginal_likelihood_th_);
    this->get_parameter("reset_count_limit", reset_count_limit_);
    this->get_parameter("expansion_x_dev", expansion_x_dev_);
    this->get_parameter("expansion_y_dev", expansion_y_dev_);
    this->get_parameter("expansion_yaw_dev", expansion_yaw_dev_);
    this->get_parameter("laser_step", laser_step_);
    this->get_parameter("sensor_noise_ratio", sensor_noise_ratio_);
    this->get_parameter("ff_", ff_);
    this->get_parameter("fr_", fr_);
    this->get_parameter("rf_", rf_);
    this->get_parameter("rr_", rr_);

    // Subscriberの設定
    // <subscriber名> = this->create_subscription<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable(), std::bind(&<class名>::<コールバック関数名>, this, std::placeholders::_1));
    // std::bindを使ってsubするコールバック関数，std::placeholdersを使ってその関数内での引数を指定する
    // std::placeholdersで指定する引数は大体1番目のもの（コールバック関数の引数が1つであるため）
    // sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable(), std::bind(&Localizer::map_callback, this, std::placeholders::_1));
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), std::bind(&Localizer::map_callback, this, std::placeholders::_1)); // サブスクライバーが処理できなくても1個前のmsgを残し常に一件保持、新しいサブスクライバーが来たら古いmsgも渡し後から来ても直前のを再送してくれる
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
    // OdomModel odom_model_(ff_, fr_, rf_, rr_);
    odom_model_ = {ff_, fr_, rf_, rr_}; // クラスメンバのodom_model_に代入するにはこのOdomModelクラスの代入用コンストラクタを使った

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
    flag_map_ = true; // マップのmsg受け取りフラグ
    printf("%d\n", flag_map_); // 1って出力出た
    // std::cout << std::boolalpha << flag_map_ << std::endl; // trueって出力出た
}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{   
    prev_odom_ = last_odom_; // 1つ前のオドメトリを保存
    last_odom_ = *msg;       // 最新のオドメトリを保存
    flag_odom_ = true; // オドメトリのmsg受け取りフラグ
    // printf("flag_odom_ = %d\n", flag_odom_);

    double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    double distance_moved = std::sqrt(dx * dx + dy * dy); // ロボットが動いた距離
    
    if (distance_moved > 0.01) { // 1cm以上
        flag_move_ = true; // 機体動いた
        // printf("flag_move_ = %d\n", flag_move_);
    }

    // (オドメトリのノイズを考慮した補正)
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_laser_ = true; // レーザーのmsg受け取りフラグ
    // printf("flag_laser_ = %d\n", flag_laser_);
}

// hz_を返す関数
int Localizer::getOdomFreq() // (constを付けることでhz_が外部から書き換えられないようにすることを保証出来る)
{
    // printf("hz_ = %d\n", hz_);
    return hz_;
}

// ロボットとパーティクルの推定位置の初期化
void Localizer::initialize()
{
    // 推定位置の初期化
    estimated_pose_.set(init_x_, init_y_, init_yaw_);

    // estimated_pose_.x_ = init_x_;
    // estimated_pose_.y_ = init_y_;
    // estimated_pose_.yaw_ = init_yaw_;
    // tf2::Quaternion quat(estimated_pose_.x, estimated_pose_.y, estimated_pose.orientation.z, estimated_pose.orientation.w);
    // tf2::Matrix3x3(quat).getEulerYPR(estimate_pose.yaw_, pitch, roll);  // yawはラジアン単位で取得
    
    // パーティクルの初期化
    particles_.clear();

    for (int i = 0; i < particle_num_; i++) {
        // particleのローカル変数を定義して，初期値を初期位置にノイズを加えてparticleをセット
        double particle_x = init_x_ + norm_rv(0, init_x_dev_);
        double particle_y = init_y_ + norm_rv(0, init_y_dev_);
        double particle_yaw = init_yaw_ + norm_rv(0, init_yaw_dev_);
        double particle_weight = 1.0;

        // Particle particle(init_x_ + norm_rv(0, init_x_dev_), init_y_ + norm_rv(0, init_y_dev_), init_yaw_ + norm_rv(0, init_yaw_dev_), particle_weight); // Particleとは一つのパーティクル作成のための構造体(x, y, yaw, weight入り)
        // ↑ではなくparticleのローカル変数自体を定義

        // 角度yawの正規化，メンバ変数のparticles_にpush_back
        normalize_angle(particle_yaw);
        Particle particle(particle_x, particle_y, particle_yaw, particle_weight);
        particles_.push_back(particle);

        // パーティクルの重みの初期化
        reset_weight();
    }
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_) // map,odom,laserの値を取得できたとき
    {
        // // tfのブロードキャスト(他のノードに送信すること)、ros1ならtf::でros2ならtf2_ros::
        // static tf2_ros::TransformBroadcaster br; // tfの座標変換情報をブロードキャストするためのオブジェクト
        // tf2_ros::Transform transform; // Transformとはtfの座標変換を表すクラスでロボットの平行移動を表すtf::Vector3と回転情報を表すtf::Quaternionを持つ
        // std::transform.setOrigin(tf2_ros::Vector3(estimated_pose_.x, estimated_pose_.y, 0.0) ); // 位置を設定
        // tf2_ros::Quaternion q;
        // q.setRPY(0, 0, estimated_pose_.yaw); // 回転情報を入力(Roll, Pitch, Yaw)の順
        // transform.setRotation(q); // 回転を設定
        // br.sendTransform(tf2_ros::StampedTransform(std::transform, ros::Time::now(), "world", "robot")); // (座標系, タイムスタンプとして使う時刻, 親座標系の名前, 今使っている座標系の名前)  

        // 初回のみ TransformBroadcaster を初期化
        if (!flag_broadcast_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        }

        // 変換情報を格納する TransformStamped メッセージ
        geometry_msgs::msg::TransformStamped transform;

        // ここで transform の座標情報を設定
        transform.header.stamp = this->now();
        transform.header.frame_id = "world"; // 親フレーム
        transform.child_frame_id = "robot"; // 子フレーム
        transform.transform.translation.x = estimated_pose_.x(); // カメラの推定位置
        transform.transform.translation.y = estimated_pose_.y();
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, estimated_pose_.yaw()); // 回転情報を入力(Roll, Pitch, Yaw)の順
        q.normalize(); // クォータニオンの正規化
        transform.transform.rotation.x = q.x(); // 回転(単位クォータニオン)
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // 推定位置の座標変換tfをブロードキャスト
        tf_broadcaster_->sendTransform(transform); // ？hppに足したけどtf_broadcaster_の使用先

        // 位置推定
        localize();

        // ロボットの移動後のmap座標系から見たodom座標系のtfをブロードキャスト
        broadcast_odom_state();
        flag_broadcast_ = true; // tfブロードキャストした
        
        // パブリッシュ
        publish_estimated_pose(); // 推定位置
        publish_particles(); // パーティクルクラウド
    }
}

// 適切な角度(-M_PI ~ M_PI)を返す＝正規化
double Localizer::normalize_angle(double angle)
{
    angle = std::atan2(std::sin(angle), std::cos(angle));

    // ↑ whileより計算速い
    // while (angle < -M_PI && M_PI < angle) { // 適切な範囲外のとき
    //     if (angle < -M_PI) {
    //         angle += 2 * M_PI;
    //     } else 
    //         angle -= 2 * M_PI;
    // }

    return angle;
}

// ランダム変数生成関数（正規分布に従った）
double Localizer::norm_rv(const double mean, const double stddev) // 平均と標準偏差
{
    static std::default_random_engine generator; // 乱数生成
    std::normal_distribution<double> distribution(mean, stddev); // 正規分布生成器の初期化

    return distribution(generator);
}

// particles_リストのパーティクルの重みの初期化(正規分布に基づいて初期値設定、正規化→初期化？)
void Localizer::reset_weight()
{
    double another_weight = 1.0 / particle_num_; // 初期重み1.0を均等配分
    // particles_.push_back(another_weight); // hppで定義済みのリストに追加
    for (auto& particle : particles_) {
        particle.set_weight(another_weight);
        // particle.cppの
            // setter
            // void Particle::set_weight(double weight)
            // {
            //     weight_ = weight;
            // }
        // 使用
    }
}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
// map座標系からみたbase_link座標系の位置と姿勢，odom座標系からみたbase_link座標系の位置と姿勢から計算
void Localizer::broadcast_odom_state()
{
    if(flag_odom_ && flag_broadcast_) // tfをブロードキャストしたときも
    {
        // TF Broadcasterの実体化
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        // broadcasterの初期化
        odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系(地図の静的な絶対座標)からみたbase_link座標系(ロボットの中心基準で動く座標) = estimated_pose_ の位置と姿勢の取得
        geometry_msgs::msg::PoseStamped map_to_base;
        map_to_base.header.stamp = this->now(); // 現在時刻をセット
        map_to_base.header.frame_id = "map"; // "map" 座標系
        map_to_base.pose.position.x = estimated_pose_.x(); // 推定位置をコピー、estimated_pose_msgだと一個前のpublishしたもの
        map_to_base.pose.position.y = estimated_pose_.y();
        map_to_base.pose.position.z = 0.0; // Poseはz成分を持っていない

        // odom座標系(ロボットの動きを元にした相対座標)からみたbase_link座標系(ロボットの中心基準で動く座標) = last_odom_ の位置と姿勢の取得
        geometry_msgs::msg::PoseStamped odom_to_base;
        odom_to_base.header.stamp = this->now(); // 現在時刻をセット
        odom_to_base.header.frame_id = "odom"; // "odom" 座標系、？座標系の名前若干統一されてるか不安
        odom_to_base.pose = last_odom_.pose.pose; // 最新のオドメトリをコピー(odom_to_baseは.position.x等、last_odom_も.position.x等と続く)

        // map座標系(地図の静的な絶対座標)からみたodom座標系(ロボットの動きを元にした相対座標)の位置と姿勢を計算（回転行列を使った単純な座標変換）
        // map座標系とodom座標系のx軸は同じではない（ロボットが回転するとodom座標系も回転する）ので，それを考慮して算出する
        // double dx = map_to_base.pose.position.x - odom_to_base.pose.position.x;
        // double dy = map_to_base.pose.position.y - odom_to_base.pose.position.y;
        // double dyaw = map_to_base.pose.orientation.z - odom_to_base.pose.orientation.z;
        // odom→baseの位置ベクトルを取得（ロボット相対位置）
        double ox = odom_to_base.pose.position.x;
        double oy = odom_to_base.pose.position.y;
        // map→baseの位置ベクトルを取得（絶対位置）
        double mx = map_to_base.pose.position.x;
        double my = map_to_base.pose.position.y;
        // base→odomの相対位置をmap座標系に変換（回転を考慮）
        double theta = tf2::getYaw(map_to_base.pose.orientation); // map座標系から見たbaseの向き
        double dx = mx - (ox * cos(theta) - oy * sin(theta));
        double dy = my - (ox * sin(theta) + oy * cos(theta));
        // yaw差分（回転の向き）を取得
        double yaw_odom = tf2::getYaw(odom_to_base.pose.orientation);
        double dyaw = theta - yaw_odom;
        dyaw = normalize_angle(dyaw); // 回転に関しては計算後に適切な範囲に変更

        // (サンプルコードに記述済み)
        tf2::Quaternion map_to_odom_quat;
        map_to_odom_quat.setRPY(0, 0, dyaw);

        // yawからquaternionを作成(サンプルコードに記述済み)
        // tf2::Quaternion map_to_odom_quat; // (？？)

        // odom座標系odomの位置姿勢情報格納するための変数(サンプルコードに記述済み)
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納
        odom_state.header.stamp = this->now();
        RCLCPP_INFO(this->get_logger(), "Current time: %ld", this->now().nanoseconds()); // デバック用

        // 親フレーム・子フレームの指定(サンプルコードに記述済み)
        odom_state.header.frame_id = map_.header.frame_id; // 親フレーム
        odom_state.child_frame_id  = last_odom_.header.frame_id; // 子フレーム

        // map座標系からみたodom座標系の原点位置と方向の格納
        // 格納対象は位置情報（x, y）と回転情報（x, y, z, w）
        odom_state.transform.translation.x = dx;
        odom_state.transform.translation.y = dy;
        // odom_state.transform.translation.z = 0.0;
        // odom_state.transform.rotation = tf2::toMsg(map_to_odom_quat);
        map_to_odom_quat.normalize(); // クォータニオンの正規化
        odom_state.transform.rotation.x = map_to_odom_quat.x(); // 回転(単位クォータニオン)
        odom_state.transform.rotation.y = map_to_odom_quat.y();
        odom_state.transform.rotation.z = map_to_odom_quat.z();
        odom_state.transform.rotation.w = map_to_odom_quat.w();

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
        // double dyaw = last_odom_.pose.pose.orientation.z - prev_odom_.pose.pose.orientation.z;
        // 回転角度yawの差は.pose.pose.orientationの回転方向成分から求める
        // double tf2::getYaw(const A & a) ： Quaternionから直接yawを取得する関数
        // 現在
        double yaw_now = tf2::getYaw(last_odom_.pose.pose.orientation);
        // 前回
        double yaw_prev = tf2::getYaw(prev_odom_.pose.pose.orientation);
        // Yawの差分を計算
        double dyaw = yaw_now - yaw_prev;
        dyaw = normalize_angle(dyaw); // 角度は適切な範囲にする
        
        // // 現在
        // tf2::Quaternion q_now(last_odom_.pose.pose.orientation.x,
        //                     last_odom_.pose.pose.orientation.y,
        //                     last_odom_.pose.pose.orientation.z,
        //                     last_odom_.pose.pose.orientation.w);
        // tf2::getRPY(q_now, std::ignore, std::ignore, yaw_now);
        // // 前回
        // tf2::Quaternion q_prev(prev_odom_.pose.pose.orientation.x,
        //                     prev_odom_.pose.pose.orientation.y,
        //                     prev_odom_.pose.pose.orientation.z,
        //                     prev_odom_.pose.pose.orientation.w);
        // tf2::getRPY(q_prev, std::ignore, std::ignore, yaw_prev);
        // double dyaw = yaw_now - yaw_prev;

        // オドメトリの標準座標
        odom_model_.set_dev(std::sqrt(dx * dx + dy * dy), std::abs(dyaw));

        // ノイズ取得
        double fw_noise = odom_model_.get_fw_noise();
        double rot_noise = odom_model_.get_rot_noise();

        // パーティクルの位置を更新
        for (auto& particle : particles_)
        {
            // パーティクルにノイズを加えて移動、位置更新
            double length = std::sqrt(dx * dx + dy * dy);
            double direction = std::atan2(dy, dx);
            double rotation = dyaw;
            particle.getPose().move(length, direction, rotation, fw_noise, rot_noise); // lengthは直進距離

            // double dx_add_noise = dx + norm_rv(0, fx_noise);
            // double dy_add_noise = dy + norm_rv(0, fx_noise);
            // double dyaw_add_noize = dyaw + norm_rv(0, rot_noise);
            // double ddx,ddy,ddyaw;
            //  ddx += dx_add_noise;
            //  ddy += dy_add_noise;
            //  ddyaw += dyaw_add_noize;

            // // 位置更新
            // particle = Particle(ddx, ddy, ddyaw, particle.weight());
        }
    }
}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    if (flag_map_ && flag_laser_)
    {
        double sum_particle_alpha = 1.0; 
        for(auto& particle : particles_)
        {
            // パーティクル1つの尤度算出
            double yudo = particle.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_); // パーティクル一つの尤度を算出
            double laser_num = ((laser_.angle_max - laser_.angle_min) / laser_.angle_increment) + 1; // パーティクル1つにおけるレーザの本数
            double particle_alpha = yudo / laser_num; // このforで回してるparticleのレーザ1本における平均尤度
            sum_particle_alpha += particle_alpha; // particles_内のparticleのレーザ1本における平均尤度の合計

            // particle.weight() *= yudo; //！重みに尤度をかける
            particle.set_weight(particle.weight() * yudo); // 重みを更新
        }
        // 全体の平均alpha算出
        // double alpha = sum_particle_alpha / particle_num_; // particles_内の1つのparticleのレーザ1本における平均尤度

        // ここからはそれぞれの関数内でforでparticles_回す
        // 重みの正規化は重みに尤度をかけて重みセットした後と、膨張リセットを行なったときにリサンプリングをする前
        normalize_belief();

        // 位置推定
        estimate_pose();

        // 周辺尤度(全パーティクルの重みの合計)の計算
        double sum_yudo_ = calc_marginal_likelihood(); // observation_updateの最初で求めた尤度による重み更新を使ってこの関数で重みの合計を算出

        // ！パーティクル1つのレーザ1本における平均尤度(alpha)を算出
        double alpha = sum_yudo_ / ((laser_.ranges.size()/laser_step_) * particles_.size()); // sum_yudo_を使ってalphaを出す

        // if (alpha < alpha_th_ && marginal_likelihood < marginal_likelihood_th_) // 閾値勝手に増やした
        if (alpha < alpha_th_ && reset_counter < reset_count_limit_) // パーティクルが広がり過ぎないように膨張リセットの回数制限
        {
            // 膨張リセット
            expansion_resetting(); // パーティクルの重みもリセットし散布までしているためこの直後にリサンプリングは不要
            reset_counter++; // リセット回数
        }
        else
        {
            // リサンプリング
            resampling(alpha);
        }
    }
}

// 周辺尤度の算出、観測更新内でParticle::likelihoodより算出された尤度を用いてparticles_のweight()を足し合わせた合計を返す
// EMCLの場合は尤度が小さすぎる場合は膨張リセットを行うため
double Localizer::calc_marginal_likelihood()
{
    // double marginal_likelihood = 0.0; //周辺尤度の平均
    // double yudo_ = 0.0; // likelihood()関数の戻り値として得られる各レーザの尤度
    // double sum_yudo_ = 0.0; //上記の合計
    // int laser_number = 0; // センサの本数

    // for (int i=0; i<laser_.ranges.size(); i+=laser_step_)
    // {
    //     yudo_ = particle.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
    //     sum_yudo_ += yudo_;
    //     laser_number++;
    // }
    
    // marginal_likelihood = sum_yudo_ / laser_number; // 全レーザの合計尤度をレーザで割って平均化
    // return marginal_likelihood;

    // double marginal_likelihood = 0.0;
    double sum_yudo_ = 0.0;

    for (auto& particle : particles_)  // 全パーティクルを走査
    {
        double yudo = particle.weight();
        sum_yudo_ += yudo;
    }

    return sum_yudo_;
    // marginal_likelihood = sum_yudo_ / particles_.size();
    // return marginal_likelihood;
}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
// 加重平均
void Localizer::estimate_pose()
{
    double sum_x = 0.0, sum_y = 0.0, sum_yaw = 0.0;
    double weight_sum = 0.0;
    
    for (auto& particle : particles_)
    {
        sum_x += particle.getPose().x() * particle.weight();
        sum_y += particle.getPose().y() * particle.weight();
        sum_yaw += particle.getPose().yaw() * particle.weight();
        weight_sum += particle.weight();
    }
    // estimated_pose_.x() = sum_x;
    // estimated_pose_.y() = sum_y;
    // estimated_pose_.yaw() = sum_yaw;
    estimated_pose_.set(sum_x, sum_y, sum_yaw);
}

// 重みの正規化(0から1の間、重要度重み、正規化→初期化？)
void Localizer::normalize_belief()
{
    double sum_weights = 0.0;
    for (auto& particle : particles_)
    {
        sum_weights += particle.weight(); // 重みの合計を計算
    }

    if (sum_weights > 0.0)
    {
        for (auto& particle : particles_)
        {
            // particle.weight() /= sum_weights; //重要度重み算出
            particle.set_weight(particle.weight() / sum_weights);
        }
    }
}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting()
{
    // ロボットが動ける範囲
    // double map_min_x = map_.info.origin.position.x; // マップ原点のx座標がロボットの動ける範囲のx座標min.
    // double map_max_x = map_min_x + map_.info.width * map_.info.resolution; // map_.info.widthはマップの幅で単位はセル数、map_.info.resolutionは1セルの大きさ[m]
    // double map_min_y = map_.info.origin.position.y;
    // double map_max_y = map_min_y + map_.info.height * map_.info.resolution;

    // ランダム生成
    std::random_device rd;
    std::mt19937 gen(rd());
    // std::uniform_real_distribution<> dist_x(map_min_x, map_max_x);
    // std::uniform_real_distribution<> dist_y(map_min_y, map_max_y);
    // std::uniform_real_distribution<> dist_yaw(-M_PI, M_PI);
    std::normal_distribution<> dist_x(estimated_pose_.x(), expansion_x_dev_); // 平均になるestimated_pose_と膨張リセットの標準偏差の値を用いて正規分布を生成
    std::normal_distribution<> dist_y(estimated_pose_.y(), expansion_y_dev_);
    std::normal_distribution<> dist_yaw(estimated_pose_.yaw(), expansion_yaw_dev_);

    for (auto& particle : particles_)
    {
        double x = dist_x(gen);
        double y = dist_y(gen);
        double yaw = dist_yaw(gen);
        
        yaw = normalize_angle(yaw); // yawを適切な範囲にする
        particle.getPose().set(x, y, yaw);
        // particle.getPose().set(random_x_in_map(), random_y_in_map(), random_angle()); // これらのランダム配置の関数は定義されていない

        particle.set_weight(1.0);
    }

    normalize_belief();
}

// リサンプリング（系統サンプリング）
// 周辺尤度に応じてパーティクルをリサンプリング
// パーティクルの重みを積み上げたリストを作成
// 1/Nずつリストを選んだパーティクルN個コピー(最初の位置だけ乱数で選ぶ)
// コピーの重みを1/Nにして新たなパーティクルの集合とする
void Localizer::resampling(const double alpha)
{
    // パーティクルの重みを積み上げたリストを作成(リサンプリングのため)
    std::vector<double> cumulative; // 累積重みリスト
    cumulative.reserve(particles_.size());
    double total_weight = 0.0;
    for (const auto& p : particles_) {
        total_weight += p.weight();
        cumulative.push_back(total_weight);
    }

    if (total_weight <= 0.0) {
        std::cerr << "Warning: Total particle weight is zero. Skipping resampling.\n";
        return;
    }

    // サンプリングのスタート位置とステップを設定
    double step = total_weight / particle_num_;
    double start = ((double)rand() / RAND_MAX) * step; // [0, step) のランダム値
    double target = start;

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
    indexes.clear();
    indexes.reserve(particle_num_); // インデックスの数をパーティクルの数に合わせる
    int index = 0;
    for (int i = 0; i < particle_num_; ++i) {
        while (target > cumulative[index] && index < cumulative.size() - 1) {
            index++;
        }
        indexes.push_back(index); // 選ばれたパーティクルのインデックスを記録
        target += step;
    }
    // for (int i=0; i<particle_num_; i++)
    // {
    //     index++;
    //     indexes.push_back(index); // indexをhppで定義済みのリストに追加
    // }

    // リサンプリング
    particles_.clear();
    particles_.reserve(particle_num_); // パーティクル数の保持
    for (int i = 0; i < particle_num_; ++i) {
        Particle sampled = particles_[indexes[i]]; // インデックスに対応する粒子を取得
        sampled.set_weight(1.0 / particle_num_);      // 重みは後で初期化するがここでも仮で設定
        particles_.push_back(sampled);
    }
    // next_particles_.clear();
    // next_particles_.reserve(particle_num_); // パーティクル数の保持
    // for (int i = 0; i < particle_num_; ++i) {
    //     Particle sampled = particles_[indexes[i]]; // インデックスに対応する粒子を取得
    //     sampled.set_weight(1.0 / particle_num_);      // 重みは後で初期化するがここでも仮で設定
    //     next_particles_.push_back(sampled);
    // }

    // hppで定義済みのリストに追加し重みを初期化
    // particles_.insert(next_particles_); // next_particles_をparticles_に代入するがリスト→リストの関数がわからないためnext_particles_の使用をやめた ↑
    reset_weight(); // particles_のparticleの重み初期化をする関数
    // particles_ = std::move(next_particles_); //観測更新はmove使用しない
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{
    // 位置推定結果をパブリッシュする
    estimated_pose_msg_.header.stamp = this->now();
    estimated_pose_msg_.header.frame_id = "world";
    estimated_pose_msg_.pose.position.x = estimated_pose_.x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.y();
    // ！toMsg = tf2::Quaternion → geometry_msgs::msg::Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose_.yaw()); // Roll=0, Pitch=0, Yaw=estimated_pose_.yaw()
    estimated_pose_msg_.pose.orientation = tf2::toMsg(q);
    // estimated_pose_msg_.pose.orientation = tf2::createQuaternionMsgFromYaw(estimated_pose_.yaw());
    
    // <publisher名>->publish(<変数名>);
    pub_estimated_pose_->publish(estimated_pose_msg_);
}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    particle_cloud_msg_.header.stamp = this->now();
    particle_cloud_msg_.poses.clear(); // 前回までのパーティクルデータの削除
    for (auto& particle : particles_)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = particle.getPose().x();
        pose.position.y = particle.getPose().y();
        // pose.position.z = particle.getPose().z();
 
        // ！toMsg = tf2::Quaternion → geometry_msgs::msg::Quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, particle.getPose().yaw()); // roll=0, pitch=0, yaw=pose.yaw()
        pose.orientation = tf2::toMsg(q); // tf2を使ってROSのQuaternionに変換
        // pose.yaw() = tf2::toMsg(tf2::Quaternion(0, 0, particle.getPose().yaw()));

        particle_cloud_msg_.poses.push_back(pose); // hppで定義済みのリストに追加、particle_cloud_msg_の型より.poses.push_back(geometry_msgs::msg::～)にする
    }

    // <publisher名>->publish(<変数名>);
    pub_particle_cloud_->publish(particle_cloud_msg_);
    is_visible_ = true; // パーティクルクラウドをパブリッシュした

}

    // for (auto& particle : particles_) {
    //     // particles_の中のparticle一つごとにfor文回せる
    // }

    // OdomModel(const double ff, const double fr, const double rf, const double rr);  // コンストラクタ
    // を使って初期化
    // - OdomModel odom_model_(0.0, 0.0, 0.0, 0.0);

    // Pose 変数名;はPose.cppに、Particle 変数名;はparticle.cppを見て内部の構造知る

        // poseのは変数名.set(x, y, yaw);

            // Particle
            // accessor
        // void set_weight(const double weight);   // 重みのセット、Particleの変数名.set_weight(引数)で使える
        // double weight() const { return weight_; } // 値を返すだけの関数(右辺)だから、重みの値を変え直接代入するとき(*=)はset_weight()を使うか、double& にして参照を返すと.weight()を直接変更できる
        // Particleのクラスの中にはPose pose_;の変数、getPose()があるから、pose_を介して座標にアクセス→particle.getPose().x()等