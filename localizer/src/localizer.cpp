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
    
}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ = *msg;
}

// hz_を返す関数
int Localizer::getOdomFreq()
{

}

// ロボットとパーティクルの推定位置の初期化
void Localizer::initialize()
{
    // 推定位置の初期化

    
    Particle particle;

    // 初期位置近傍にパーティクルを配置

    // パーティクルの重みの初期化
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_)
    {

    }

}

// 適切な角度(-M_PI ~ M_PI)を返す
double Localizer::normalize_angle(double angle)
{

}

// ランダム変数生成関数（正規分布）
double Localizer::norm_rv(const double mean, const double stddev)
{
    
}

// パーティクルの重みの初期化
void Localizer::reset_weight()
{

}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
// map座標系からみたbase_link座標系の位置と姿勢，odom座標系からみたbase_link座標系の位置と姿勢から計算
void Localizer::broadcast_odom_state()
{
    if(flag_broadcast_)
    {
        // TF Broadcasterの実体化
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        // broadcasterの初期化
        odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系からみたbase_link座標系の位置と姿勢の取得

        // odom座標系からみたbase_link座標系の位置と姿勢の取得

        // map座標系からみたodom座標系の位置と姿勢を計算（回転行列を使った単純な座標変換）

        // yawからquaternionを作成
        tf2::Quaternion map_to_odom_quat;

        // odom座標系odomの位置姿勢情報格納するための変数
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納

        // 親フレーム・子フレームの指定
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

        // map座標系からみたodom座標系の原点位置と方向の格納

        // tf情報をbroadcast(座標系の設定)
    }

}

// 自己位置推定
// 動作更新と観測更新を行う
void Localizer::localize()
{

}

// 動作更新
// ロボットの微小移動量を計算し，パーティクルの位置をノイズを加えて更新
void Localizer::motion_update()
{

}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    // パーティクル1つのレーザ1本における平均尤度を算出
    const double alpha;
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{

}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
void Localizer::estimate_pose()
{
    
}

// 重みの正規化
void Localizer::normalize_belief()
{

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
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{

}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    if(is_visible_)
    {
        
    }
}