/*
DWA: Dynamic Window Approach

速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner/local_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    this -> declare_parameter("hz",10);
    this->declare_parameter("robot_frame", "base_link");
//parameterは後から数値変更
    this->declare_parameter("max_vel1", 0.5);
    this->declare_parameter("max_vel2", 0.3);
    this->declare_parameter("max_yawrate1", 1.0);
    this->declare_parameter("max_yawrate2", 0.6);
    this->declare_parameter("weight_dist1_", 0.6);
    this->declare_parameter("weight_dist2_", 0.3);
    this->declare_parameter("weight_heading1_", 0.6);
    this->declare_parameter("weight_heading2_", 0.3);
    this->declare_parameter("radius_margin_1",0.5);
    this->declare_parameter("radius_margin_2",1.0);

    this->declare_parameter("max_accel_",2.0);
    this->declare_parameter("max_dyawrate_",2.0);

    this->declare_parameter("turn_there_yawrate_", 0.6);
    this->declare_parameter("avoid_thres_vel_", 0.6);

    this->declare_parameter("dt_", 0.1);
    this->declare_parameter("goal_tolerance_",0.5);
    this->declare_parameter("predict_time_",2.0);
    this->declare_parameter("vel_reso_",0.1);
    this->declare_parameter("yawrate_reso_",0.1);



    


    // ###### パラメータの取得 ######

    this -> get_parameter("hz",hz_);
    this->get_parameter("robot_frame", robot_frame_);

    max_vel1_ = this->get_parameter("max_vel1").as_double();
    max_vel2_ = this->get_parameter("max_vel2").as_double();
    max_yawrate1_ = this->get_parameter("max_yawrate1").as_double();
    max_yawrate2_ = this->get_parameter("max_yawrate2").as_double();
    weight_dist1_ = this->get_parameter("weight_dist1_").as_double();
    weight_dist2_ = this->get_parameter("weight_dist2_").as_double();
    weight_heading1_ = this->get_parameter("weight_heading1_").as_double();
    weight_heading2_ = this->get_parameter("weight_heading2_").as_double();
    radius_margin1_ = this->get_parameter("radius_margin1_").as_double();
    radius_margin2_ = this->get_parameter("radius_margin2_").as_double();
    max_accel_ = this->get_parameter("max_accel_").as_double();
    max_dyawrate_ = this->get_parameter("max_dyawrate_").as_double();

    turn_there_yawrate_ = this->get_parameter("turn_there_yawrate_").as_double();
    avoid_thres_vel_ = this->get_parameter("avoid_thres_vel_").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance_").as_double();
    predict_time_ = this->get_parameter("predict_time_").as_double();
    vel_reso_ = this->get_parameter("vel_reso_").as_double();
    yawrate_reso_ = this->get_parameter("yawrate_reso_").as_double();

    dt_ = this->get_parameter("dt_").as_double();
    mode_ = 1;
    max_vel_ = max_vel1_;
    max_yawrate_ = max_yawrate1_;
    radius_margin_ = radius_margin1_; 
    weight_heading_ = weight_heading1_;
    weight_dist_ = weight_dist1_;


    // ###### tf_buffer_とtf_listenerを初期化 ######
    // tfの初期化
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // ####### Subscriber #######
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/roomba/goal", rclcpp::QoS(1).reliable(),std::bind(&DWAPlanner::local_goal_callback(),this,std::placeholders::_1));
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/e_pose", rclcpp::QoS(1).reliable(),std::bind(&DWAPlanner::obs_poses_callback(),this,std::placeholders::_1));

    // ###### Publisher ######
    
    pub_cmd_speed_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control",rclcpp::QoS(1).reliable());//#########
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/roomba/optimal_path",rclcpp::QoS(1).reliable());//#########
    pub_predict_path_ =this->create_publisher<nav_msgs::msg::Path>("/roomba/predict_path",rclcpp::QoS(1).reliable());//#########
}

// local_goalのコールバック関数
// local_goalはマップ座標系(map)だが，実際の移動に合わせるためにルンバ座標系(base_link)に変換する処理を行う
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    local_goal_ = *msg;

    printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        // tf を取得（source_frame -> target_frame）
        transform = tf_buffer_->lookupTransform(
            robot_frame_,  // "base_link" などロボット座標系
            local_goal_.header.frame_id,  // 変換元のフレーム
            rclcpp::Time(0),  // 最新の tf を取得
            rclcpp::Duration::from_seconds(0.5) // タイムアウト
        );

        // tf の取得に成功したらフラグを立てる
        flag_local_goal_ = true;
    }
    catch (tf2::TransformException& ex)
    {
        // 取得に失敗した場合、警告を出力しフラグをリセット
        RCLCPP_WARN(this->get_logger(), "Could not transform local goal: %s", ex.what());
        flag_local_goal_ = false;
        return;
    }
    // 取得した変換を用いて local_goal_ をロボット座標系 (base_link) に変換
    geometry_msgs::msg::PointStamped local_goal_transformed;
    tf2::doTransform(local_goal_, local_goal_transformed, transform);

    // 変換結果を更新
    local_goal_ = local_goal_transformed;

}

// obs_posesのコールバック関数
void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
}

// hzを返す関数
int DWAPlanner::get_freq()
{
    return hz_;
}

// 唯一，main文で実行する関数
// can_move()がTrueのとき，calc_final_input()を実行し，速度と旋回速度を計算
// それ以外の場合は，速度と旋回速度を0にする
void DWAPlanner::process()
{
    if (!can_move()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping the robot.");
        roomba_control(0.0, 0.0); // 停止指令を送る
        return;
    }

    // DWA による最適な速度を計算
    std::vector<double> control_input = calc_final_input();

    // ロボットを制御
    roomba_control(control_input[0], control_input[1]);
}


// ゴールに着くまでTrueを返す
bool DWAPlanner::can_move()
{
    // ゴール位置（local_goal_）と現在位置（roomba_）のユークリッド距離を計算
    double dx = local_goal_.point.x - roomba_.x;
    double dy = local_goal_.point.y - roomba_.y;
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);

    // ゴール許容範囲内に到達したら false を返す（移動停止）
    if (distance_to_goal < goal_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return false;
    }

    // まだゴールに到達していない場合は true を返す（移動継続）
    return true;
}


// Roombaの制御入力を行う
void DWAPlanner::roomba_control(const double velocity, const double yawrate)
{
    cmd_speed_.velocity = velocity;  // 並進速度をセット
    cmd_speed_.yawrate  = yawrate;   // 旋回速度をセット

    pub_cmd_speed_ -> publish(cmd_speed_);
}

// 最適な制御入力を計算
std::vector<double> DWAPlanner::calc_final_input()
{
    // 変数を定義，初期化
    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 旋回状況に応じた減速機能
    change_mode();
    mode_ = mode_log_.back();

    // ダイナミックウィンドウを計算
    calc_dynamic_window();

    int i = 0; // 現在の軌跡のインデックス保持用

    // ###### 並進速度と旋回速度のすべての組み合わせを評価 ######
    for (double v = dw_.min_vel; v <= dw_.max_vel; v += vel_reso_)
    {
        for (double w = dw_.min_yawrate; w <= dw_.max_yawrate; w += yawrate_reso_)
        {
            // 軌跡を計算
            std::vector<State> traj = calc_traj(v, w);
            trajectories.push_back(traj);

            // 各評価関数を計算
            double total_score = calc_evaluation(traj);

            // 最適な入力を更新
            if (total_score > max_score)
            {
                max_score = total_score;
                input = {v, w};
                index_of_max_score = i;
            }

            i++; // インデックスを更新
        }
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // ###### pathの可視化 #######
    visualize_traj(trajectories[index_of_max_score], pub_local_path_, this->now());

    //roomba_control(roomba_.velocity,roomba_.yawrate); 

    return input;
}


// 旋回状況に応じた減速機能
// ロボットの旋回速度や速度によって減速モードに切り替える（普段よりも遅く動く）
void DWAPlanner::change_mode()
{
    // 旋回速度が閾値以上 または 並進速度が閾値以下なら減速モード
    if (std::fabs(roomba_.yawrate) > turn_thres_yawrate_ || roomba_.velocity < avoid_thres_vel_)
    {
        mode_ = 2;
        max_vel_ = max_vel2_;            // 減速時の最大並進速度
        max_yawrate_ = max_yawrate2_;    // 減速時の最大旋回速度
        radius_margin_ = radius_margin2_; // 障害物回避を強化
        weight_heading_ = weight_heading2_;
        weight_dist_ = weight_dist2_;
        mode_log_.push_back(2);//減速モード
    }
    else
    {
        mode_ = 1;
        max_vel_ = max_vel1_;            // 通常時の最大並進速度
        max_yawrate_ = max_yawrate1_;    // 通常時の最大旋回速度
        radius_margin_ = radius_margin1_; // 通常の余裕
        weight_heading_ = weight_heading1_;
        weight_dist_ = weight_dist1_;
        mode_log_.push_back(1);//通常モード
    }
}


void DWAPlanner::calc_dynamic_window()
{
    // 現在の速度と旋回速度
    double vel = roomba_.velocity;
    double yawrate = roomba_.yawrate;

    // ###### 車両モデルによるWindow ######
    // 設計上の最大速度・最小速度の制約
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // ####### 運動モデルによるWindow #######
    // ロボットの加減速によって実現可能な速度範囲
    double Vd[] = {
        std::max(min_vel_, vel - max_accel_ * dt_), // 最小速度
        std::min(max_vel_, vel + max_accel_ * dt_), // 最大速度
        std::max(-max_yawrate_, yawrate - max_dyawrate_ * dt_), // 最小旋回速度
        std::min(max_yawrate_, yawrate + max_dyawrate_ * dt_)  // 最大旋回速度
    };

    // ###### 最終的なDynamic Window ######
    dw_.min_vel = std::max(Vs[0], Vd[0]);
    dw_.max_vel = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);

    
}


// 指定された予測時間までロボットの状態を更新し，予測軌跡を生成
std::vector<State> DWAPlanner::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> traj;  // 軌跡を保存するリスト
    State state = roomba_;    // 現在のロボットの状態をコピー

    double time = 0.0;  // 経過時間をカウント
    while (time <= predict_time_) 
    {
        traj.push_back(state);  // 現在の状態を軌跡に追加
        move(state, velocity, yawrate);  // 1ステップ移動
        time += dt_;  // 時間を進める
    }
    return traj;
}

// 予測軌跡作成時における仮想ロボットを移動
void DWAPlanner::move(State& state, const double velocity, const double yawrate)
{
    state.x += velocity * cos(state.yaw) * dt_;  // x座標の更新
    state.y += velocity * sin(state.yaw) * dt_;  // y座標の更新
    state.yaw += yawrate * dt_;                  // 角度の更新
    state.velocity = velocity;                   // 速度の更新
    state.yawrate = yawrate;                     // 旋回速度の更新
}


// angleを適切な角度(-M_PI ~ M_PI)の範囲にして返す
double DWAPlanner::normalize_angle(double angle)
{
    while (angle < -M_PI && M_PI < angle) { // 適切な範囲外のとき
        if (angle < -M_PI) {
            angle += 2 * M_PI;
        } else 
            angle -= 2 * M_PI;
    }
    return angle;
}

// 評価関数を計算
double DWAPlanner::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = weight_heading_ * calc_heading_eval(traj);
    const double distance_score = weight_dist_    * calc_dist_eval(traj);
    const double velocity_score = weight_vel_     * calc_vel_eval(traj);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
// 軌跡のゴール方向への向きやすさを評価する関数

double DWAPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    if (traj.empty()) return 0.0; // 軌跡がない場合は評価値0

    // 軌跡の最後の状態（予測時間後のロボットの位置）
    State last_state = traj.back();  

    // 目標地点との角度（ロボットの進行方向と目標方向のずれ）
    double goal_dx = local_goal_.point.x - last_state.x;
    double goal_dy = local_goal_.point.y - last_state.y;
    double goal_angle = atan2(goal_dy, goal_dx); // 目標までの角度

    // 進行方向との誤差（どれだけ目標に向いているか）
    double angle_diff = normalize_angle(goal_angle - last_state.yaw);

    // 評価値：角度誤差が小さいほどスコアが高い
    return cos(angle_diff); // 1 に近いほど目標方向を向いている
}


// distの評価関数を計算
// 軌跡の障害物回避性能を評価する関数

double DWAPlanner::calc_dist_eval(const std::vector<State>& traj)
{
    if (traj.empty()) return 0.0; // 軌跡がない場合は評価値0

    double min_dist = std::numeric_limits<double>::max(); // 最小距離を大きな値で初期化

    // すべての障害物に対して、軌跡上の最も近い点の距離を計算
    for (const auto& state : traj)
    {
        for (const auto& obs_pose : obs_poses_.poses)//autiの方がより効率的かも
        {
            double dx = state.x - obs_pose.position.x;
            double dy = state.y - obs_pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy); // ユークリッド距離

            if (dist < min_dist)
            {
                min_dist = dist; // 最小距離を更新
            }
        }
    }

    // 障害物に近すぎると評価を低くする（安全性を確保）
    return min_dist; // 距離が大きいほど評価が高い
}


// velocityの評価関数を計算
// 軌跡の速度評価を計算する関数

double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    if (traj.empty()) return 0.0; // 軌跡がない場合は評価値0

    // 軌跡の最後の速度を評価値とする
    double final_velocity = traj.back().velocity;

    // 速度が高いほど評価が高いようにする
    return final_velocity / max_vel_;
}

// 軌跡を可視化するための関数

void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now)
{
    if (traj.empty()) return; // 軌跡が空なら何もしない

    // nav_msgs::msg::Path メッセージを作成
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;          // 現在時刻を設定
    path_msg.header.frame_id = robot_frame_; // ロボットの座標系 (例: "base_link")

    // 軌跡の各状態を geometry_msgs::msg::PoseStamped に変換
    for (const auto& state : traj)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = now;
        pose_stamped.header.frame_id = robot_frame_;
        pose_stamped.pose.position.x = state.x;
        pose_stamped.pose.position.y = state.y;
        pose_stamped.pose.position.z = 0.0;

        // 方向 (yaw) をクォータニオンに変換
        tf2::Quaternion q;
        q.setRPY(0, 0, state.yaw);
        pose_stamped.pose.orientation = tf2::toMsg(q);

        // Path メッセージに追加
        path_msg.poses.push_back(pose_stamped);
    }

    // 可視化のために Publish
    pub_predict_path->publish(path_msg);
}
