<<<<<<< HEAD
#include "local_map_creator/local_map_creator.hpp"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creater"){
    // パラメータの取得(hz, map_size, map_reso)
    this->get_parameter("hz", hz_);
    this->get_parameter("map_size", map_size_);
    this->get_parameter("map_reso", map_reso_);

    // Subscriberの設定
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/o_points", rclcpp::QoS(1).reliable(), std::bind(&LocalMapCreator::obs_poses_callback, this, std::placeholders::_1));

    // Publisherの設定
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/l_map", rclcpp::QoS(1).reliable());

    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header:フレームIDとタイムスタンプ
    local_map_.header.stamp = this -> now();
    local_map_.header.frame_id = "map";

    //   info(width, height, position.x, position.y):マップの解析度、サイズ、原点位置など
    local_map_.info.width = static_cast<int>(map_size_ / map_reso_); // グリッド数で表す
    local_map_.info.resolution = map_reso_;
    local_map_.info.height = static_cast<int>(map_size_ / map_reso_); // グリッド数で表す
    local_map_.info.origin.position.x = -map_size_ / 2.0; // .info.origin.positionはローカルマップの原点でマップの左下の座標を表す
    local_map_.info.origin.position.y = -map_size_ / 2.0;
    local_map_.info.origin.position.z = 0.0;
    local_map_.info.origin.orientation.x = 0.0;
    local_map_.info.origin.orientation.y = 0.0;
    local_map_.info.origin.orientation.z = 0.0;
    local_map_.info.origin.orientation.w = 1.0;

    //   data:実際のマップデータ(local_map_.data[]の0:空き, 100:障害物, -1:未知)
    local_map_.data.resize(local_map_.info.width * local_map_.info.height, -1); // ☆マップの初期化とは違いマップのサイズを確保し、全て未知-1に
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    obs_poses_ = *msg;
    flag_obs_poses_ = true;     // msgを取得した場合に true を返すことで判定できるようにする
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq(){
    return hz_; // ループ周波数の数値を返す
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process(){
    if (flag_obs_poses_){
        update_map();
        flag_obs_poses_ = false; // またmsgを取得したときコールバックでtrue判定できるようにfalseに戻しておく
    }
}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map(){
    init_map();         // マップを初期化

    // 障害物の位置を考慮してマップを更新する
    for (const auto &pose : obs_poses_.poses){
        // 障害物の位置情報
        int index = xy_to_grid_index(pose.position.x, pose.position.y); // (x, y)座標をグリッド数で示す
        if (index >= 0 && index < static_cast<int>(local_map_.data.size())) {
            local_map_.data[index] = 100; // 障害物を示す値
        }
    }
    local_map_.header.stamp = this->now();
    // 更新したマップをpublishする
    pub_local_map_ -> publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map(){
    std::fill(local_map_.data.begin(), local_map_.data.end(), -1); // ☆サイズは決まっているから新しく確保せずに内容だけをリセットする
    // fill: local_map_.data[] の local_map_.data.begin() から local_map_.data.end() までの要素をすべて -1 で埋める
}

// 指定された距離と角度がマップの範囲内か判定する
bool LocalMapCreator::in_map(const double dist, const double angle){
    double x = dist * cos(angle);
    double y = dist * sin(angle);
    if (x >= -map_size_/2.0 && x <= map_size_/2.0 && y >= -map_size_/2.0 && y <= map_size_/2.0) return true;  // マップ内部ならtrue
    else return false;
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle){
    if (in_map(dist, angle)) {
        double x = dist * cos(angle);
        double y = dist * sin(angle);
        return xy_to_grid_index(x, y);
    } else {
        return -1;
    }
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y){
    int grid_x = static_cast<int>((x + map_size_ / 2.0) / map_reso_);
    int grid_y = static_cast<int>((y + map_size_ / 2.0) / map_reso_);
    return grid_y * local_map_.info.width + grid_x;
}
=======
#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/e_pose", rclcpp::QoS(1).reliable(),std::bind(&LocalGoalCreator::poseCallback,this,std::placeholders::_1));
    //estimated poseの購読
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/g_path", rclcpp::QoS(1).reliable(),std::bind(&LocalGoalCreator::pathCallback,this,std::placeholders::_1));
    //global pathの購読
    //pubやsubの定義，tfの統合
    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/roomba/goal",rclcpp::QoS(1).reliable());//#########
    //初期値の設定
    hz_= this->declare_parameter<int>("hz", 100);// ループ周期 [Hz]
    goal_index_ = this->declare_parameter<int>("goal", 1);// グローバルパス内におけるローカルゴールのインデックス
    index_step_ = this->declare_parameter<int>("index", 1); // １回で更新するインデックス数
    target_distance_ = this->declare_parameter<float>("distance", 1.0);// 現在位置-ゴール間の距離 [m]
    //初期設定
    geometry_msgs::msg::PointStamped path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_ =*msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_=*msg;
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
    if (!path_.poses.empty()) {
        publishGoal();
    }
    

}


void LocalGoalCreator::publishGoal()
{
    if (goal_index_ >= static_cast<int>(path_.poses.size())) {
        return; // すでに最後のゴールなら更新しない
    }

    // ゴールまでの距離を取得
    target_distance_ = getDistance();
    // double threshold_distance_ = 1.0;
    
    // ゴールに近づいたら次のゴールを選択
    if (target_distance_ < threshold_distance_) {
        goal_index_ += index_step_;

        // インデックスがパスの範囲を超えたら最後のゴールに留まる
        if (goal_index_ >= static_cast<int>(path_.poses.size())) {
            goal_index_ = static_cast<int>(path_.poses.size()) - 1;
        }
    }

    if (goal_index_ < static_cast<int>(path_.poses.size())) {
        
        path_msg.point = path_.poses[goal_index_].pose.position;

        local_goal_pub_->publish(path_msg);
    }
}


double LocalGoalCreator::getDistance()
{
    //if (path_.poses.empty()) return std::numeric_limits<double>::max(); // パスが空なら最大値を返す

    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    return sqrt(dx * dx + dy * dy);
}
>>>>>>> 8edb808d1a695da6c1221b3ae8bf54da72834e3e
