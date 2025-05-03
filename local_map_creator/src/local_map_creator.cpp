#include "local_map_creator/local_map_creator.hpp"

using namespace std::chrono_literals;

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("teamb_local_map_creater"){
    // global変数の定義
    this->declare_parameter<double>("hz", 4.0);
    this->declare_parameter<double>("map_size", 4.0);
    this->declare_parameter<double>("map_reso", 0.025);

    // パラメータの取得(hz, map_size, map_reso)。今回はyamlファイルを使っていないため、本来は必要ない。
    this->get_parameter("hz", hz_);
    this->get_parameter("map_size", map_size_);
    this->get_parameter("map_reso", map_reso_);
    
    // プログラムを動かす間隔。1/Hz
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / hz_), std::bind(&LocalMapCreator::process, this));

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
    local_map_.header.frame_id = "base_link";

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

    // data:実際のマップデータ (local_map_.data[]の0: 空き, 100: 障害物, -1: 未知)
    local_map_.data.resize(local_map_.info.width * local_map_.info.height, -1); // ☆マップの初期化とは違いマップのサイズを確保し、全て未知-1に
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    obs_poses_ = *msg;
    flag_obs_poses_ = true;     // msgを取得した場合に true を返すことで判定できるようにする
    // RCLCPP_INFO(this->get_logger(), "LMC: got msg ----------------------");
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
        RCLCPP_INFO(this->get_logger(), "LMC: Map updated.");
    } else {
        // obs_poses_がmsgを受け取っていない場合はエラー
        RCLCPP_WARN(this->get_logger(), "ERROR LMC: No obs data received yet.");
        return;
    }
}

// 障害物の情報をもとにローカルマップをrenewal
void LocalMapCreator::update_map(){
    init_map();         // マップのinitialize

    // 軽量化のため、ここでまとめてdeclarement
    double obs_x = 0.0;
    double obs_y = 0.0;
    double obs_dist  = 0.0;
    double obs_angle = 0.0;
    int index = 0;

    // 障害物情報を1度受け取るごとに、msg内の全ての障害物情報をマップにreflect
    for (const auto &obs_pose : obs_poses_.poses){
        // 極座標と直交座標へstorage
        obs_x = obs_pose.position.x;
        obs_y = obs_pose.position.y;
        obs_dist  = hypot(obs_x, obs_y);
        obs_angle = atan2(obs_y, obs_x);
        printf("x: %5.2f, y: %5.2f, dist: %5.2f, angle: %5.2f\n", obs_x, obs_y, obs_dist, obs_angle);

        // (1度に受け取った障害物点群情報に対し) マップの各グリッド (ピクセル) に対する「空き」か「障害物」のjudgement
        for(double target_dist = 0.0; (target_dist<obs_dist) && is_in_map(target_dist, obs_angle); target_dist += map_reso_){
            index = polar_to_grid_index(target_dist, obs_angle);
            local_map_.data[index] = 0;             // (マップ内外に関わらず) 障害物がある方向全てに "empty"
            
            if(is_in_map(obs_dist, obs_angle)) {
                index = xy_to_grid_index(obs_x, obs_y);
                local_map_.data[index] = 100;       // 障害物があるグリッドで、かつマップ内なら "obstacle"
            }
        }
    }
    local_map_.header.stamp = this->now();  // 不必要？

    // 更新したマップをpublish
    pub_local_map_ -> publish(local_map_);
    RCLCPP_INFO(this->get_logger(), "LMC: published.");
}

// マップの初期化 (すべてが「未知」になる)
void LocalMapCreator::init_map(){
    std::fill(local_map_.data.begin(), local_map_.data.end(), -1); // ☆サイズは決まっているから新しく確保せずに内容だけをリセットする
    // fill: local_map_.data[] の local_map_.data.begin() から local_map_.data.end() までの要素をすべて -1 で埋める
}

// 指定された距離と角度がマップの範囲内か判定する
bool LocalMapCreator::is_in_map(const double dist, const double angle){
    double x = dist * cos(angle);
    double y = dist * sin(angle);
    if (x >= -map_size_/2.0 && x <= map_size_/2.0 && y >= -map_size_/2.0 && y <= map_size_/2.0) return true;  // マップ内部ならtrue
    else return false;
}

// 極座標からグリッドのインデックスを返す
int LocalMapCreator::polar_to_grid_index(const double dist, const double angle){
    if(is_in_map(dist, angle)) {
        // double x = dist * cos(angle);
        // double y = dist * sin(angle);
        return xy_to_grid_index(dist * cos(angle), dist * sin(angle));  // (x座標, y座標)
    } else {
        return -1;
    }
}

// 直交座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y){
    if(is_in_map(hypot(x, y), atan2(y, x))){
        int grid_x = static_cast<int>((x + map_size_ / 2.0) / map_reso_);
        int grid_y = static_cast<int>((y + map_size_ / 2.0) / map_reso_);
        return grid_y * local_map_.info.width + grid_x;
    } else {
        return -1;
    }
}