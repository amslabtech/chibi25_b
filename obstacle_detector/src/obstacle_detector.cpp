#include "obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector"){
    // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
    obs_dist_ = this->declare_parameter<double>("obs_dist", 1.5);         // 前方障害物までの距離。これを下回ると障害物として認識する
    laser_num_ = this->declare_parameter<int>("laser_num");
    ignore_dist_ = this->declare_parameter<double>("ignore_dist", 0.2);     // これを下回った場合は無視する
    timer_ = this->create_wall_timer(0.5s, std::bind(&ObstacleDetector::process, this)); // プログラムを動かす間隔。0.5s


    // subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(1).reliable(), std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));

    // publisher
    o_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/o_points", rclcpp::QoS(1).reliable());

}

//Lidarから障害物の情報を取得
void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    scan_ = *msg;   
}

//一定周期で行う処理(obstacle_detectorの処理)
void ObstacleDetector::process(){
    scan_obstacle();    
}

//Lidarから障害物情報を取得し，障害物の座標をpublish  ※msg型は自分で決めてください
void ObstacleDetector::scan_obstacle(){
    laser_num_ = (scan_.value().angle_max - scan_.value().angle_min) / scan_.value().angle_increment;    // 測距レーザの本数
    int central_num = laser_num_ / 2;        // 中心レーザの番号。このレーザの角度 = π/2 rad
    double min_angle = M_PI/2 - central_num * scan_.value().angle_increment;  // angle_minの角度 [rad]。Roombaの3時方向を0とする。
        // min_angleとcentral_numはLiDARの仕様により固定のはずなので、値が確かめられたら数式を消してもよい

    std::vector<double> obs_ranges(laser_num_, 0.0);       // 各レーザによる測定値格納用配列
    std::vector<double> obs_angles(laser_num_, 0.0);       // 角度用配列
    resize_points();         // 障害物座標格納配列 (points型)。配列サイズを変更する
    for(int i=0; i<laser_num_; i++) obs_ranges[i] = scan_.value().ranges[i];     // ポインタを使えばもっと簡素にできるが要検証
    for(int i=0; i<laser_num_; i++) obs_angles[i] = scan_.value().angle_increment * i + min_angle;  

    printf("OBS_DET test1: min_angle = %5.3f \n", min_angle);

    // 障害物情報配列への格納
    for(int i=0; i<laser_num_; i++){
        if(obs_ranges[i] < ignore_dist_) {
            // 検知物までの距離がignore_dist_未満の場合は障害物でないとする
            o_points_[i] = {false, 0, 0, 0, 0};

        } else if(obs_ranges[i] < obs_dist_ && !is_ignore_scan(obs_angles[i])) {
            // 検知物までの距離がobs_dist_未満で、ignoreしない場合に障害物があるとし、座標を格納
            o_points_[i].exist = true;
            o_points_[i].r = obs_ranges[i];
            o_points_[i].theta = obs_angles[i];
            // 以下2つの直交座標変換は必要ないならコメントアウトする
            o_points_[i].x = o_points_[i].r * cos(o_points_[i].theta);
            o_points_[i].y = o_points_[i].r * sin(o_points_[i].theta);
            
        } else {
            o_points_[i] = {false, 0, 0, 0, 0};
        }
    }

    // ROS2メッセージに変換してpublish
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "laser_frame";        // laser_frame 座標系に基づいた表記であることの明示
    
    for (const auto &p : o_points_) {
        if (p.exist) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;
            // orientation は使わないのでデフォルト（0）でも良い
            msg.poses.push_back(pose);
        }
    }
    o_points_pub_->publish(msg);
    
    points_print();     // デバッグ用


    /*
    障害物の座標を格納するmsgについて
    構造体 o_points_ (points型)の宣言：
        struct points{
            bool exist; // trueなら障害物あり
            double x;
            double y;
            double r;
            double theta;
        };    
    */
}


//無視するlidar情報の範囲の決定(lidarがroombaの櫓の中にあり，櫓の４つの柱を障害物として検出してしまうため削除が必要)
bool ObstacleDetector::is_ignore_scan(double angle){
    // ignoreする場合 (柱である場合) にtrueを返す
    // 柱の角度はExcelで調べました。データはGoogleドライブ参照
    double pillar_B[4] = {-0.80, 0.63, 2.24, 3.75};    // 各柱の角度領域の下端。[rad]
    double pillar_T[4] = {-0.62, 1.20, 2.74, 3.93};    // 各柱の角度領域の上端。[rad]

    for(int i=0; i<4; i++) if(pillar_B[i] < angle && angle < pillar_T[i])  return true;
    return false;
}

// 可変長配列 o_points の長さ決定
void ObstacleDetector::resize_points(){
    o_points_.resize(laser_num_, {0, 0.0});
}

// 障害物情報のprintf
void ObstacleDetector::points_print(){
    for(int i=0; i<laser_num_; i++){
        if(o_points_[i].exist){
            printf("\t\tOBS_DET test2: i = %d, r = %5.3f, θ = %5.3f[deg]\n", i, o_points_[i].r, o_points_[i].theta * 180/M_PI);
        }
    }
}