#include "obstacle_detector/obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector"){
    // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
    obs_dist_ = this->declare_parameter<double>("obs_dist", 0.7);         // 前方障害物までの距離。これを下回ると障害物として認識する
    laser_num_ = this->declare_parameter<int>("laser_num");
    timer_ = this->create_wall_timer(0.5s , std::bind(&ObstacleDetector::prosess, this)); // プログラムを動かす間隔。0.5s

    // subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(1).reliable(), std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));

    // publisher
    o_points_pub_ = this->create_publisher</* msg型 */>(
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
    std::vector<double> obs_ranges(laser_num_, 0.0);       // 各レーザによる測定値格納用配列
    resize_points();         // 障害物座標格納配列 (points型)。配列サイズを変更する
    for(int i=0; i<laser_num_; i++) obs_ranges[i] = scan_.value().ranges[i];     // ポインタを使えばもっと簡素にできるが要検証

    int central_num = laser_num_ / 2;        // 中心レーザの番号。このレーザの角度 = π/2 rad
    double min_angle_theta = M_PI/2 - central_num * scan_.value().angle_inclement;  // angle_minの角度 [rad]。Roombaの3時方向を0とする。
    // 以上の2つはLiDARの仕様により固定のはずなので、値が確かめられたら数式を消してもよいかもしれない
    printf("OBS_DET test1: min_angle_theta = %5.3f \n", min_angle_theta);

    // 障害物情報配列への格納
    for(int i=0; i<laser_num_; i++){
        if(obs_ranges[i] < obs_dist_ && !is_ignore_scan()){
            // 検知物までの距離がobs_dist_未満で、ignoreしない場合に障害物があるとし、座標を格納
            o_points_[i].exist = true;
            o_points_[i].r = obs_ranges[i];
            o_points_[i].theta = min_angle_theta + scan_.value().angle_inclement * i;
            // 以下2つの直交座標変換は必要ないならコメントアウトする
            o_points_[i].x = o_points_.r * cos(o_points_.theta);
            o_points_[i].y = o_points_.r * sin(o_points_.theta);
        } else {
            o_points_[i] = {false, 0, 0, 0, 0};
        }
    }

    // ROS2メッセージに変換してpublish
    auto msg = sensor_msgs::msg::PointCloud();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "laser_frame";

    for (const auto &p : o_points_) {
        if (p.exist) {
            geometry_msgs::msg::Point32 point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.0;
            msg.points.push_back(point);
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
bool ObstacleDetector::is_ignore_scan(){
    // ignoreする場合 (柱である場合) にtrueを返す
    // 角度(と距離?)でif文
    // double pillar_B[4] = {0.7, 2.3, 3.9, 5.4}    // 各柱の角度領域の下端
    // double pillar_T[4] = {0.8, 2.4, 4.0, 5.6}    // 各柱の角度領域の上端

    // for(int i=0; i<4; i++){
    //     if()
    // }
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