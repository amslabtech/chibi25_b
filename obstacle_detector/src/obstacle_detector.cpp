#include "obstacle_detector/obstacle_detector.hpp"

ObstacleDetector::ObstacleDetector() : Node("teamb_obstacle_detector"){
    // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
    obs_dist_ = this->declare_parameter<double>("obs_dist", 10);         // 前方障害物までの距離。これを下回ると障害物として認識する
    laser_num_ = this->declare_parameter<int>("laser_num", 1);              // とりあえず1を代入。33行目で正しく計算される。
    ignore_dist_ = this->declare_parameter<double>("ignore_dist", 0.5);     // これを下回った場合は無視する
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
    if(!scan_.has_value()){
        // scan_がmsgを受け取っていない場合はエラー
        RCLCPP_WARN(this->get_logger(), "ERROR Obstacle_detector: No scan data received yet.");
        return;
    }
    scan_obstacle();    
}

//Lidarから障害物情報を取得し，障害物の座標をpublish  ※msg型は自分で決めてください
void ObstacleDetector::scan_obstacle(){
    laser_num_ = (scan_.value().angle_max - scan_.value().angle_min) / scan_.value().angle_increment;    // 測距レーザの本数
    int central_num = laser_num_ / 2;        // 中心レーザの番号。このレーザの角度 = 0 rad
    double min_angle = 0 - central_num * scan_.value().angle_increment;  // angle_minの角度 [rad]。Roombaの3時方向? を0とする。
        // laser_numとcentral_num、min_angleはLiDARの仕様により固定のはずなので、値が確かめられたら数式を消してもよい

    std::vector<double> obs_ranges(laser_num_, 0.0);       // 各レーザによる測定値格納用配列
    std::vector<double> obs_angles(laser_num_, 0.0);       // 角度様配列
    resize_points();         // 障害物座標格納配列 (points型)。配列サイズを変更する
    for(int i=0; i<laser_num_; i++) obs_ranges[i] = scan_.value().ranges[i];     // ポインタを使えばもっと簡素にできるが要検証
    for(int i=0; i<laser_num_; i++) obs_angles[i] = scan_.value().angle_increment * i + min_angle;  

    // printf("OBS_DET test1: min_angle = %5.3f \n", min_angle);
    printf("\n");   // デバッグ用

    // 障害物情報配列への格納
    for(int i=0; i<laser_num_; i++){
        if(obs_ranges[i] < ignore_dist_) {
            // 検知物までの距離がignore_dist_未満の場合 (0.2m以下の場合) は障害物でないとする
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
    // 前後の点から1つだけ大きく離れた値の場合外れ値とみなす(.existをfalseにする)処理？
    // .existがtrueになっている点を全てpublishするのではなく、連続している点群を1つの情報としてpublishする方針でもいいかもしれない

    // ROS2メッセージに変換してpublish
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";        // laser 座標系に基づいた表記であることの明示
    
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
    
    points_print();     // デバッグ用printf


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

    double pillar_B[4] = {-0.79-M_PI/2, 0.62-M_PI/2, 2.23-M_PI/2, 3.74-M_PI/2};    // 各柱の角度領域の下端。[rad]
    double pillar_T[4] = {-0.63-M_PI/2, 1.21-M_PI/2, 2.75-M_PI/2, 3.94-M_PI/2};    // 各柱の角度領域の上端。[rad]


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
            printf("\tOBS_DET: i = %4d, r = %5.3f, θ[deg] = %5.3f\n", i, o_points_[i].r, o_points_[i].theta * 180/M_PI);
        }
    }
}
