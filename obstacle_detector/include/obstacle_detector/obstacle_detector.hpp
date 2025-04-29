#ifndef obstacle_detector_HPP
#define obstacle_detector_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <math.h>

// 座標格納用構造体
struct points{
  bool exist; // trueなら障害物あり
  double x;
  double y;
  double r;
  double theta;
};

class ObstacleDetector : public rclcpp::Node
{
  public:
    ObstacleDetector();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process();

    // 関数
    void scan_obstacle();   // 障害物情報の読み取りとpublish
    bool is_ignore_scan(double angle);  // 柱かの判定
    void resize_points();   // points配列の長さ決定
    void points_print();    // 障害物情報のprintf

  private:
    // 変数
    double obs_dist_;
    double ignore_dist_;
    int laser_num_;
    std::vector<double> angle_bottom_;
    std::vector<double> angle_top_;

    double hz_;
    std::string robot_frame_;

    std::vector<points> o_points_;
    std::optional<sensor_msgs::msg::LaserScan> scan_;

    // pub & sub
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;                 // scan SharedPtrは1stに倣って入れた
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr o_points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;



};

#endif  // b_obstacle_detector_hpp