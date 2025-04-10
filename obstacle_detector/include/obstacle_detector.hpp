#ifndef obstacle_detector_HPP
#define obstacle_detector_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>
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
    bool is_ignore_scan();  // 柱かの判定
    void resize_pionts();   // points配列の長さ決定
    void points_print();    // 障害物情報のprintf

    // 変数
    double obs_dist_ = 0.0;
    int laser_num_ = 0;
    std::optional<sensor_msgs::msg::LaserScan> scan_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;                 // scan SharedPtrは1stに倣って入れた
    rclcpp::Publisher</* ??? */>::SharedPtr o_points_pub_;

  private:
    std::vector<points> o_points_;


};

#endif  // b_obstacle_detector_hpp