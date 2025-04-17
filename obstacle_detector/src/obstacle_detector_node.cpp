#include "obstacle_detector.hpp"

// 今までのnode.cppほぼそのままコピペ
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ObstacleDetector> obsdetec = std::make_shared<ObstacleDetector>();
    rclcpp::spin(obsdetec);
    rclcpp::shutdown();

    return 0;
}