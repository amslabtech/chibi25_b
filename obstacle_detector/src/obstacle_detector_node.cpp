<<<<<<< HEAD
#include "obstacle_detector.hpp"
=======
#include "obstacle_detector/obstacle_detector.hpp"
>>>>>>> 8edb808d1a695da6c1221b3ae8bf54da72834e3e

// 今までのnode.cppほぼそのままコピペ
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ObstacleDetector> obsdetec = std::make_shared<ObstacleDetector>();
    rclcpp::spin(obsdetec);
    rclcpp::shutdown();

    return 0;
}