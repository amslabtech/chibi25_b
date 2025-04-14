#include "localizer/localizer.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Localizer>();
	rclcpp::Rate loop_rate(node->getOdomFreq());
    node->initialize();
	while(rclcpp::ok())
	{
		node->process();
		rclcpp::spin_some(node); // call_back一回のため
	    loop_rate.sleep(); // 自力でループさせる
	}
	rclcpp::shutdown();
	return 0;
}
