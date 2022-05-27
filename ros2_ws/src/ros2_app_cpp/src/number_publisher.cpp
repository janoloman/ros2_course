#include "rclcpp/rclcpp.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
	NumberPublisherNode(): Node("number_publisher"), number_(0)
	{
		number_timer_ = this->create_wall_timer(std::chrono::seconds(1), 
																						std::bind(&NumberPublisherNode::printNumber, this));
		RCLCPP_INFO(this->get_logger(), "Number publisher has been started");
	}


private:
	void printNumber()	
	{
		RCLCPP_INFO(this->get_logger(), "%d", number_);
	}

	int number_;
	rclcpp::TimerBase::SharedPtr number_timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<NumberPublisherNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}