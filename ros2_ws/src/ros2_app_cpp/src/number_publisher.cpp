#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
	NumberPublisherNode(): Node("number_publisher")
	{
		this->declare_parameter("number_to_publish",2);
		this->declare_parameter("number_publish_frecuency",1.0);

		number_ = this->get_parameter("number_to_publish").as_int();
		double publish_frecuency = this->get_parameter("number_publish_frecuency").as_double();

		number_publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);

		number_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_frecuency)), 
																						std::bind(&NumberPublisherNode::publishNumber, this));
		RCLCPP_INFO(this->get_logger(), "Number publisher has been started");
	}


private:
	void publishNumber()
	{
		auto msg = std_msgs::msg::Int64();
		msg.data = number_;
		number_publisher_->publish(msg);
	}

	int number_;
	rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr number_publisher_;
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