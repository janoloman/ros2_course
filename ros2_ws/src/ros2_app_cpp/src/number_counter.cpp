#include "rclcpp/rclcpp.hpp"

class NumberCounterNode : public rclcpp::Node 
{
public:
    NumberCounterNode() : Node("number_counter") 
    {
        RCLCPP_INFO(this->get_logger(), "Number counter has been started");
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
