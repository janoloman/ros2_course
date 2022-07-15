#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/coordinates2_d.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleController::callbackTurtlePose, this, std::placeholders::_1));
        
        target_subscriber_ = this->create_subscription<my_robot_interfaces::msg::Coordinates2D>(
            "/target_coordinates", 10, std::bind(&TurtleController::callbackTarget, this, std::placeholders::_1));

        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(100.0)), 
            std::bind(&TurtleController::ControlLoop, this));
        

        RCLCPP_INFO(this->get_logger(), "Turtle counter has been started");
    }

private:
    turtlesim::msg::Pose pose_;
    my_robot_interfaces::msg::Coordinates2D target_;
    bool turtlesim_up_;
    bool target_up_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::Coordinates2D>::SharedPtr target_subscriber_;
    
    rclcpp::TimerBase::SharedPtr control_loop_timer_;


    void callbackTurtlePose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = *msg.get();
        turtlesim_up_ = true;        
    }

    void callbackTarget(const my_robot_interfaces::msg::Coordinates2D::SharedPtr msg)
    {
        if (msg->x > 0.0 && msg->x < 11.0 && msg->y > 0.0 && msg->y < 11.0)
        {
            RCLCPP_INFO(this->get_logger(), "Received new valid target");
            target_ = *msg.get();
            target_up_ = true;
        }
    }

    void ControlLoop()
    {
        if(!turtlesim_up_ || !target_up_)
            return;

        double dist_x = target_.x - pose_.x;
        double dist_y = target_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            // position
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
                angle_diff -= 2 * M_PI;
            else if (angle_diff < -M_PI)
                angle_diff += 2 * M_PI;
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        cmd_vel_publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
