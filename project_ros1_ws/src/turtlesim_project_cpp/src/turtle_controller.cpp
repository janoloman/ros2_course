#include <ros/ros.h>
#include <my_robot_msgs/Coordinates2D.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class TurtleController
{

public:
    TurtleController(ros::NodeHandle *nh) : turtlesim_up_(false), target_up_(false)
    {
        cmd_vel_publisher_ = nh->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        pose_subscriber_ = nh->subscribe("/turtle1/pose", 10, &TurtleController::callbackTurtlePose, this);
        target_subscriber_ = nh->subscribe("target_coordinates", 10, &TurtleController::callbackTarget, this);
        control_loop_timer_ = nh->createTimer(ros::Duration(0.01), std::bind(&TurtleController::controlLoop, this));
        ROS_INFO("Turtle controller has been started.");
    }

private:
    turtlesim::Pose pose_;
    my_robot_msgs::Coordinates2D target_;
    bool turtlesim_up_;
    bool target_up_;

    ros::Publisher cmd_vel_publisher_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber target_subscriber_;
    ros::Timer control_loop_timer_;

    void callbackTurtlePose(const turtlesim::Pose &msg)
    {
        pose_ = msg;
        turtlesim_up_ = true;
    }

    void callbackTarget(const my_robot_msgs::Coordinates2D &msg)
    {
        if (msg.x > 0.0 && msg.x < 11.0 && msg.y > 0.0 && msg.y < 11.0)
        {
            ROS_INFO("Received new valid target");
            target_ = msg;
            target_up_ = true;
        }
    }

    void controlLoop()
    {
        if (!turtlesim_up_ || !target_up_)
        {
            return;
        }

        double dist_x = target_.x - pose_.x;
        double dist_y = target_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        geometry_msgs::Twist msg;

        if (distance > 0.5)
        {
            // position
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        cmd_vel_publisher_.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_controller");
    ros::NodeHandle nh;
    TurtleController controller = TurtleController(&nh);
    ros::spin();
    return 0;
}