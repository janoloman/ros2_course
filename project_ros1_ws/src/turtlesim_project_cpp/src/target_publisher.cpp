#include <ros/ros.h>
#include <my_robot_msgs/Coordinates2D.h>
#include <cstdlib>

class TargetPublisher
{

public:
    TargetPublisher(ros::NodeHandle *nh)
    {
        double publish_interval;
        ros::param::param<double>("~publish_interval", publish_interval, 3.0);
        pub_ = nh->advertise<my_robot_msgs::Coordinates2D>("target_coordinates", 10);
        timer_ = nh->createTimer(ros::Duration(publish_interval), std::bind(&TargetPublisher::sendRandomCoordinates, this));
        ROS_INFO("Target publisher has been started");
    }

private:
    ros::Publisher pub_;
    ros::Timer timer_;

    // returns random double number in range [0.0, 1.0)
    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    void sendRandomCoordinates()
    {
        double x = randomDouble() * 11.0;
        double y = randomDouble() * 11.0;
        publishCoordinates(x, y);
    }

    void publishCoordinates(double x, double y)
    {
        my_robot_msgs::Coordinates2D msg;
        msg.x = x;
        msg.y = y;
        pub_.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_publisher");
    ros::NodeHandle nh;
    TargetPublisher target_publisher = TargetPublisher(&nh);
    ros::spin();
    return 0;
}