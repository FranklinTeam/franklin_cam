#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("id_manager receive : [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "id_manager");
    ros::NodeHandle n;
    ros::Publisher id_manager_pub = n.advertise<std_msgs::String>("id_chatter_down", 1000);
	ros::Subscriber sub = n.subscribe("id_chatter_up", 1000, chatterCallback);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;

		ss << "hello world " << count;

        msg.data = ss.str();
        ROS_INFO("id_manager send : [%s]", msg.data.c_str());
        id_manager_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
