#include <string>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher joint_pub;

void ctrl_Callback(const std_msgs::String msg){

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "frkl_cam");
    ros::NodeHandle n;

    joint_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber ctrl_sub = n.subscribe("ctrl", 1, ctrl_Callback);

    ros::Rate loop_rate(30);

    while(1){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
