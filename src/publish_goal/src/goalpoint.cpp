
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char *argv[])
{

    //奇葩问题
    std::cout<<666<<std::endl;
    // 初始化ROS节点
    ros::init(argc, argv, "goal_publisher");

    // 创建ROS节点句柄
    ros::NodeHandle nh("~");

    // 创建一个用于发布导航目标点的Publisher
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // 创建一个导航目标点消息
    geometry_msgs::PoseStamped goal_msg;

    // 设置导航目标点的时间戳
    goal_msg.header.stamp = ros::Time::now();

    // 设置导航目标点的参考坐标系
    goal_msg.header.frame_id = "odom"; 

    // 设置导航目标点的坐标（假设为(1.0, 2.0, 0.0)）
    goal_msg.pose.position.x = 1.0;
    goal_msg.pose.position.y = 2.0;
    goal_msg.pose.position.z = 0.0;
    // 设置导航目标点的方向（假设为朝向原点）
    goal_msg.pose.orientation.w = 1.0;

    // 发布导航目标点消息
    goal_pub.publish(goal_msg);

    // 等待一段时间以确保消息被发送
    ros::Duration(1.0).sleep();

    return 0;
}