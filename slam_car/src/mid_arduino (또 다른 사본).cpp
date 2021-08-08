#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/MultiArrayLayout.h"

#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/UInt16MultiArray.h>

int angle = 5;
int sub_data = 0;
float l_x=0, l_y=0, l_z=0, a_x=0,a_y=0,a_z=0;
int red_flag = 0, blue_flag =0;
int stop_flag = 0;

void msgCallback(const std_msgs::UInt16::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    //ROS_INFO("recieve msg = %d",sub_data);
    //sub_data = msg->data;
    //ROS_INFO("recieve msg = %d",msg->stamp.nsec);
    //ROS_INFO("recieve msg = %d",msg->data);
}

void msgCallback_stop(const std_msgs::UInt16::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    stop_flag = msg->data;
    ROS_INFO("stop_flag = %d",stop_flag);
    //ROS_INFO("recieve msg = %d",sub_data);
    //sub_data = msg->data;
    //ROS_INFO("recieve msg = %d",msg->stamp.nsec);
    //ROS_INFO("recieve msg = %d",msg->data);
}
void msgCallback_color(const std_msgs::UInt16MultiArray::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    red_flag = msg->data[0];
    blue_flag = msg->data[1];

    ROS_INFO("red_flag = %d",red_flag);
    ROS_INFO("blue_flag = %d",blue_flag);
    //sub_data = msg->data;
    //ROS_INFO("recieve msg = %d",msg->stamp.nsec);
    //ROS_INFO("recieve msg = %d",msg->data);
}

void msgCallback_obs(const geometry_msgs::Twist::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    //ROS_INFO("recieve msg = %d",sub_data);
    l_x = msg->linear.x;
    l_y = msg->linear.y;
    l_z = msg->linear.z;
    a_x = msg->angular.x;
    a_y = msg->angular.y;
    a_z = msg->angular.z;
    if(red_flag){
        l_x = 0.0;
        l_y = 0.0;
        l_z = 0.0;
        a_x = 0.0;
        a_y = 0.0;
        a_z = 0.4;
    }
    else if(blue_flag){
        l_x = 0.0;
        l_y = 0.0;
        l_z = 0.0;
        a_x = 0.0;
        a_y = 0.0;
        a_z = -0.4;
    }
    //ROS_INFO("recieve msg = %d",msg->stamp.nsec);
    //ROS_INFO("recieve msg = %d",msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"mid_arduino");
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<std_msgs::UInt16>("servo", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    std_msgs::UInt16 servo_msg;
    geometry_msgs::Twist msg2;
    ros::Rate loop_rate(1);

    ros::Subscriber sub = nh.subscribe("chatter",100,msgCallback); // topic이 맞아 주제에 해당할때마다 반환 인터럽트?? 조건에 따라
    ros::Subscriber sub2 = nh.subscribe("cmd_vel_obs",1,msgCallback_obs); // topic이 맞아 주제에 해당할때마다 반환 인터럽트?? 조건에 따라
    ros::Subscriber sub3 = nh.subscribe("red_point",1,msgCallback_color);
    ros::Subscriber sub4 = nh.subscribe("stop_flag",1,msgCallback_stop);
    
    while(ros::ok())
    {
        angle = (int)sub_data/100;
        servo_msg.data = angle;

        if(stop_flag){
            l_x = 0.0;
            l_y = 0.0;
            l_z = 0.0;
            a_x = 0.0;
            a_y = 0.0;
            a_z = 0.0;
        }
        msg2.linear.x=l_x; 
        msg2.linear.y=l_y;
        msg2.linear.z=l_z;   //no use
        msg2.angular.x=a_x;  //no use
        msg2.angular.y=a_y;  //no use
        msg2.angular.z=a_z;
        pub2.publish(msg2);
        pub1.publish(servo_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //ros::spin(); // 대기함수 call back을 대기 
    return 0;

}

