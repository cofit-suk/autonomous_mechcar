#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

int angle = 5;
int sub_data = 0;
void msgCallback(const std_msgs::UInt16::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    ROS_INFO("recieve msg = %d",sub_data);
     sub_data = msg->data;
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
    ros::Rate loop_rate(10);

    ros::Subscriber sub = nh.subscribe("chatter",100,msgCallback); // topic이 맞아 주제에 해당할때마다 반환 인터럽트?? 조건에 따라
    while(ros::ok())
    {
        angle = (int)sub_data/100;
        servo_msg.data = angle;

        msg2.linear.x=0.5; 
        msg2.linear.y=0.0;
        msg2.linear.z=0.0;   //no use
        msg2.angular.x=0.0;  //no use
        msg2.angular.y=0.0;  //no use
        msg2.angular.z=0.0;
        pub2.publish(msg2);
        pub1.publish(servo_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //ros::spin(); // 대기함수 call back을 대기 
    return 0;

}

