#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Int16MultiArray.h>

int angle = 5;
int sub_data = 0;
int linear_vel_data = 0;
int angular_vel_data = 0;
int r_pwm = 0;
int l_pwm = 0;
int yolo_x=0;
int yolo_y=0;

void yoloCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    int xmin = 0, xmax = 0, ymin=0, ymax=0;
    xmin = msg-> bounding_boxes[0].xmin;
    xmax = msg-> bounding_boxes[0].xmax;
    ymin = msg-> bounding_boxes[0].ymin;
    ymax = msg-> bounding_boxes[0].ymax;
    yolo_x = (xmin+xmax)/2;
    yolo_y = (ymin+ymax)/2;
    if(yolo_y>=250){
        r_pwm = 250;
        l_pwm= 250;
    }
    else if(yolo_y<250){
        r_pwm= 0;
        l_pwm = 0;
    }
    //ROS_INFO("y= %d",yolo_y);
}
void msgCallback(const std_msgs::UInt16::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    ROS_INFO("recieve msg = %d",sub_data);
     sub_data = msg->data;
    //ROS_INFO("recieve msg = %d",msg->stamp.nsec);
    //ROS_INFO("recieve msg = %d",msg->data);
}

void key_msgCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) // 구조체 선언 msg로 선언한다.
{
    linear_vel_data = msg->data[0];
    angular_vel_data = msg->data[1];
    //ROS_INFO("linear_vel = %d",linear_vel_data);
    //ROS_INFO("angle_vel = %d",angular_vel_data);	
    //ROS_INFO("recieve msg = %d",msg->stamp.nsec);
    //ROS_INFO("recieve msg = %d",msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"mid_arduino");
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<std_msgs::Int16MultiArray>("motor_PWM", 10);
    //ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    std_msgs::Int16MultiArray speed_msg;
    geometry_msgs::Twist msg2;
    ros::Rate loop_rate(10);

    ros::Subscriber sub = nh.subscribe("chatter",100,msgCallback); // topic이 맞아 주제에 해당할때마다 반환 인터럽트?? 조건에 따라
    ros::Subscriber sub2 = nh.subscribe("/cmd_vel",100,key_msgCallback);
    //ros::Subscriber sub3 = nh.subscribe("/darknet_ros/bounding_boxes",100,yoloCallback);

    while(ros::ok())
    {
        speed_msg.data.clear();
        angle = (int)sub_data/100;

        if(linear_vel_data == 100 && angular_vel_data == 0)
        {
            speed_msg.data.push_back(200);
            speed_msg.data.push_back(200);
        }
        else if(linear_vel_data == -100 && angular_vel_data == 0)
        {
            speed_msg.data.push_back(-200);
            speed_msg.data.push_back(-200);
        }
        else if(linear_vel_data == 0 && angular_vel_data == 100)
        {
            speed_msg.data.push_back(-100);
            speed_msg.data.push_back(100);
        }
        else if(linear_vel_data == 0 && angular_vel_data == -100)
        {
            speed_msg.data.push_back(100);
            speed_msg.data.push_back(-100);
        }

        else
        {
            speed_msg.data.push_back(0);
            speed_msg.data.push_back(0);
        }
        ROS_INFO("R_speed_msg = %d",speed_msg.data[1]);
        ROS_INFO("L_speed_msg = %d",speed_msg.data[0]);
        msg2.linear.x=0.5; 
        msg2.linear.y=0.0;
        msg2.linear.z=0.0;   //no use
        msg2.angular.x=0.0;  //no use
        msg2.angular.y=0.0;  //no use
        msg2.angular.z=0.0;
        //pub2.publish(msg2);
        //R_speed_msg.data =r_pwm;
        //L_speed_msg.data =l_pwm;
        pub1.publish(speed_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //ros::spin(); // 대기함수 call back을 대기 
    return 0;

}

