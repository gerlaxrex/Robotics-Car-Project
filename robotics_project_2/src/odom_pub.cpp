#include <ros/ros.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#define _USE_MATH_DEFINES
#define steering_factor 18

class pub_sub{
public:
    pub_sub(){
        sub = nh.subscribe("/speedsteer",1000, &pub_sub::odom_compute, this);
        sub_imu  = nh.subscribe("/swiftnav/rear/imu", 1000, &pub_sub::readImu, this);
        pub = nh.advertise<nav_msgs::Odometry>("/odom_std",1000);
        pub_imu = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
        //Set important values for the publisher and the message
	    distance_front = 1.765; last_received_message = 0.0; timeStep = 0.0;
        old_theta = 0.0; theta = 0.0; omega = 0.0; speed = 0.0;
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        //Odometry published from odom->base_link
        msg.header.frame_id = "odom";
	    msg.child_frame_id = "base_link";
    }

    //Callback used to change the frame_id of the IMU messages received.
    void readImu(const sensor_msgs::Imu::ConstPtr& messageImu){
        sensor_msgs::Imu msg = *messageImu;
        msg.header.frame_id = "imu_link";
        msg.header.stamp = ros::Time::now();
        pub_imu.publish(msg);
    }

    //Callback for the computation of odometry
    void odom_compute(const geometry_msgs::PointStamped::ConstPtr& message){
        
        if(last_received_message == 0.0) last_received_message = ros::Time::now().toSec();
	    double init = ros::Time::now().toSec(); //Approximate time when callback is called
        
        timeStep = init - last_received_message; 
        
        speed = message->point.y / 3.6; //speed in m/s
        omega = (speed/distance_front)*(tan(((message->point.x)/18)*M_PI/180));
        
        msg.twist.twist.linear.x = speed*cos(theta);
        msg.twist.twist.linear.y = speed*sin(theta);
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.z = omega;

        old_theta = theta;
        theta += omega*timeStep; //new value for theta

        if (abs(omega) <= 0.02){
            //Special case for omega approximately small (Runge-Kutta formula)
	        msg.pose.pose.position.x += speed*timeStep*cos(old_theta + omega*timeStep/2);
	        msg.pose.pose.position.y += speed*timeStep*sin(old_theta + omega*timeStep/2);	  
	    }else{  
            //Exact formula
	        msg.pose.pose.position.x += speed*(sin(theta) - sin(old_theta))/omega;
	        msg.pose.pose.position.y -= speed*(cos(theta) - cos(old_theta))/omega;
    	}

        if(theta >= 2*M_PI) theta -= 2*M_PI;
        else if (theta <= -2*M_PI) theta += 2*M_PI;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        
        msg.pose.pose.orientation = odom_quat;
	    
        //set the timestamp for the message
	    msg.header.stamp = ros::Time::now();

        //Publish the messages 
        pub.publish(msg);
        last_received_message = ros::Time::now().toSec();
    };
    
private:
    ros::NodeHandle nh;
    ros::Publisher pub, pub_imu;
    ros::Subscriber sub, sub_imu;
    nav_msgs::Odometry msg;
    double last_received_message, timeStep;
    float distance_front, old_theta, speed, omega, theta;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_sub_odom");
    pub_sub mypubsub;
    ros::spin();
    return 0;
}
