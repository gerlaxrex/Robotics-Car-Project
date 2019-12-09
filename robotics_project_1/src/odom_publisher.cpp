#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "robotics_project_1/floatStamped.h"
#include "robotics_project_1/odom_msg.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <robotics_project_1/carOdometryConfig.h>
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define _USE_MATH_DEFINES
#define steering_factor 18

class pub_sub{
public:
    pub_sub(){
        //Initilize the publishers
        pub = nh.advertise<robotics_project_1::odom_msg>("/odom_custom", 1000);
        pub_std = nh.advertise<nav_msgs::Odometry>("/odom_std", 1000);
        //Subscribe to the various topics
        left_wheel_sub.subscribe(nh, "/speedL_stamped",1);
        right_wheel_sub.subscribe(nh, "/speedR_stamped",1);
        steer_sub.subscribe(nh, "/steer_stamped", 1); 

	mysync.reset( new sync(MySyncPolicy(10), right_wheel_sub, left_wheel_sub, steer_sub));
        mysync->registerCallback(boost::bind(&pub_sub::speed_Listen, this,  _1, _2, _3));
        //Set important values for the publisher and the message
	distance_rear = 1.3; distance_front = 1.765; last_received_message = 0.0; timeStep = 0.0;
        odom_type = false;
        message.odom_type = "Differential";
        message.steer = 0.0;
        message.speed = 0.0;
        message.omega = 0.0;
        message.x = 0.0;
        message.y = 0.0;                
        message.theta = 0.0;
        old_theta = 0.0;
	
	message_std.header.frame_id = "world";
	message_std.child_frame_id = "base_link";
	
	f = boost::bind(&pub_sub::valueReconfigure, this, _1, _2);
        server.setCallback(f);
    }

    void speed_Listen(const robotics_project_1::floatStamped::ConstPtr& sr, const robotics_project_1::floatStamped::ConstPtr& sl, const robotics_project_1::floatStamped::ConstPtr& str){

        if(last_received_message == 0.0) last_received_message = ros::Time::now().toSec();
	double init = ros::Time::now().toSec(); //Approximate time when callback is called
        timeStep = init - last_received_message;
        message.speed = (sr->data + sl->data)/2;  //Speed of the rear wheels
	
	//Change the computation of the omega and steer according to the odom_type value
        if(!odom_type){
            message.odom_type = "Differential";
            message.steer = 0.0;
            message.omega = (sr->data - sl->data)/distance_rear;
        }else{
            message.odom_type = "Ackerman";
            message.steer = (str->data/steering_factor)*M_PI/180;
            message.omega = (message.speed/distance_front)*(tan(message.steer));
        }

	//update the values of the pose in according to the chosen odom type
        old_theta = message.theta;
        message.theta += message.omega*timeStep;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(message.theta);
        if (abs(message.omega) <= 0.02){
                //Special case for omega approximately small (Runge-Kutta formula)
	        message.x += message.speed*timeStep*cos(message.theta + message.omega*timeStep/2);
	        message.y += message.speed*timeStep*sin(message.theta + message.omega*timeStep/2);	  
	}else{  
                //Exact formula
	        message.x += message.speed*(sin(message.theta) - sin(old_theta))/message.omega;
	        message.y -= message.speed*(cos(message.theta) - cos(old_theta))/message.omega;
	}

        if(message.theta >= 2*M_PI) message.theta -= 2*M_PI;
        else if (message.theta <= -2*M_PI) message.theta += 2*M_PI;

        //set the position in the standard message
        message_std.pose.pose.position.x = message.x;
        message_std.pose.pose.position.y = message.y;
	message_std.pose.pose.position.z = 0.0;
        message_std.pose.pose.orientation = odom_quat;

	//set the velocity in the standard message
	message_std.child_frame_id = "base_link";
	message_std.twist.twist.linear.x = message.speed * cos(message.theta);
	message_std.twist.twist.linear.y = message.speed * sin(message.theta);
	message_std.twist.twist.angular.z = message.omega;

	//set the timestamps of both messages
	message.header.stamp = ros::Time::now();
	message_std.header.stamp = ros::Time::now();

	//publish both messages
        pub.publish(message);
        pub_std.publish(message_std);

        last_received_message = ros::Time::now().toSec();
    };
    
    void valueReconfigure(robotics_project_1::carOdometryConfig& config, uint32_t level){
        ROS_INFO("Reconfigure Request: %f %f %f", config.posx, config.posy, config.theta);

	message.x = config.posx;
        message.y = config.posy;
        message.theta = config.theta;
	odom_type = config.odom_type;
    }
    
private:
    ros::NodeHandle nh;
    ros::Publisher pub, pub_std;
    message_filters::Subscriber<robotics_project_1::floatStamped> left_wheel_sub, right_wheel_sub, steer_sub;  //Subscribers with message filters
    //Message Filters members
    typedef message_filters::sync_policies::ApproximateTime<robotics_project_1::floatStamped, robotics_project_1::floatStamped, robotics_project_1::floatStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> sync;
    boost::shared_ptr<sync> mysync;
    //Messages for the topics
    robotics_project_1::odom_msg message;
    nav_msgs::Odometry message_std;

    double last_received_message, timeStep;
    bool odom_type;
    float distance_rear, distance_front, old_theta;
    //Dynamic Reconfigure members
    dynamic_reconfigure::Server<robotics_project_1::carOdometryConfig> server;
    dynamic_reconfigure::Server<robotics_project_1::carOdometryConfig>::CallbackType f;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_sub_odom");
    pub_sub mypubsub;
    ros::spin();
    return 0;
}
