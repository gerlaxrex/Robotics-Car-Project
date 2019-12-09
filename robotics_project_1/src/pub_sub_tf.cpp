#include "ros/ros.h"
#include "robotics_project_1/odom_msg.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub
{
public:
  	tf_sub_pub(){  
  		sub = n.subscribe("/odom_custom", 1000, &tf_sub_pub::callback_custom, this);
	}

	//Callback for '/odom_custom' topic
	void callback_custom(const robotics_project_1::odom_msg::ConstPtr& msg){
		tf::Transform transform1, single_front;
		double R = 1.75/tan(msg->steer);
		transform1.setOrigin( tf::Vector3(msg->x, msg->y, 0));
		single_front.setOrigin(tf::Vector3(1.75, 0, 0));
		tf::Quaternion q1, q2;
		q1.setRPY(0, 0, msg->theta);
		q2.setRPY(0,0,msg->steer);
		transform1.setRotation(q1);
		single_front.setRotation(q2);
		br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "car"));
		br.sendTransform(tf::StampedTransform(single_front, ros::Time::now(), "car", "front_wheel"));
	}

private:
	ros::NodeHandle n; 
	tf::TransformBroadcaster br;
	ros::Subscriber sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_pub_tf");
	tf_sub_pub my_tf_sub_bub;
	ros::spin();
	return 0;
}