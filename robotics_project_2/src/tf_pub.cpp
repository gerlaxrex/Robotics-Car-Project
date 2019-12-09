#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"

class tf_sub_pub
{
public:
  	tf_sub_pub(){  
  		sub = n.subscribe("/odom_std", 1000, &tf_sub_pub::callback_custom, this);  
	}
	
	void callback_custom(const nav_msgs::Odometry::ConstPtr& msg){
		tf2::Transform transform1;
        tf2::Quaternion q1;
		transform1.setOrigin( tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0));
		tf2::convert(msg->pose.pose.orientation, q1);
		transform1.setRotation(q1);
		tf2::Stamped<tf2::Transform> stmptf(transform1, ros::Time::now(),"odom");
		geometry_msgs::TransformStamped transformTfGeom = tf2::toMsg(stmptf);
		transformTfGeom.child_frame_id = "base_link";
		br.sendTransform(transformTfGeom);
	}

private:
	ros::NodeHandle n; 
	tf2_ros::TransformBroadcaster br;
	ros::Subscriber sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_pub_tf");
	tf_sub_pub my_tf_sub_bub;
	ros::spin();
	return 0;
}