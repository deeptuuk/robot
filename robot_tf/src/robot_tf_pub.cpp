#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "tf_transform_pub");
	ros::NodeHandle n;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}