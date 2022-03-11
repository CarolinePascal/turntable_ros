#include "turntable_ros/Turntable.h"

/* Do not forget to load ADLINK drivers befor using the turntable !*/

int main( int argc, char **argv)
{
	// Initialize ROS objects
	ros::init(argc, argv, "turntable_node");

	Turntable turntable;

	ros::spin();
	return(0);
}
