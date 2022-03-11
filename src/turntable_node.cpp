#include "turntable_ros/Turntable.h"

/* Do not forget to load ADLINK drivers befor using the turntable !*/

int main( int argc, char **argv)
{
	// Initialize ROS objects
	ros::init(argc, argv, "turntable_node");

	// Retrieve eventual turntable GPIB primary address
	ros::NodeHandle nh;
	int turntablePAD = TURNTABLE_PAD;
	nh.getParam("turntable_GPIB_PAD", turntablePAD);
	
	Turntable turntable;

	ros::spin();
	return(0);
}
