# turntable_ros

ROS package dedicated to the control of the type 9640 Bruel & Kjaer turntable, using the GPIB IEEE 488.2 interface of the type 5997 controller and an ADLINK USB-3488A USB/GPIB adaptator.

## Description

This package contains a single node `turntable_node` which launches the following ROS services :

- `turntable_set_acceleration` : Sets the acceleration of the turntable, ranging from 1 to 6 (arbitrary unit) (turntable_ros::Int);
- `turntable_max360` : Sets the spinning limitation of the turntable, to avoid eventual cable knots (std_srvs::SetBool);
- `turntable_set_0_ref` : Sets 0° reference of the turntable to its current position (std_srvs::Empty);

- `turntable_set_abs_position` : Sends an absolute position command to the turntable (turntable_ros::Int);
- `turntable_set_rel_position` : Sends a relative position command to the turntable (turntable_ros::Int);

- `turntable_start_rotation` : Starts a rotation motion at a given rotation speed (in turns per second) (turntable_ros::Float);
- `turntable_stop_rotation` : Stops the rotation motion (std_srvs::Empty);

These 7 ROS services implement all functionnalities offered by the Bruel & Kjaer turntable. 

## Installation

A particular attention must be given to the installation and loading of the ADLINK drivers for the USB-3488A USB/GPIB adaptator.

## Usage

In order to access the turntable ROS services, one simply has to launch the `turntable_node` rosnode. If known, the GPIB primary address of the turntable wan be specified under the `turntable_GPIB_PAD` ROS parameter. Otherwise, a address listenning routine will retrieve the turntable GPIB primary address, provided it is the only GPIB device connected to the controlling computer.
