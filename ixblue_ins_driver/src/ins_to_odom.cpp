/*
 * Copyright (c) 2022, SENAI CIMATEC
 */

#include "ins_to_odom.h"


InsDvlListener::InsDvlListener() {

  ros::NodeHandle nh;
  ins_dvl_listener_ = nh.subscribe("mccr/ix/ins", 1, &InsDvlListener::insDvlCallback, this);



}

void InsDvlListener::insDvlCallback(const ixblue_ins_msgs::InsPtr& msg) {
  // --- Initialisation
  ROS_WARN_STREAM("HI " << msg->latitude << "HELLO");
}