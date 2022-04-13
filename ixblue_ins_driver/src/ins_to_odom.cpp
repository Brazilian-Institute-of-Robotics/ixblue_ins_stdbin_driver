/*
 * Copyright (c) 2022, SENAI CIMATEC
 */

#include "ins_to_odom.h"


InsDvlListener::InsDvlListener() {

  ros::NodeHandle nh;
  ins_dvl_listener_ = nh.subscribe("ixblue_ins_driver/ix/ins", 1, &InsDvlListener::insDvlCallback, this);

  // ins_dvl_listener_ = nh.subscribe("ixblue_ins_driver/ix/dvl", 1, &InsDvlListener::insDvlCallback, this);

}

void InsDvlListener::insDvlCallback(const ixblue_ins_msgs::InsPtr& msg_ins) {
  // --- Initialisation
  ROS_WARN_STREAM("INS Latitude " << msg_ins->latitude << " graus");
  // ROS_WARN_STREAM("Longitudinal ground speed " << msg_dvl->xv1_groundspeed_ms << " m/s");
}