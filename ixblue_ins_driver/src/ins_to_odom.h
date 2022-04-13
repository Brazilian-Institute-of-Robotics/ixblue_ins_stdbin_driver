/*
 * Copyright (c) 2022, SENAI CIMATEC
 */

#include <ixblue_ins_msgs/Ins.h>
#include <ixblue_ins_msgs/SVS.h>
#include <ixblue_ins_msgs/DVL.h>
#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <ros/node_handle.h>
#include "ros/subscriber.h"

class InsDvlListener {
  public:
    InsDvlListener();


  private:
    ros::Subscriber ins_dvl_listener_;


    void insDvlCallback(const ixblue_ins_msgs::InsPtr& msg_ins);


};