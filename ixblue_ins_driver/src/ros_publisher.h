#pragma once

#include <ixblue_ins_msgs/Ins.h>
#include <ixblue_ins_msgs/SVS.h>
#include <ixblue_ins_msgs/DVL.h>
#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <nav_msgs/Odometry.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

#include <string>

#include "diagnostics_publisher.h"

class ROSPublisher
{
public:
    ROSPublisher();
    void onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                         const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // Standard ros msgs
    static sensor_msgs::ImuPtr
    toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
             bool use_compensated_acceleration);

    static sensor_msgs::NavSatFixPtr
    toNavSatFixMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    static sensor_msgs::TimeReferencePtr
    toTimeReference(const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // iXblue ros msgs
    static ixblue_ins_msgs::InsPtr
    toiXInsMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    static ixblue_ins_msgs::SVSPtr
    toSVSMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    static ixblue_ins_msgs::DVLPtr
    toDVLMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    nav_msgs::Odometry convertToOdomIns();

    // INS get message
    sensor_msgs::ImuPtr imuMsgGet;
    ixblue_ins_msgs::DVLPtr DVLMsgGet;
    ixblue_ins_msgs::InsPtr iXinsMsgGet;
    ixblue_ins_msgs::SVSPtr SVSMsgGet;

protected:
    // Header
    std_msgs::Header getHeader(const ixblue_stdbin_decoder::Data::NavHeader& headerData,
                               const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    // Launch parameters
    std::string frame_id;
    std::string time_source;
    std::string time_origin;
    bool use_compensated_acceleration;
    bool publish_odom_ins;
    bool dvl_available; // TODO - Remove later. If the DVLMsgGet works

    ros::NodeHandle nh;

    // Publishers
    ros::Publisher stdImuPublisher;
    ros::Publisher stdNavSatFixPublisher;
    ros::Publisher stdTimeReferencePublisher;
    ros::Publisher stdInsPublisher;
    ros::Publisher stdSVSPublisher;
    ros::Publisher stdDVLPublisher;
    DiagnosticsPublisher diagPub;

    // Odom Publisher
    ros::Publisher stdInsOdomPublisher;

    // Utils
    bool useInsAsTimeReference = true;
    bool useUnixAsTimeOrigin = true;

};
