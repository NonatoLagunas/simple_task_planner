#include "simple_task_planner/headstatus.h"

HeadStatus::HeadStatus(ros::NodeHandle &nh, std::string headPoseTopic):
    m_headPoseTopic(headPoseTopic)
{
    ros::Subscriber subHeadCurrentPose = nh.subscribe("/hardware/head/current_pose", 100, &HeadStatus::hdCurrentPoseCallback, this);
}

void HeadStatus::hdCurrentPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    m_headPan = msg->data[0];
    m_headTilt = msg->data[1];
}

float HeadStatus::getHeadPan()
{
    return m_headPan;
}

float HeadStatus::getHeadTilt()
{
    return m_headTilt;
}

void HeadStatus::getHeadPose(float &headPan, float &headTilt)
{
    headPan = m_headPan;
    headTilt = m_headTilt;
}
