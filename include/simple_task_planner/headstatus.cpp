#include "simple_task_planner/headstatus.h"

HeadStatus::HeadStatus(ros::NodeHandle &nh, std::string headPoseTopic):
    m_headPoseTopic(headPoseTopic)
{
    ros::Subscriber subHeadCurrentPose = nh.subscribe(m_headPoseTopic, 100, &HeadStatus::headPoseCallback, this);

    m_headPosePublisher = nh.advertise<std_msgs::Float32MultiArray>(m_headPoseTopic, 100);
}

void HeadStatus::headPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void HeadStatus::setHeadPose(float headPan, float headTilt)
{
    std_msgs::Float32MultiArray headPoseMsg;
    headPoseMsg.data.push_back(headPan);
    headPoseMsg.data.push_back(headTilt);
    m_headPosePublisher.publish(headPoseMsg);

}
