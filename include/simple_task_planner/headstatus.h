/**
 * @class HeadStatus
 * @brief Reads and modifies the current status of the robot's head.
 *
 * Reads the robot's head status from ROS, from the corresponding topics/servi-
 * ces published and/or advertised by the hardware head modules. Also it writes
 * the news values for the head status.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_HEADSTATUS_H
#define _JUSTINA_HEADSTATUS_H
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
class HeadStatus
{
    private:
        ros::Publisher m_headPosePublisher; /**< ROS publisher for the head
                                              pose topic*/

        std::string m_headPoseTopic; /**< Stores the name of the topic from 
                                       where the head status will be obtained*/

        float m_headPan; /**< Stores the current robot's head pan */

        float m_headTilt; /**< Stores the current robot's head tilt*/

        /**
         * @brief Head current pose callback
         * 
         * Updates the robot's head pose when the corresponding topic is
         * updated.
         *
         * @param poseMsg The new value of the topic when it's updated. 
         */
        void headPoseCallback(
                const std_msgs::Float32MultiArray::ConstPtr& poseMsg);
    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new HeadStatus object.
         *
         * @param nh The ROS Node ÇHandler of the simple task planner node.
         * @param headPoseTopic The name of the topic which will be updated
         * when the robot's head pose information.
         */
        HeadStatus(ros::NodeHandle &nh, std::string headPoseTopic = 
                "/hardware/head/current_pose");
        
        /**
         * @brief Returns the value of the current head pan.
         * 
         * @return The value of the current head pan.
         */
        float getHeadPan();

        /**
         * @brief Returns the value of the current head tilt.
         * 
         * @return The value of the current head tilt.
         */
        float getHeadTilt();

        /**
         * @brief Returns the value of the current head pan and tilt by 
         * reference.
         * 
         * @param headPan Stores the value of the current head pan.
         * @param headPan Stores the value of the current head tilt.
         * @return void
         */
        void getHeadPose(float &headPan, float &headTilt);

        /**
         * @brief Set the goal values for the head pose.
         *
         * Receives two float values that represent the goal head tilt and pan,
         * and writes it to the corresponding ROS topic.
         * 
         * @param headPan The goal head pan value.
         * @param headPan The goal head tilt value.
         * @return void
         */
        void setHeadPose(float headPan, float headTilt);
};
#endif
