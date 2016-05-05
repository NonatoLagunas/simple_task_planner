/**
 * @class SpeechRecognitionStatus
 * @brief Reads the current status of the robot's speech recognition system.
 *
 * Reads the robot's speech recognition status from ROS, from the corresponding 
 * topics published and/or advertised by the speech recognition module.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_SPRECSTATUS_H
#define _JUSTINA_SPRECSTATUS_H
#include <string>
#include "ros/ros.h"
#include "hri_msgs/RecognizedSpeech.h"
class SpeechRecognitionStatus
{
    private:

        std::string m_recoSentencesTopic; /**< Stores the name of the 
                                            topic where the recognized
                                            sentences will be writen*/ 

        /**
         * @brief Current recognized sentence callback
         * 
         * Updates the robot's recognized speech sentence when the 
         * corresponding topic is updated.
         *
         * @param recoSentenceMsg The new value of the topic when it's updated. 
         */
        void recoSentenceCallback(
                const hri_msgs::RecognizedSpeech::ConstPtr& msg);
    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new SpeechRecognitionStatus object.
         *
         * @param nh The ROS Node Handler of the simple task planner node.
         * @param sprecRecoSentencesTopic The name of the topic which will be 
         * updated with the current recognized speech value.
         */
        SpeechRecognitionStatus(ros::NodeHandle &nh, std::string 
                recoSentencesTopic= "recognizedSpeech");
        
        /**
         * @brief Returns the value of the current head pan and tilt by 
         * reference.
         * 
         * @param headPan Stores the value of the current head pan.
         * @param headPan Stores the value of the current head tilt.
         * @return void
         */
        //void getHeadPose(float &headPan, float &headTilt);
};
#endif
