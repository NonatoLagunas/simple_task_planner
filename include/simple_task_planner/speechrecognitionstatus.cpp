#include "simple_task_planner/speechrecognitionstatus.h"

SpeechRecognitionStatus::SpeechRecognitionStatus(ros::NodeHandle &nh,
        std::string recoSentencesTopic) : 
    m_recoSentencesTopic(recoSentencesTopic)
{
    ros::Subscriber subHeadCurrentPose = nh.subscribe(m_recoSentencesTopic,
            100, &SpeechRecognitionStatus::recoSentenceCallback, this);
}

void SpeechRecognitionStatus::recoSentenceCallback(const 
        hri_msgs::RecognizedSpeech::ConstPtr &recoSentenceMsg)
{

}
