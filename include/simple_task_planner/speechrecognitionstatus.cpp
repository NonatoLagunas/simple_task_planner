#include "simple_task_planner/speechrecognitionstatus.h"

SpeechRecognitionStatus::SpeechRecognitionStatus(ros::NodeHandle &nh,
        std::string sprecRecoSentencesTopic) : 
    m_sprecRecoSentencesTopic(sprecRecoSentencesTopic)
{
}
