#include "simple_task_planner/speechrecognitionstatus.h"

SpeechRecognitionStatus::SpeechRecognitionStatus(ros::NodeHandle &nh,
        std::string recoSentencesTopic) : 
    m_recoSentencesTopic(recoSentencesTopic)
{
    m_listenActivated = false;

    ros::Subscriber subHeadCurrentPose = nh.subscribe(m_recoSentencesTopic,
            100, &SpeechRecognitionStatus::recoSentenceCallback, this);
}

void SpeechRecognitionStatus::recoSentenceCallback(const 
        hri_msgs::RecognizedSpeech::ConstPtr &recoSentencesMsg)
{
    clearRecoSentencesQueue(m_lastRecoSentencesQueue);

    int hypothesisCount = recoSentencesMsg->hypotesis.size();
    for(int i=0; i<hypothesisCount; i++)
    {
        RecognizedSentence currentHypothesis(recoSentencesMsg->hypotesis[i], 
                recoSentencesMsg->confidence[i]);
        m_lastRecoSentencesQueue.push(currentHypothesis);

        if(m_listenActivated) 
            m_listenRecoSentencesQueue.push(currentHypothesis);
    }
}

void SpeechRecognitionStatus::startListening()
{
    m_listenActivated = true;
}

void SpeechRecognitionStatus::stopListening()
{
    m_listenActivated = false;
}

void SpeechRecognitionStatus::clearRecoSentencesQueue (
        std::queue<RecognizedSentence> &queue)
{
    std::queue<RecognizedSentence> emptyQueue;
    std::swap( queue, emptyQueue );
}

void SpeechRecognitionStatus::clearListenedSentencesQueue()
{
    if(m_listenActivated) 
        clearRecoSentencesQueue(m_listenRecoSentencesQueue);
}
