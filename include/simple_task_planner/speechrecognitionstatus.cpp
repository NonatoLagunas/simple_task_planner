#include "simple_task_planner/speechrecognitionstatus.h"

SpeechRecognitionStatus::SpeechRecognitionStatus(ros::NodeHandle &nh,
        std::string recoSentencesTopic) : 
    m_recoSentencesTopic(recoSentencesTopic)
{
    /**
     * The listen mode is turned off by default
     */
    m_listenActivated = false;

    /**
     * Subscribe to the recognized speech ros topic
     */
    ros::Subscriber subHeadCurrentPose = nh.subscribe(m_recoSentencesTopic,
            100, &SpeechRecognitionStatus::recoSentenceCallback, this);
}

void SpeechRecognitionStatus::recoSentenceCallback(const 
        hri_msgs::RecognizedSpeech::ConstPtr &recoSentencesMsg)
{
    /**
     * Clear the last recognized sentences queue in order to enqueue the new
     * recognized sentences
     */
    clearRecoSentencesQueue(m_lastRecoSentencesQueue);

    int hypothesisCount = recoSentencesMsg->hypotesis.size();
    /**
     * Add each recognized sentence to the last recognized sentences queue and, 
     * if the listen mode is activated, in the listen recognized sentences 
     * queue.
     */
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
    /**
     * Creates an empty queue
     */
    std::queue<RecognizedSentence> emptyQueue;
    /**
     * Swap the receiven queue with the empty queue
     */
    std::swap( queue, emptyQueue );
}

void SpeechRecognitionStatus::clearListenedSentencesQueue()
{
    if(m_listenActivated) 
        clearRecoSentencesQueue(m_listenRecoSentencesQueue);
}
