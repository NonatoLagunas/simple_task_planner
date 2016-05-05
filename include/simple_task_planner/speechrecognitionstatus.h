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
#include <queue>
#include "ros/ros.h"
#include "hri_msgs/RecognizedSpeech.h"
class SpeechRecognitionStatus
{
    private:

        /**
         * @struct RecognizedSentence 
         * @brief A structure to represent a recognized sentence tuple.
         * 
         * @var RecognizedSentence::hypothesis Contains a hypothetical 
         * recognized sentence.
         * @var RecognizedSentence::confidence Contains the confidence of the 
         * recognized sentence.
         */
        struct RecognizedSentence
        {
            std::string hypothesis;
            double confidence;
            /**
             * @brief Creates a new RecognizedSpeech tuple.
             *
             * @param newHypothesis The hypothesis to store.
             * @param newConfidence The hypothesis asociated confidence.
             */
            RecognizedSentence(std::string newHypothesis, double newConfidence):
                hypothesis(newHypothesis), confidence(newConfidence) {}
        };

        std::queue<RecognizedSentence> 
            m_lastRecoSentencesQueue; /**< Stores the last recognized
                                        hypothesis */
        std::queue<RecognizedSentence> 
            m_listenRecoSentencesQueue; /**< Stores all the recognized 
                                          sentences during the listen mode */

        bool m_listenActivated; /**< Indicates when to start to enqueue the 
                                  recognized sentences */

        std::string m_recoSentencesTopic; /**< Stores the name of the 
                                            topic where the recognized
                                            sentences will be writen*/ 

        /**
         * @brief Current recognized sentence callback
         * 
         * Updates the robot's recognized speech sentences when the 
         * corresponding topic is updated and stores it in the recoSentences
         * queue when the listening mode is activated.
         *
         * @param recoSentencesMsg The new value of the topic when it's updated. 
         * @return void
         */
        void recoSentenceCallback(
                const hri_msgs::RecognizedSpeech::ConstPtr& recoSentencesMsg);

        /**
         * @brief Remove all the element of a recognize sentence  queue.
         *
         * @param queue[in,out] The queue to empty.
         */
        void clearRecoSentencesQueue (std::queue<RecognizedSentence> &queue);
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
         * @brief Activates the listening mode i.e. start the storing of the
         * recognized sentences.
         */
        void startListening();

        /**
         * @brief Deactivates the listening mode i.e. stop the storing of the
         * recognized sentences.
         */
        void stopListening();

        /**
         * @brief Clears the recognized sentences queue (filled during the 
         * listening mode).
         *
         * This method only clears the queue when the lisening mode is 
         * deactivated.
         */
        void clearListenedSentencesQueue();
};
#endif
