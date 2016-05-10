/**
 * @class SimpleTasks
 * @brief Contains methods to perform multiple robot simple tasks.
 *
 * This class contains methods that use the primitives tasks libraries in order
 * to perform composed tasks. Examples of composed tasks which this class can
 * perform are:
 *  - Ask a question to the human and waits for a human confirmation.
 *  - Find an object using the sensors and take it.
 *  - Move to a location and find a person.
 * The main idea of this class is asociate each task with a ROS service to be 
 * available for other ROS nodes.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_SIMPLETASKS_H
#define _JUSTINA_SIMPLETASKS_H
#include <string>
#include <chrono>
#include <iostream>
#include "ros/ros.h"
#include "justina_tools/speechgeneratortasks.h"
#include "justina_tools/langunderstandingtasks.h"

class SimpleTasks
{
    private:
        SpeechGeneratorTasks m_spgenTasks; /**< Object to perform speech
                                             generator tasks. */

        LangUnderstandingTasks m_langundTasks; /**< Object to perform language 
                                                 understading tasks. */
    public:
        /**
         * @brief Class constructor.
         * 
         * Creates a new SimpleTasks object.
         */
        SimpleTasks();

        /**
         * @brief Performs the simple task to ask a question to the user and 
         * wait for a confirmation from the user.
         *
         * This task consist of the following steps:
         *  1. The robot ask a question to the user.
         *  2. For a period of time, the robot waits for the user's answer 
         *  (yes or not) and while the robot waits for the answer it will 
         *  repeat the question with certain frequency.
         * @param questionToAsk The question that the robot will ask to the user.
         * @param timeOut The amount of time (milliseconds) of the robot's 
         * wait-for-answer. 
         * @param repatTimeOut The amount of time (milliseconds) that the robot
         * must wait before repeat the question to the user if this does not 
         * respond.
         * @return True if the user answer with a positive confirmation to the 
         * robot's question. False otherwise.
         */
        bool askAndWaitForConfirm(std::string questionToAsk, int timeOut,
                int repeatTimeOut=5000);
};
#endif
