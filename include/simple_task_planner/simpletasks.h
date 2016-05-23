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
#include <boost/chrono.hpp>
#include <iostream>
#include "ros/ros.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/speechrecognitionstatus.h"
#include "robot_service_manager/langunderstandingtasks.h"

class SimpleTasks
{
    private:
        SpeechGeneratorTasks m_spgenTasks; /**< Object to perform speech
                                             generator tasks. */

        LangUnderstandingTasks m_langundTasks; /**< Object to perform language 
                                                 understading tasks. */

		SpeechRecognitionStatus m_sprec; /**< Object to know the sprec status.
                                           */

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
         *
         * @param t_questionToAsk The question that the robot will ask to the 
         * user.
         * @param t_timeout The amount of time (milliseconds) of the robot's 
         * wait-for-answer. 
         * @param t_repatTimeout The amount of time (milliseconds) that the 
         * robot must wait before repeat the question to the user if this does 
         * not respond. If no time given, the question will be repeated every 
         * 5000 millisecs.
         * @return True if the user answer with a positive confirmation to the 
         * robot's question. False otherwise.
         */
        bool askAndWaitForConfirm(std::string t_questionToAsk, int t_timeout,
                int t_repeatTimeout=5000);

        /**
         * @brief Performs the simple task to wait for a start-follow voice 
         * command comming from the user. 
         *
         * This task consist in the following steps:
         *  1. For a period of time, the robot wait's for the user start follow
         *  command (i.e. ~robot start follow me). 
         *  While the robot waits for the command it can repeat a given 
         *  sentence to the user (something like "i am waiting for the 
         *  command").
         *
         * @param[in] t_sentenceToRepeat The sentence that the robot will speech
         * to the user.
         * @param[out] t_goalToFollow The goal that the robot must follow 
         * (indicated by the user).
         * @param[in] t_timeout The amount of time (milliseconds) that the robot
         * will wait for the answer.
         * @param[in] t_repeatTimeout The amount of time (milliseconds) that the
         * robot will repeat the start sentence tot he user.
         * @return True if the user gives a valid start command to the robot.
         * False otherwise.
         */
        bool waitForStartFollowCommand(
                std::string t_sentenceToRepeat,
                std::string &t_goalToFollow,
                int t_timeout, 
                int t_repeatTimeout=5000
                );
};
#endif
