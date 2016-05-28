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
#include <map>
#include <vector>
#include <utility>
#include "ros/ros.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/speechrecognitionstatus.h"
#include "robot_service_manager/langunderstandingtasks.h"
#include "robot_service_manager/headstatus.h"
#include "robot_service_manager/facerecognitiontasks.h"

class SimpleTasks
{
    private:
        SpeechGeneratorTasks m_spgenTasks; /**< Object to perform speech
                                             generator tasks. */

        LangUnderstandingTasks m_langundTasks; /**< Object to perform language 
                                                 understading tasks. */

		SpeechRecognitionStatus m_sprec; /**< Object to know the sprec status.
                                           */

        FaceRecognitionTasks m_recoFaces; /**< Object to prform face 
                                            recognition tasks. */

        HeadStatus m_headStatus; /**< Object to perform head movements. */

        /**
         * @brief Initiaizes a vector that stores head movements to its default
         * value.
         *
         * @param t_vector The vector to initialize.
         */
        void initializeHeadVector(
                std::vector<std::pair<float, float> > &t_vector
                );
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

        /**
         * @brief Performs the simple task to wait for a start-guiding voice 
         * command comming from the user. 
         *
         * This task consist in the following steps:
         *  1. For a period of time, the robot wait's for the user start guide
         *  command (i.e. ~robot start guiding me). 
         *  While the robot waits for the command it can repeat a given 
         *  sentence to the user (something like "i am waiting for the 
         *  command").
         *
         * @param[in] t_sentenceToRepeat The sentence that the robot will speech
         * to the user.
         * @param[out] t_goalToGuide The goal that the robot must guide 
         * (indicated by the voice command).
         * @param[in] t_timeout The amount of time (milliseconds) that the robot
         * will wait for the command.
         * @param[in] t_repeatTimeout The amount of time (milliseconds) that the
         * robot must wait to repeat the start sentence to the user.
         * @return True if the user gives a valid guide command to the robot.
         * False otherwise.
         */
        bool waitForStartGuideCommand(
                std::string t_sentenceToRepeat,
                std::string &t_goalToGuide,
                int t_timeout, 
                int t_repeatTimeout=5000
                );

        /**
         * @brief Perform the task of wait for a valid voice command comming 
         * from the user.
         *
         * This task consist in the following steps:
         *  1. For a period of time, the robot wait's for a user voice command.
         *  2. When the robot hears a command, it try to parse it.
         *  3. If the parse is valid then, the command and the params are 
         *  returned. If not, back to the step 1.
         *
         *  @param[out] t_command The resulted parsed command. If no valid 
         *  command is given during the valid period of time, the returned 
         *  value will be an empty string.
         *  @param[out] t_params The resulted params of the parsed command.
         *  @param[in] t_timeout The amount of time (milliseconds) that the 
         *  robot will wait for a command.
         */
        bool waitForCommand(std::string &t_command,
                std::map<std::string, std::string> &t_params, int t_timeout);

        /**
         * @brief Perform the task of remeber a person's face.
         *
         * This tasks consists in the following steps:
         *  1. There is a person in front of the robot (looking, or not, at the
         *      robot's camera).
         *  2. The robot start to instruct to the operator.
         *  3. After the instruction, the robot starts to search for a human 
         *      face in front of it.
         *  4. If the robot does not find a face, it starts to move the head in
         *      different positions and tries to find a fece again.
         *  5. When the robot finds a face, it starts to training the face, at
         *      the end of this step the robot announce tht the training is 
         *      finished.
         *
         * @param t_findInstructions The robot instructions for the human, 
         * during the find face phase. To speech the default instructions a 
         * parameter value of "default" must be passed.
         * @param t_faceName The name to assign to the remebered human.
         * @param t_headMovements The head movements to perform during the find
         * search phase. To perform a default movements set, an empty list must
         * be passed as parameter. The head movements consist of a vector of 
         * float pairs in which each pair indicates the headPan and headTilt
         * respectively.
         * @return True if the human face was found and remember succesfully. 
         * False otherwise.
         */
        bool faceSearchAndRemeber(
                std::string t_findInstructions,
                std::string t_faceName, 
                std::vector<std::pair<float, float> > t_headMovements =
                    std::vector<std::pair<float, float> >()
                );
        
        /**
         * @brief Performs the task of ask for a person's name and store it.
         *
         * Assuming there is a person in front of the robot.
         * This tasks consists in the following steps:
         *  1. The robot asks for the person's name.
         *  2. The robot waits until the person provide its name. While the 
         *      robot is waiting it will repeat the question with a specified
         *      frequency.
         *  3. When the person provides his name, the robot will ask for a name
         *      confimation to the person. 
         *  4. If the confirmation is positive, the task end. If not, the task
         *      start again from the begginning (there will be an attempts 
         *      limit).
         *
         * @param[out] t_personName Stores the obtained person's name.
         * @param[in] attemptTimeout The limit time of each learn name attempt.
         * @param[in] t_repeatTimeout The amount of time (milliseconds) that the
         * robot must wait to ask to the user for her/his name again.
         * @param[in] t_maxTaskAttempts The max number of attempts in the 
         * confirmation step.
         * @return True if the person's name was obtained succesfully. False 
         * otherwise.
         */
        bool askForName(std::string &t_personName, int attemptTimeout = 30000,
                int t_repeatTimeout = 10000, int t_maxTaskAttempts = 3);

};
#endif
