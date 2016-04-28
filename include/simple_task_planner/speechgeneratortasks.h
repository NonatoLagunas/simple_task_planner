/**
 * @class SpeechGeneratorTasks
 * @brief Perform speech generator tasks.
 *
 * Prform the speech generator tasks using the Microsoft SPGEN module, 
 * running on Windows and connected to BlackBoard, via BBROS Bridge.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_SPEECHTASKS_H
#define _JUSTINA_SPEECHTASKS_H
#include <string>
#include "ros/ros.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
class SpeechGeneratorTasks
{
    public:
        /*
         * @brief Class Constructor
         * 
         * Creates a new SpeechGeneratorTasks object.
         */
        SpeechTasks();
        /*
         * @brief Performs a synchronous text-to-speech task.
         * 
         * Makes a call to a ROS service, advertised by the BBROS-Bridge
         * node, to perform a text-to-speech task using the SPGEN module,
         * running on windows an connected to BlackBoard.
         *
         * @param textToSpeech The message to generate
         * @param timeOut Waiting time (in milliseconds) for the BlackBoard to
         * respond
         * @return true if the task was performed succesfully, false otherwise
         */
        bool syncSpeech(std::string textToSpeech, int timeOut);
        /*
         * @brief Performs an asynchronous text-to-speech task.
         * 
         * Makes a call to a ROS service, advertised by the BBROS-Bridge
         * node, to perform a text-to-speech task using the SPGEN module,
         * running on windows an connected to BlackBoard.
         *
         * @param textToSpeech The message to generate
         * @return void
         */
        void asyncSpeech(std::string textToSpeech);
};
#endif
