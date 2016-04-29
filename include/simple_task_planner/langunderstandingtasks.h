/**
 * @class LangUnderstandingTasks
 * @brief Perform language uderstanding tasks.
 *
 * Perform the language understanding tasks using the language_understanding 
 * node, running on Linux and connected to ROS, via ROS services.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_LANGUNDTASKS_H
#define _JUSTINA_LANGUNDTASKS_H
#include <string>
#include "ros/ros.h"
#include "language_understanding/parse_sentence.h"
class LangUnderstandingTasks
{
    private:
        std::string m_parseSentenceServName; /**< Stores the name of the
                                               service which will perform
                                               the parse sentence task */

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new LangUnderstandingTasks object.
         *
         * @param parseSentenceServName The name of the service which will 
         * perform the synchronous text-to-speech task. Default "/spg_say"
         */
        LangUnderstandingTasks(std::string parseSentenceServName=
                "/language_understanding/parse_sentence");
        /**
         * @brief Performs the parse sentence task.
         * 
         * Makes a call to a ROS service, advertised by the language 
         * understanding node, to perform a sentence parsing task.
         *
         * @param sentenceToParse The sentence to parse
         * @param parseResult Stores the result command of the parse sentence 
         * task
         * @return true if the task was performed succesfully, false otherwise
         */
        bool parseSentence(std::string sentenceToParse, 
                std::string &parseResult);
};
#endif
