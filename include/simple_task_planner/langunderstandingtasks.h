/**
 * @class LangUnderstandingTasks
 * @brief Perform language uderstanding tasks.
 *
 * Perform language understanding tasks by calling a ROS service wich uses the 
 * planning_msgs/parse_sentence_cfr srv format.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_LANGUNDTASKS_H
#define _JUSTINA_LANGUNDTASKS_H
#include <string>
#include <map>
#include "ros/ros.h"
#include "planning_msgs/parse_sentence_cfr.h"
class LangUnderstandingTasks
{
    private:
        /**
         * @struct CommandFrame
         * @brief A structure to store the Command Frame Representation of a 
         * parsed sentence.
         *
         * @var CommandFrame::command Contains the command corresponding
         * to the sentence.
         * @var CommandFrame::params Contains the pair parameters-values of the
         * corresponding command.
         */
        struct CommandFrame
        {
            std::string command;
            std::map<std::string, std::string> params;

            /**
             * @brief Creates a new CommandFrame struct object.
             *
             * @param newCommand The command to store.
             * @param newParams The parameters to store.
             */
            CommandFrame(std::string newCommand = "", 
                    std::map<std::string, std::string> newParams = 
                    std::map<std::string, std::string>()) :
                command(newCommand), params(newParams) {}
        };

        std::string m_parseSentenceServName; /**< Stores the name of the
                                               service which will perform
                                               the parse sentence task.*/

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
                "/language_understanding/parse_sentence_cfr");
        /**
         * @brief Performs the parse sentence task.
         * 
         * Makes a call to a ROS service, advertised by the language 
         * understanding node, to perform a sentence parsing task.
         *
         * @param sentenceToParse The sentence to parse
         * @param parseResult Stores the result command and parameters of the
         * parse sentence task.
         * @return true if the task was performed succesfully, false otherwise
         */
        bool parseSentence(std::string sentenceToParse, 
                CommandFrame &parseResult);

        /**
         * @brief Indicates if a given sentence corresponds to a positive 
         * confirmation (eg. robot yes).
         *
         * @param sentence The sentence to verify.
         * @return True if the sentence correspond to a positive  confirmation,
         * False otherwise.
         */
        bool isPositiveUserConfirmation(std::string sentence);
};
#endif
