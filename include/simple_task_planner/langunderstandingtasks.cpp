#include "simple_task_planner/langunderstandingtasks.h"

LangUnderstandingTasks::LangUnderstandingTasks (
        std::string parseSentenceServName) : m_parseSentenceServName(
            parseSentenceServName)
{
}

bool LangUnderstandingTasks::isPositiveUserConfirmation(std::string sentence)
{
    CommandFrame parseResult;
    parseSentence(sentence, parseResult);

    if(parseResult.command.compare("CONFIRMATION") == 0)
    {
        if(parseResult.params["confirmation"].compare("yes")==0)
        {
            return true;
        }
    }

    return false;
}

bool LangUnderstandingTasks::parseSentence(std::string sentenceToParse, 
        CommandFrame &parseResult)
{
    /**
     * Create a client object to call the ROS service
     */
	ros::NodeHandle nodeHandler;
	ros::ServiceClient client = nodeHandler.serviceClient
        <planning_msgs::parse_sentence_cfr>(m_parseSentenceServName);

    /**
     * Create a srv object to send the request to the ROS service
     */
	planning_msgs::parse_sentence_cfr srv;
	srv.request.sentence = sentenceToParse;
	
    /**
     * Call the ROS service
     */
	if(client.call(srv))
	{
		/**
         * Verify if the sentence was succesfully parsed
         */
		if(srv.response.cfr.command.compare("NO_INTERPRETATION") != 0)
			return true;

        /**
         * To store the command an parameters resulting from the parsing.
         */
        std::string resultCommand;
        std::map<std::string, std::string> resultParams;

        /**
         * Storing the parameters resulting from the parsing.
         */
        resultCommand = srv.response.cfr.command;
        for(int i=0; i<srv.response.cfr.frame_id.size(); i++)
        {
            resultParams.insert(std::pair<std::string, std::string> (
                        srv.response.cfr.frame_id[i], 
                        srv.response.cfr.frame_value[i]
                        ));
        }
        parseResult.command = resultCommand;
        parseResult.params = resultParams;

	}

	return false;
}
