LangUnderstandingTasks::LangUnderstandingTasks(
        std::string parseSentenceServName) : m_parseSentenceServName(
            parseSentenceServName)
{
}

bool LangUnderstandingTasks::LangUnparseSentence(std::string sentenceToParse, 
        std::string &parseResult)
{
	ros::NodeHandle nodeHandler;
	ros::ServiceClient client = n.serviceClient
        <language_understanding::parse_sentence>(m_parseSentenceServName);

	language_understanding::parse_sentence srv;
	srv.request.sentence = sentenceToParse;
	
	if(client.call(srv))
	{
		parseResult = srv.response.conceptual_dependency;
		/*verify if the sentence was succesfully parsed*/
		if(srv.response.conceptual_dependency.compare("not_parsed") != 0)
			return true;
	}

	return false;
}
