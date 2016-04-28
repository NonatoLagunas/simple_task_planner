#ifndef SPEECHTASKS
#define SPEECHTASKS
#include <string>
#include "ros/ros.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
class SpeechTasks
{
	public:
		SpeechTasks();
	private:
		bool syncSpeech(std::string textToSpeech, int);
		void asyncSpeech(std::string textToSpeech);
};
#endif
