#include "simple_task_planner/langunderstandingtasks.h"
#include "simple_task_planner/speechgeneratortasks.h"
#include "ros/ros.h"
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_task_planner");
    ros::NodeHandle nodeHandler;
    SpeechGeneratorTasks spgExecuter;
    LangUnderstandingTasks langUndExecuter;

    std::string parseResult;
    std::string input("robot follow me");
    langUndExecuter.parseSentence(input, parseResult);
    std::cout << "P result: " << parseResult << std::endl;
    //spgExecuter.syncSpeech("sync test", 5000);

    //spgExecuter.asyncSpeech("async test");
}
