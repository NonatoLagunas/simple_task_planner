#include "simple_task_planner/speechrecognitionstatus.h"
#include "simple_task_planner/langunderstandingtasks.h"
#include "simple_task_planner/speechgeneratortasks.h"
#include "simple_task_planner/headstatus.h"
#include "ros/ros.h"
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_task_planner");
    ros::NodeHandle nodeHandler;
    std::cout << "sub 1 " << std::endl;
    HeadStatus hdStatus(nodeHandler);
    std::cout << "sub 2 " << std::endl;
    HeadStatus hdStatus2(nodeHandler);
    SpeechRecognitionStatus spStatus(nodeHandler);
    SpeechGeneratorTasks spgExecuter;
    LangUnderstandingTasks langUndExecuter;

    std::string parseResult;
    std::string input("robot follow me");
    langUndExecuter.parseSentence(input, parseResult);
    std::cout << "P result: " << parseResult << std::endl;
    //spgExecuter.syncSpeech("sync test", 5000);
    ros::spin();

    //spgExecuter.asyncSpeech("async test");
}
