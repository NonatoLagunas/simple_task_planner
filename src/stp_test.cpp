#include "simple_task_planner/speechgeneratortasks.h"
#include "ros/ros.h"
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_task_planner");
    ros::NodeHandle nodeHandler;
    SpeechGeneratorTasks spgExecuter;

    spgExecuter.syncSpeech("sync test", 5000);

    spgExecuter.asyncSpeech("async test");
}
