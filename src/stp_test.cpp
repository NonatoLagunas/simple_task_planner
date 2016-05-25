#include "robot_service_manager/speechrecognitionstatus.h"
#include "robot_service_manager/langunderstandingtasks.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/headstatus.h"
#include "simple_task_planner/simpletasks.h"
#include "simple_task_planner/taskadvertiser.h"
#include "ros/ros.h"
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_task_planner");
    ros::NodeHandle nodeHandler("simple_task_planner");

    TaskAdvertiser ta(nodeHandler);
    SimpleTasks st;
    std::string command;
    std::map<std::string, std::string> par;
    if(st.waitForCommand(command, par, 10000))
    {
        std::cout << "command " << command << std::endl;
    }
    else
    {
        std::cout << "nothing" << std::endl;
    }

    ros::spin();
    
    std::cout << "sub 1 " << std::endl;
    HeadStatus hdStatus(&nodeHandler);
    std::cout << "sub 2 " << std::endl;
    HeadStatus hdStatus2(&nodeHandler);
    SpeechGeneratorTasks spgExecuter;
    std::string goalToFollow;
    if(st.waitForStartFollowCommand("i am waiting for the start command", goalToFollow, 10000))
    {
        std::cout << "goal to follow: " << goalToFollow << std::endl;
    }
    else
    {
        std::cout << "no command received" << std::endl;
    }
    if(st.askAndWaitForConfirm("waiting for confirmation",10000))
    {
        std::cout << "robot yes received" << std::endl;
    }
    else
    {
        std::cout << "no confirmation" << std::endl;
    }
    //st.askAndWaitForConfirm("",10000,5000);

    LangUnderstandingTasks lang;

    std::string goal;
    if(lang.isFollowGoalInstruction("robot follow me", goal))
        std::cout << "follow command goal: " << goal << std::endl;
    if(lang.isFollowGoalInstruction("robot follow the wall", goal))
        std::cout << "follow command goal: " << goal << std::endl;
    if(lang.isFollowGoalInstruction("robot follow ", goal))
        std::cout << "follow command goal: " << goal << std::endl;
    if(lang.isFollowGoalInstruction("sdsfw", goal))
        std::cout << "follow command goal: " << goal << std::endl;
    if(lang.isFollowGoalInstruction("robot bring me the mayo", goal))
        std::cout << "follow command goal: " << goal << std::endl;
    if(lang.isPositiveUserConfirmation("robot yes"))
        std::cout << "robot yes  " << std::endl;
    if(lang.isPositiveUserConfirmation("robot no"))
        std::cout << "robot no " << std::endl;
    if(lang.isPositiveUserConfirmation("edsf"))
        std::cout << "edsf" << std::endl;
    if(lang.isPositiveUserConfirmation("robot brint s"))
        std::cout << "robot brint s" << std::endl;

    if(lang.isStartFollowInstruction("robot start follow me", goal))
        std::cout << "start follow command goal: " << goal << std::endl;
    if(lang.isStartFollowInstruction("robot follow the wall", goal))
        std::cout << "start follow command goal: " << goal << std::endl;
    if(lang.isStopFollowInstruction("robot stop follow me", goal))
        std::cout << "stop follow command goal: " << goal << std::endl;
    if(lang.isStopFollowInstruction("robot stop follow the wall", goal))
        std::cout << "stop follow command goal: " << goal << std::endl;
    //spgExecuter.syncSpeech("sync test", 5000);
    //ros::spin();

    //spgExecuter.asyncSpeech("async test");
    
    //create a ros topic for each stp task

}
