#include "robot_service_manager/speechrecognitionstatus.h"
#include "robot_service_manager/langunderstandingtasks.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/headstatus.h"
#include "simple_task_planner/simpletasks.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"
#include "ros/ros.h"
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_task_planner");
    ros::NodeHandle nodeHandler;
    std::cout << "sub 1 " << std::endl;
    HeadStatus hdStatus(&nodeHandler);
    std::cout << "sub 2 " << std::endl;
    HeadStatus hdStatus2(&nodeHandler);
    SpeechGeneratorTasks spgExecuter;
    SimpleTasks st;
    if(st.askAndWaitForConfirm("",10000))
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
