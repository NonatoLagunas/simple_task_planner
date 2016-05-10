#include "justina_tools/speechrecognitionstatus.h"
#include "justina_tools/langunderstandingtasks.h"
#include "justina_tools/speechgeneratortasks.h"
#include "justina_tools/headstatus.h"
#include "simple_task_planner/simple_tasks.h"
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
    SpeechRecognitionStatus spStatus(&nodeHandler);
    SpeechGeneratorTasks spgExecuter;
    SimpleTasks st;
    st.askAndWaitForConfirm("",10000, 15000);
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
}
