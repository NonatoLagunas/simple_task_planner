#include "simple_task_planner/taskadvertiser.h"

TaskAdvertiser::TaskAdvertiser(ros::NodeHandle t_nh) :
    m_nh(t_nh)
{
    if(ros::isInitialized())
    {
        //advertise all the services
        startAdvertising();
    }
}

void TaskAdvertiser::startAdvertising()
{
    m_askAndWaitForConfirmSrv = m_nh.advertiseService("wait_for_confirmation", 
            &TaskAdvertiser::askAndWaitForConfirmCallback, this);

    m_waitStartFollowSrv = m_nh.advertiseService("wait_for_start_follow", 
            &TaskAdvertiser::waitStartFollowCallback, this);

    m_waitForCommandSrv = m_nh.advertiseService("wait_for_command", 
            &TaskAdvertiser::waitForCommandCallback, this);
}

bool TaskAdvertiser::waitForCommandCallback(
        planning_msgs::wait_for_command::Request &req,
        planning_msgs::wait_for_command::Response &resp
        )
{
    return true;
}

bool TaskAdvertiser::askAndWaitForConfirmCallback( 
        planning_msgs::wait_for_confirm::Request &req,
        planning_msgs::wait_for_confirm::Response &resp
        )
{
    return resp.confirmation_received = m_simpleTasks.askAndWaitForConfirm(
            req.repeat_sentence.sentence, req.timeout, 
            req.repeat_sentence.repeat_time
            );
}

bool TaskAdvertiser::waitStartFollowCallback( 
        planning_msgs::wait_for_start_follow::Request &req,
        planning_msgs::wait_for_start_follow::Response &resp
        )
{

    return resp.confirmation_received = m_simpleTasks.waitForStartFollowCommand(
            req.repeat_sentence.sentence, resp.goal_to_follow, req.timeout, 
            req.repeat_sentence.repeat_time
            );
}
