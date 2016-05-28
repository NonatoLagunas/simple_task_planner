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
    m_rememberFaceSrv = m_nh.advertiseService("search_and_remember_face", 
            &TaskAdvertiser::rememberFaceCallback, this);

    m_askAndWaitForConfirmSrv = m_nh.advertiseService("wait_for_confirm", 
            &TaskAdvertiser::askAndWaitForConfirmCallback, this);

    m_waitStartFollowSrv = m_nh.advertiseService("wait_for_start_follow", 
            &TaskAdvertiser::waitStartFollowCallback, this);

    m_waitForCommandSrv = m_nh.advertiseService("wait_for_command", 
            &TaskAdvertiser::waitForCommandCallback, this);

    m_waitForStartGuideSrv = m_nh.advertiseService("wait_for_start_guide", 
            &TaskAdvertiser::waitForStartGuideCallback, this);
}

bool TaskAdvertiser::rememberFaceCallback( 
        planning_msgs::search_remember_face::Request &req,
        planning_msgs::search_remember_face::Response &resp
        )
{
    std::vector<std::pair<float, float> > headMovements;
    for(int currentHeadMov=0; currentHeadMov<req.head_movs.size();
            currentHeadMov++)
    {
        headMovements.push_back(std::pair<float, float>(
                    req.head_movs[currentHeadMov].data[0],
                    req.head_movs[currentHeadMov].data[1]
                    ));
    }
    resp.training_success = m_simpleTasks.faceSearchAndRemeber(
            req.robot_instructions, 
            req.face_id, 
            headMovements);

    return true;
}

bool TaskAdvertiser::waitForCommandCallback(
        planning_msgs::wait_for_command::Request &req,
        planning_msgs::wait_for_command::Response &resp
        )
{
    std::map<std::string, std::string> params;
    resp.command_received = m_simpleTasks.waitForCommand(resp.cfr.command, 
            params, req.timeout);

    planning_msgs::CFRParams currentParam;
    std::map<std::string, std::string>::iterator it;
    for(it=params.begin(); it != params.end(); ++it)
    {
        currentParam.frame_id = it->first;
        currentParam.frame_value = it->second;
        resp.cfr.params.push_back(currentParam);
    }

    return true;
}

bool TaskAdvertiser::askAndWaitForConfirmCallback( 
        planning_msgs::wait_for_confirm::Request &req,
        planning_msgs::wait_for_confirm::Response &resp
        )
{
    resp.confirmation_received = m_simpleTasks.askAndWaitForConfirm(
            req.repeat_sentence.sentence, req.timeout, 
            req.repeat_sentence.repeat_time
            );

    return true;
}

bool TaskAdvertiser::waitForStartGuideCallback(
        planning_msgs::wait_for_switch::Request &req,
        planning_msgs::wait_for_switch::Response &resp
        )
{
    resp.command_received = m_simpleTasks.waitForStartGuideCommand(
            req.repeat_sentence.sentence, resp.goal, req.timeout, 
            req.repeat_sentence.repeat_time
            );

    return true;
}
bool TaskAdvertiser::waitStartFollowCallback( 
        planning_msgs::wait_for_switch::Request &req,
        planning_msgs::wait_for_switch::Response &resp
        )
{

    resp.command_received = m_simpleTasks.waitForStartFollowCommand(
            req.repeat_sentence.sentence, resp.goal, req.timeout, 
            req.repeat_sentence.repeat_time
            );

    return true;
}
