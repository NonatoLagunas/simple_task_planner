#include "simple_task_planner/simpletasks.h"


bool SimpleTasks::faceSearchAndRemeber(std::string t_findInstructions, 
        std::string t_faceName, 
        std::vector<std::pair<float, float> > t_headMovements)
{
    using namespace std;

    //setting default parameters
    if(t_headMovements.size()==0)
    {
        initializeHeadVector(t_headMovements);
    }
    t_findInstructions = (t_findInstructions=="default") ? 
        "Please look straight to my kinect camera" : t_findInstructions;

    //Instruct to the human
    m_spgenTasks.syncSpeech(t_findInstructions, 10000);

    //search for the human face by moving the head to different positions
    bool faceTrained=false;
    ros::Duration headWaitDuration(3);
    for(int currentHeadPos=0; 
            currentHeadPos<t_headMovements.size() && !faceTrained;
            ++currentHeadPos)
    {
        //move the head to the current position of the list
        m_headStatus.setHeadPose(
                t_headMovements[currentHeadPos].first,
                t_headMovements[currentHeadPos].second
                );
        ros::spinOnce();
        
        //wait for the head to reach the position 
        headWaitDuration.sleep();

        //Try to detect train the face on the current head position
        //if(faceTrained = m_recoFaces.train(t_faceName, 10))
        //{
        //m_spenTasks.syncSpeech("human, smile", 5000);
        //m_recoFaces.train(t_faceName, 10))
        //ros::SpinOnce();
        //m_spenTasks.syncSpeech("now, a serious face", 5000);
        //m_recoFaces.train(t_faceName, 10))
        //ros::SpinOnce();
        //m_spenTasks.syncSpeech("i have remembered your face", 5000);
        //}
        //ros::SpinOnce();
        //if(m_recoFaces.trainFace()) break;
    }
    
    return faceTrained;
}

void SimpleTasks::initializeHeadVector(
        std::vector<std::pair<float, float> > &t_vector)
{
    using namespace std;

    t_vector.clear();
    t_vector.push_back(pair<float, float>(0.0, 0.0));
    t_vector.push_back(pair<float, float>(-0.4, 0.0));
    t_vector.push_back(pair<float, float>(0.4, 0.0));
    t_vector.push_back(pair<float, float>(0.0, -0.4));
    t_vector.push_back(pair<float, float>(-0.4, -0.4));
    t_vector.push_back(pair<float, float>(0.4, -0.4));
    t_vector.push_back(pair<float, float>(0.0, 0.4));
    t_vector.push_back(pair<float, float>(-0.4, 0.4));
    t_vector.push_back(pair<float, float>(0.4, 0.4));
    t_vector.push_back(pair<float, float>(0.0, 0.0));
}
