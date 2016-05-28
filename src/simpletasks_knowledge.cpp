#include "simple_task_planner/simpletasks.h"


bool SimpleTasks::faceSearchAndRemeber(std::string t_findInstructions, 
        std::string t_faceID, 
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
        
        //wait for the head to reach the position (just a timer not a topic 
        //update)
        headWaitDuration.sleep();

        //Try to train the face on the current head position
        if((faceTrained = m_recoFaces.trainFace(t_faceID, 3000)))
        {
            m_spgenTasks.syncSpeech("human, smile", 5000);
            m_recoFaces.trainFace(t_faceID, 3000);
            m_spgenTasks.syncSpeech("now, a serious face", 5000);
            m_recoFaces.trainFace(t_faceID, 3000);
            m_spgenTasks.syncSpeech("I have remembered your face", 5000);
        }
        ros::spinOnce();
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
