#include "simple_task_planner/simpletasks.h"

SimpleTasks::SimpleTasks()
{
}

bool SimpleTasks::askAndWaitForConfirm(std::string t_questionToAsk, 
        int t_timeout, int t_repeatTimeout)
{
    using namespace boost::chrono;

    //start to listen voic commands
    m_sprec.startListening();

    //Loop to wait for the user's response until timeout
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();

    while(ros::ok() && millisElapsed.count() < t_timeout
            && !m_sprec.isSentenceRecognized())
    {
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeout == 0)
        {
    		m_spgenTasks.asyncSpeech(t_questionToAsk);
        }
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }
    //disable the speech recognition listen mode
    m_sprec.stopListening();

    //verify if the time out was reached
    if(millisElapsed.count()>=t_timeout || !ros::ok())
    {
        return false;
    }
    //parse the recognized sentence
    return m_langundTasks.isPositiveUserConfirmation(
            m_sprec.getLastRecognizedSentence());
}

bool SimpleTasks::waitForStartFollowCommand(std::string t_sentenceToRepeat, 
        std::string &t_goalToFollow, int t_timeout, int t_repeatTimeout)
{
    using namespace boost::chrono;
    m_sprec.startListening();
    
    //Loop to wait for the user's start command
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();
    while(ros::ok() && millisElapsed.count() < t_timeout)
    {
        //sentence heard
        if(m_sprec.isSentenceRecognized())
        {
            if(m_langundTasks.isStartFollowInstruction(
                        m_sprec.getLastRecognizedSentence(), t_goalToFollow))
            {
                m_sprec.stopListening();
                return true;
            }
            m_sprec.stopListening();
            m_sprec.startListening();
        }
        
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeout == 0)
        {
    		m_spgenTasks.asyncSpeech(t_sentenceToRepeat);
        }
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }

    m_sprec.stopListening();
    t_goalToFollow = "";

    return false;
}

bool SimpleTasks::waitForCommand(
        std::string &t_command,
        std::map<std::string, std::string> &t_params, 
        int t_timeout
        )
{
    using namespace boost::chrono;
    m_sprec.startListening();
    
    //Loop to wait for the user's start command
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();
    while(ros::ok() && millisElapsed.count() < t_timeout)
    {
        //sentence heard
        if(m_sprec.isSentenceRecognized())
        {
            if(m_langundTasks.isValidCommand(
                        m_sprec.getLastRecognizedSentence(),
                        t_command,
                        t_params))
            {
                m_sprec.stopListening();
                return true;
            }
            m_sprec.stopListening();
            m_sprec.startListening();
        }
        
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }

    m_sprec.stopListening();
    t_command = "";

    return false;
}
