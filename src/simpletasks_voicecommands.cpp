#include "simple_task_planner/simpletasks.h"

SimpleTasks::SimpleTasks()
{
}

bool SimpleTasks::askAndWaitForConfirm(std::string t_questionToAsk, 
        int t_timeOut, int t_repeatTimeOut)
{
    SpeechGeneratorTasks m_spgenTasks;
    LangUnderstandingTasks m_langundTasks;

    m_sprec.startListening();
    //Loop to wait for the user's response until the timeout reached.
    using namespace boost::chrono;
    steady_clock::time_point taskStartTime = steady_clock::now();
    milliseconds millisElapsed;
    while(ros::ok() && millisElapsed.count() < t_timeOut
            && !m_sprec.isSentenceRecognized())
    {
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeOut == 0)
        {
    		 // Ask the question to the user.
    		m_spgenTasks.asyncSpeech(t_questionToAsk);
        }
        steady_clock::time_point taskCurrentTime = steady_clock::now();
        millisElapsed = duration_cast<milliseconds>(
                taskCurrentTime - taskStartTime
                );
        ros::spinOnce();

    }
    m_sprec.stopListening();

    //verify if the time out was reached
    if(millisElapsed.count()>=t_timeOut || !ros::ok())
    {
        return false;
    }
    //parse the recognized sentence
    return m_langundTasks.isPositiveUserConfirmation(
            m_sprec.getLastRecognizedSentence());
}
