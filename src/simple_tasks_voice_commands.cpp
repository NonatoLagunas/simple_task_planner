#include "simple_task_planner/simple_tasks.h"

SimpleTasks::SimpleTasks()
{
}

bool SimpleTasks::askAndWaitForConfirm(std::string questionToAsk, int timeOut, 
        int repeatTimeOut)
{
    SpeechGeneratorTasks m_spgenTasks;
    LangUnderstandingTasks m_langundTasks;

    /**
     * Ask the question to the user.
     */
    //m_spgenTasks.asyncSpeech(questionToAsk);

    /**
     * Loop to wait for the user's response until the timeout reached.
     */
    using namespace std::chrono;
    steady_clock::time_point taskStartTimePoint = steady_clock::now();
    milliseconds millisElapsed = duration_cast<milliseconds> (
            taskStartTimePoint - taskStartTimePoint
            );
    std::cout << "start " << std::endl;
    while(millisElapsed.count() < repeatTimeOut)
    {
        /**
         * Repeat the the question to the user again every 5 seconds.
         */
        //if(t2 - t1 >=5s)
        //{
        //    m_spgenTasks.asyncSpeech(questionToAsk);
        //}
        steady_clock::time_point taskCurrentTimePoint = steady_clock::now();
        millisElapsed = duration_cast<milliseconds>(
                taskCurrentTimePoint - taskStartTimePoint
                );
    }
    std::cout << "end " << std::endl;

    return true;
}
