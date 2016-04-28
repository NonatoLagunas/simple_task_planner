#include "speechgeneratortasks.h"

SpeechGeneratorTasks::SpeechTasks(std::string syncSpeechServiceName="/spg_say",
        std::string asyncSpeechServiceName="/spg_say")
{
    m_syncSeechServName = syncSpeechServiceName;
    m_asyncSpeechServName = asyncSpeechServiceName;
}

bool SpeechGeneratorTasks::syncSpeech(std::string textToSpeech, int timeOut)
{
    ros::NodeHandle nodeHandler;
    ros::ServiceClient client = nodeHandler.serviceClient
        <bbros_bridge::Default_ROS_BB_Bridge>(m_syncSpeechServiceName);

    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = textToSpeech;
    srv.request.timeout = timeOut;

    if(client.call(srv))
    {
        return true;
    }
    return false;
}

bool SpeechGeneratorTasks::asyncSpeech(std::string textToSpeech)
{
    ros::NodeHandle nodeHandler;
    ros::ServiceClient client = nodeHandler.serviceClient
        <bbros_bridge::Default_ROS_BB_Bridge>(m_asyncSpeechServiceName);

    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = textToSpeech;
    srv.request.timeout = 0;

    if(client.call(srv))
    {
        return true;
    }
    return false;
}
