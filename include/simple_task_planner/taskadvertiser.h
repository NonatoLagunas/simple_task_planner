/**
 * @class TaskAdvertiser
 * @brief Class to advertise the simple tasks as ROS services.
 *
 * This class use the tasks libraries in order to advertise each (ideally) to
 * ROS services. For this, this clase uses the needed justina messages packages
 * such as: planning_msgs, hri_msgs, etc.
 *
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_TASKADVERTISER_H_
#define _JUSTINA_TASKADVERTISER_H_

#include "simple_task_planner/simpletasks.h"
#include "planning_msgs/wait_for_confirm.h"
#include "planning_msgs/wait_for_start_follow.h"
#include "planning_msgs/wait_for_command.h"
#include "ros/ros.h"

class TaskAdvertiser
{
    private:

        ros::NodeHandle m_nh; /**< ROS node handler. */
        
        bool m_isAdvertising; /**< Indicates if the tasks are advertised. **/

        SimpleTasks m_simpleTasks; /**< To call the simple tasks. */

        ros::ServiceServer m_askAndWaitForConfirmSrv; /**< ROS object to 
                                                        advertise the wait for 
                                                        confirmation task as a 
                                                        service. */

        ros::ServiceServer m_waitStartFollowSrv; /**< ROS object to advertise 
                                                   the wait for start follow 
                                                   task as a service. */

        ros::ServiceServer m_waitForCommandSrv; /**< ROS object to advertise 
                                                   the wait for command 
                                                   task as a service. */

        bool askAndWaitForConfirmCallback(
                planning_msgs::wait_for_confirm::Request &req,
                planning_msgs::wait_for_confirm::Response &resp
                );

        bool waitStartFollowCallback(
                planning_msgs::wait_for_start_follow::Request &req,
                planning_msgs::wait_for_start_follow::Response &resp
                );

        bool waitForCommandCallback(
                planning_msgs::wait_for_command::Request &req,
                planning_msgs::wait_for_command::Response &resp
                );
    public:

        TaskAdvertiser(
                ros::NodeHandle nh = ros::NodeHandle("simple_task_planner")
                );

        /**
         * @brief Advertise the simple tasks as services.
         */
        void startAdvertising();

};

#endif
