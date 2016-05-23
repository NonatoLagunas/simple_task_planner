/**
 * @class TasksAdvertiser
 * @brief  Class for advertise all the simple tasks to ros services.
 *
 * This class use the tasks libraries in order to advertise each (ideally) to
 * ROS services. For this, this clase uses the needed justina messages packages
 * such as: planning_msgs, hri_msgs, etc.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_TASKSADVERTISER_H
#define _JUSTINA_TASKSADVERTISER_H

#include "ros/ros.h"
#include "simple_task_planner/simple_tasks.h"

class SimpleTasks
{
    private:
        SimpleTasks m_simpleTasks;

        bool askAndWaitForConfirm

    public:
        /**
         * @brief Class constructor.
         * 
         * Creates a new TasksAdvertiser object.
         */
        TasksAdvertiser();

};
#endif
