#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goals");

	MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
		ROS_INFO("Waiting for the move_base action server");
	}

    while(1)
    {
//        double * p3=new double [5];
//        p3[0]=0.0;
//        p3[1]=1.0;
//        p3[2]=2.0;
//        p3[3]=3.0;
//        p3[4]=4.0;

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 1.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal 1");
        ac.sendGoal(goal);
        ac.waitForResult();

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 2.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal 2");
        ac.sendGoal(goal);
        ac.waitForResult();

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 3.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal 3");
        ac.sendGoal(goal);
        ac.waitForResult();

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 4.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal 4");
        ac.sendGoal(goal);
        ac.waitForResult();

//        delete [] p3;

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have arrived to the goal position");
        else
        {
            ROS_INFO("The base failed for some reason");
        }

     }

	return 0;
}
