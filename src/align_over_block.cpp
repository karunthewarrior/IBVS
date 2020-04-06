#include <mbz2020_ibvs/AlignOverBlockAction.h>
#include <mbz2020_ibvs/ibvs.h>
#include <actionlib/server/simple_action_server.h>
#include <signal.h>

//This file implements the align over block action server.

typedef actionlib::SimpleActionServer<mbz2020_ibvs::AlignOverBlockAction> Server;

void append_to_convergence_list(std::vector<ros::Time>& convergence_list, ros::Time t){
    if(convergence_list.size() == 2){
        convergence_list.pop_back();
        convergence_list.push_back(t);
    }
    else 
        convergence_list.push_back(t);
}

void execute(const mbz2020_ibvs::AlignOverBlockGoalConstPtr& goal, Server* as, ros::NodeHandle* n)
{
    mbz2020_ibvs::AlignOverBlockResult result;
    
    int convergence_time_desired, loop_rate_desired, control_mode, timeout;
    double convergence_threshold;
    std::vector<double> camera_info;

    n->getParam("ibvs/loop_rate", loop_rate_desired);
    n->getParam("ibvs/control_mode", control_mode);
    ROS_INFO("CONTROL MODE: %d", control_mode);

    mode m;
    if (goal->execution_mode.data == "DRONE_PICKUP"){
        m = DRONE_PICKUP;
        n->getParam("ibvs/drone_pickup/convergence_threshold", convergence_threshold);
        n->getParam("ibvs/drone_pickup/convergence_time", convergence_time_desired);
        n->getParam("ibvs/drone_pickup/camera_info", camera_info);
        n->getParam("ibvs/drone_pickup/timeout", timeout);
    }
    else if(goal->execution_mode.data == "DRONE_PLACEMENT"){
        m = DRONE_PLACEMENT;
        n->getParam("ibvs/drone_placement/convergence_threshold", convergence_threshold);
        n->getParam("ibvs/drone_placement/convergence_time", convergence_time_desired);
        n->getParam("ibvs/drone_placement/camera_info", camera_info);
        n->getParam("ibvs/drone_placement/timeout", timeout);
    }
    else{
        ROS_INFO("INVALID EXECUTION MODE- check header file for modes");
        result.is_converged = false;
        as->setSucceeded(result);
        return;
    }
    ROS_INFO("Creating IBVS");

    IBVS* ibvs_obj = new IBVS(*n, m);
    
    if(goal->z.data.size() != ibvs_obj->num_image_points_ ||  goal->corners.data.size() != ibvs_obj->num_image_points_*2)
    {
        std::cout<<"BAD DATA"<<std::endl;
        result.is_converged = false;
        as->setSucceeded(result);
        return;
    }

    // TODO: Make function for this block
    //----------------------------------------
    Eigen::MatrixXd fd(2,ibvs_obj->num_image_points_);
    std::vector<double> zd;
    for(int i = 0; i < ibvs_obj->num_image_points_; i++)
    {
        fd(0,i) = (goal->corners.data[i] -  camera_info[2]) / camera_info[0];
        zd.push_back(goal->z.data[i]);
    }
    for(int i = ibvs_obj->num_image_points_; i < ibvs_obj->num_image_points_*2; i++)
    {
        fd(1,i-ibvs_obj->num_image_points_) = (goal->corners.data[i] - camera_info[5]) / camera_info[4];
    }
    //----------------------------------------

    std::cout<<fd<<std::endl;
    ros::Rate loop_rate(loop_rate_desired);
    Eigen::VectorXd x;
    ros::Time start_time = ros::Time::now();
    std::vector<ros::Time> convergence_list;
    ros::Time sleep_time;
    ros::Duration convergence_time = ros::Duration(0);
    mbz2020_ibvs::AlignOverBlockFeedback feedback;
    ibvs_obj->set_goal_state(fd, zd);
    while (ros::ok())
    {
        if (as->isPreemptRequested())
        {
            ROS_INFO("CANCEL REQUESTED");
            ibvs_obj->publish_velocity(Eigen::VectorXd::Zero(4));
            result.is_converged = false;
            as->setSucceeded(result);
            break;
        }
        x = ibvs_obj->compute_velocity(control_mode);
        //ROS_INFO_THROTTLE(0.1, "Velocity: x: %f, y: %f, z: %f, yaw: %f", x(0), x(1), x(2), x(5));
        ibvs_obj->publish_velocity(x);
        
        feedback.error.data = ibvs_obj->error_norm_;

        as->publishFeedback(feedback);

        ros::spinOnce();
        if((ros::Time::now() - start_time) > ros::Duration(timeout)){
                std::cout<<(ros::Time::now() - start_time)<<"YAA"<<ros::Duration(timeout)<<std::endl;
		result.is_converged = false;
        	as->setSucceeded(result);
        	ROS_INFO("TIMEOUT");
		break;
	}
        
        if (ibvs_obj->check_convergence(convergence_threshold))
        {
            append_to_convergence_list(convergence_list, ros::Time::now());
            if (convergence_list.size() == 2){
                convergence_time = convergence_list[1] - convergence_list[0];
            }
            if (convergence_time > ros::Duration(convergence_time_desired)){
                result.is_converged = true;
                as->setSucceeded(result);
                ROS_INFO("CONVERGED");
                sleep_time = ros::Time::now();

                // Sleeping 1 second before exiting servoing action
                while (ros::ok() && ((ros::Time::now() - sleep_time) < ros::Duration(0.2))){
                    ibvs_obj->publish_velocity(Eigen::VectorXd::Zero(6));
                }
                break;
            }
        }
        else
        {
        	convergence_list.clear();
        }
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "align_over_block_server");
    ros::NodeHandle n;
    mbz2020_ibvs::AlignOverBlockGoal goal;
    Server server(n, "align_over_block", boost::bind(&execute, _1, &server, &n), false);
    server.start();
    ros::spin();
    return 0;
}
