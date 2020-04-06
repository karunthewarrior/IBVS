#include <mbz2020_ibvs/ibvs.h>
#include <geometry_msgs/TwistStamped.h>
#include <mbz2020_custom_messages/Int32MultiArrayStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <Eigen/QR>
#include <cmath>

IBVS::IBVS(ros::NodeHandle& nh, mode m){
    nh_ = nh;
    update_execution_mode(m);
    std::string z_estimate_topic, velocity_command_topic;
    int loop_rate_desired;
    nh.getParam("ibvs/z_estimate_topic", z_estimate_topic);
    nh.getParam("ibvs/loop_rate", loop_rate_desired);
    nh.getParam("ibvs/velocity_command_topic", velocity_command_topic);
    ROS_INFO("Topic %s",ibvs_corners_topic_);

    // Intiliazing gain- First 3 values for xyz and last for yaw
    K_ = Eigen::VectorXd::Zero(4);
    for (int i = 0; i < 3; i++){
        K_(i) = k_[0];
    }
    K_(3) = k_[1];

    L_ = Eigen::MatrixXd::Zero(num_image_points_*2,4);
    L_star_ = Eigen::MatrixXd::Zero(num_image_points_*2,4);

    fd_.setZero();

    f_.setZero();
    z_estimate_ = std::vector<double>(0);
    
    corners_sub_ = nh_.subscribe(ibvs_corners_topic_, 1, &IBVS::corners_callback, this);
    z_sub_ = nh_.subscribe(z_estimate_topic, 1, &IBVS::z_estimate_callback, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(velocity_command_topic, loop_rate_desired);
}

Eigen::MatrixXd IBVS::generate_interaction_matrix_block(const double u, const double v, const double z){
    Eigen::MatrixXd L(2,4);
    L << 1/z,0,-u/z,-v,0,1/z,-v/z,u;
    return L;
}

Eigen::MatrixXd IBVS::generate_full_interaction_matrix(const Eigen::MatrixXd& corners_mat, const std::vector<double>& z){
    std::vector<Eigen::MatrixXd> l;
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(num_image_points_*2,4);
    for(int i = 0; i < num_image_points_; i++)
    {
        L.block(i*2,0,2,4) = generate_interaction_matrix_block(corners_mat(0, i), corners_mat(1, i), z[i]);
    }
    return L;
}

void IBVS::corners_callback(const mbz2020_custom_messages::Int32MultiArrayStampedConstPtr& corners){
    Eigen::MatrixXd c(2,num_image_points_);
    if (corners->array.data.size() == 2*num_image_points_){
        corners_stamp_ = corners->header.stamp;
        for(int i = 0; i < num_image_points_; i++)
        {
            c(0,i) = (corners->array.data[i] - camera_info_[2]) / camera_info_[0];
        }
        for(int i = num_image_points_; i < num_image_points_*2; i++)
        {
            c(1,i-num_image_points_) = (corners->array.data[i] - camera_info_[5]) / camera_info_[4];
        }
        f_ = c;
    }
    else
    {
        ROS_WARN_THROTTLE(0.5, "WRONG NUMBER OF CORNER POINTS");
    }
}

void IBVS::z_estimate_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
    z_estimate_.clear();
    sf30_stamp_ = msg->header.stamp;
    for(size_t i = 0; i < num_image_points_; i++)
    {
        z_estimate_.push_back(msg->pose.pose.position.z + block_zoffset_);   
    }
}

void IBVS::set_goal_state(const Eigen::MatrixXd& fd, const std::vector<double>& z){
    fd_ = fd;
    L_star_ = generate_full_interaction_matrix(fd_, z);
}

Eigen::VectorXd IBVS::compute_velocity(int mode){
    Eigen::VectorXd y = Eigen::VectorXd::Zero(4);
    bool is_empty = f_.isZero(0);
    if(!is_empty && z_estimate_.size() != 0)
    {
        if ((ros::Time::now() - corners_stamp_ < ros::Duration(0.5)) &&
            (ros::Time::now() - sf30_stamp_ < ros::Duration(0.5)))
        {
            L_ = generate_full_interaction_matrix(f_, z_estimate_);
            if(mode == 1) // mode 1 is combined control_mode
                {
                L_ = (L_ + L_star_)/2;
                }
            error_ = fd_- f_;
            Eigen::Map<Eigen::VectorXd> error_vec(error_.data(), (1,num_image_points_*2));
            Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(L_);
            Eigen::MatrixXd L_inv = cod.pseudoInverse();
            y = K_.cwiseProduct(L_inv * error_vec);

            for(int i = 0; i < 4; i++){
                y(i) = clip(y(i),-clip_threshold_[i], clip_threshold_[i]);
            }
        }
        else
        {
            ROS_WARN_THROTTLE(0.5, "CORNER DETECTION OR LASER STOPPED WORKING - CHECK SENSORS");
            return y;
        }
    }
    else
        ROS_WARN_THROTTLE(0.5, "NO INPUTS FROM CORNERS OR Z ESTIMATE FROM LASER - CHECK SENSORS");
    return y;
}

bool IBVS::check_convergence(double value){
    bool is_empty = f_.isZero(0);
    if(!is_empty && z_estimate_.size() != 0){
        error_norm_ = error_.squaredNorm();
	    if (error_norm_ > value){
                return false;
            }
    }
    else{
        return false;
	}
    return true;
}

void IBVS::publish_velocity(const Eigen::VectorXd velocity){
    geometry_msgs::TwistStamped velocity_msg;
    velocity_msg.header.frame_id = "base_link";
    if (execution_mode_ == DRONE_PICKUP){
        velocity_msg.twist.linear.x = velocity(1);
        velocity_msg.twist.linear.y = velocity(0);
        velocity_msg.twist.linear.z = velocity(2);
        velocity_msg.twist.angular.z = velocity(3);
    }
    else if(execution_mode_ == DRONE_PLACEMENT){ 
        velocity_msg.twist.linear.x = -velocity(0);
        velocity_msg.twist.linear.y = velocity(1);
        velocity_msg.twist.linear.z = velocity(2);
        velocity_msg.twist.angular.z = velocity(3);
    }
    velocity_pub_.publish(velocity_msg);
}

void IBVS::update_execution_mode(mode m){
    execution_mode_ = m;
    error_norm_ = INT_MAX;
    if (execution_mode_ == DRONE_PICKUP){
        nh_.getParam("ibvs/drone_pickup/corners_topic", ibvs_corners_topic_);
        nh_.getParam("ibvs/drone_pickup/num_image_points", num_image_points_);
        nh_.getParam("ibvs/drone_pickup/camera_info", camera_info_);
        nh_.getParam("ibvs/drone_pickup/clip_threshold", clip_threshold_);
        nh_.getParam("ibvs/drone_pickup/block_zoffset", block_zoffset_);
        nh_.getParam("ibvs/drone_pickup/gain_K", k_);
    }
    else if (execution_mode_ == DRONE_PLACEMENT){
        nh_.getParam("ibvs/drone_placement/corners_topic", ibvs_corners_topic_);
        nh_.getParam("ibvs/drone_placement/num_image_points", num_image_points_);
        nh_.getParam("ibvs/drone_placement/camera_info", camera_info_);
        nh_.getParam("ibvs/drone_placement/clip_threshold", clip_threshold_);
        nh_.getParam("ibvs/drone_placement/block_zoffset", block_zoffset_);
        nh_.getParam("ibvs/drone_placement/gain_K", k_);
    }
}
