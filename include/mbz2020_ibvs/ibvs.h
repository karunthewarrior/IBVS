#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mbz2020_custom_messages/Int32MultiArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

enum mode {DRONE_PICKUP, DRONE_PLACEMENT, HUSKY_PICKUP, HUSKY_PLACEMENT};

class IBVS {
private:
    ros::Subscriber corners_sub_, z_sub_;
    ros::Publisher velocity_pub_;
    ros::NodeHandle nh_;
    Eigen::MatrixXd f_,fd_;
    Eigen::MatrixXd L_, L_star_;
    Eigen::VectorXd K_;
    Eigen::MatrixXd error_;
    std::vector<double> z_estimate_, camera_info_, clip_threshold_;
    ros::Time corners_stamp_, sf30_stamp_;
    double block_zoffset_;
    std::string ibvs_corners_topic_;

    mode execution_mode_;

    Eigen::MatrixXd generate_interaction_matrix_block(const double u, const double v, const double z);
    Eigen::MatrixXd generate_full_interaction_matrix(const Eigen::MatrixXd& corners_mat, const std::vector<double>& z);

    void corners_callback(const mbz2020_custom_messages::Int32MultiArrayStampedConstPtr& corners);
    void z_estimate_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    std::vector<double> k_;

public:
    IBVS(ros::NodeHandle& nh, mode m);
    double error_norm_;
    Eigen::VectorXd compute_velocity(int mode = 0);
    void set_goal_state(const Eigen::MatrixXd& fd, const std::vector<double>& z);
    void publish_velocity(const Eigen::VectorXd velocity);
	bool check_convergence(double value);
    void update_execution_mode(mode m);
    int num_image_points_;
};