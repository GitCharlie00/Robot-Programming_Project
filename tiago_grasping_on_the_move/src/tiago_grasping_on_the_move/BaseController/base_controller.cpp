#include <ros/ros.h>
#include <math.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

class BaseController {
public:
    BaseController() {
        // Node settings
        ros::NodeHandle nh;
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
        velocity_subscriber = nh.subscribe("/mobile_base_controller/cmd_vel", 10, &BaseController::base_vel_callback, this);
        base_odom_subscriber = nh.subscribe("/gazebo/model_states", 10, &BaseController::base_odom_callback, this);
        target_odom_subscriber = nh.subscribe("/gazebo/model_states", 10, &BaseController::target_odom_callback, this);
        rate = 10;  // 10 Hz
        
        // Initialize other member variables...
    }

    void base_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "tiago") {
                eps_b_x = msg->pose[i].position.x;
                eps_b_y = msg->pose[i].position.y;
                // Handle quaternion and other calculations...
            }
        }
    }

    void target_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "unit_sphere") {
                eps_t_x = msg->pose[i].position.x;
                eps_t_y = msg->pose[i].position.y;
            }
            if (msg->name[i] == "unit_box_1") {
                eps_n_x = msg->pose[i].position.x;
                eps_n_y = msg->pose[i].position.y;
            }
        }
    }

    void base_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        history.v.push_back(msg->linear.x);
        history.w.push_back(msg->angular.z);
    }

    void close_gripper() {
        trajectory_msgs::JointTrajectory trajectory;
        trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.000, 0.000};
        point.time_from_start = ros::Duration(1.0);
        trajectory.points.push_back(point);
        pub_gripper_controller.publish(trajectory);
    }

    void open_gripper() {
        trajectory_msgs::JointTrajectory trajectory;
        trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.044, 0.044};
        point.time_from_start = ros::Duration(1.0);
        trajectory.points.push_back(point);
        pub_gripper_controller.publish(trajectory);
    }

    void move_base_to_desired_orientation() {
        ros::Rate loop_rate(rate);
        while (ros::ok() && !is_desired_pose_reached) {
            if (rho <= gripper_threshold) {
                close_gripper();
            }

            if (rho > threshold) {
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = v_b;
                vel_msg.angular.z = (k_alpha * alpha) * (v_b / rho);
                velocity_publisher.publish(vel_msg);
            } else {
                // Handle second phase...
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::Publisher velocity_publisher;
    ros::Subscriber velocity_subscriber;
    ros::Subscriber base_odom_subscriber;
    ros::Subscriber target_odom_subscriber;
    ros::Publisher pub_gripper_controller;
    int rate;

    // Attributes
    double eps_b_x, eps_b_y, theta;
    double eps_t_x, eps_t_y;
    double eps_c_x, eps_c_y;
    double eps_c_n_x, eps_c_n_y;
    double eps_n_x, eps_n_y;
    double alpha, beta, rho, rho_n;
    double v_b, k_alpha, k_beta;
    double gripper_threshold, threshold;
    bool is_desired_pose_reached;
    
    struct History {
        std::vector<double> v;
        std::vector<double> w;
        std::vector<double> alpha;
        std::vector<double> rho;
        std::vector<double> beta;
        std::vector<double> rho_n;
        double time;
        std::vector<double> x;
        std::vector<double> y;
    } history;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "cust_base_vel_controller");
    BaseController controller;
    controller.move_base_to_desired_orientation();
    return 0;
}
