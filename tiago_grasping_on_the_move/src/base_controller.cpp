#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "tangent_point.h"  // Ensure this is included

class Quaternion {
public:
    double w, x, y, z;

    Quaternion(double w = 1, double x = 0, double y = 0, double z = 0)
        : w(w), x(x), y(y), z(z) {}

    double norm() {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalize() {
        double n = norm();
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    Quaternion conjugate() {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion multiply(const Quaternion& other) {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }

    std::vector<double> toEulerAngles() {
        std::vector<double> euler(3);
        double ysqr = y * y;

        // Roll (x-axis rotation)
        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + ysqr);
        euler[0] = std::atan2(t0, t1);

        // Pitch (y-axis rotation)
        double t2 = +2.0 * (w * y - z * x);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        euler[1] = std::asin(t2);

        // Yaw (z-axis rotation)
        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (ysqr + z * z);
        euler[2] = std::atan2(t3, t4);

        return euler;
    }
};

class BaseController {
private:
    ros::NodeHandle nh;
    ros::Publisher velocity_publisher;
    ros::Subscriber base_odom_subscriber;
    ros::Subscriber target_odom_subscriber;
    ros::Rate rate;

    double eps_b_x, eps_b_y, theta;
    Quaternion orientation_quaternion;
    double eps_t_x, eps_t_y, eps_c_x, eps_c_y, eps_c_n_x, eps_c_n_y;
    double eps_n_x, eps_n_y;
    double alpha, beta, rho, rho_n;
    double v_b, k_alpha, k_beta;
    double gripper_threshold, threshold, gripper_threshold_n, threshold_n, r_c, d, d_n;

    bool is_desired_pose_reached;
    bool is_desired_orientation_updated;
    bool second_phase;
    bool set_init_rho;

    // Placeholder for history data - not implementing full plotting functionality in C++
    std::vector<double> history_v, history_w, history_alpha, history_rho, history_beta, history_rho_n, history_x, history_y;

public:
    BaseController()
        : rate(10), v_b(0.3), k_alpha(4), k_beta(2.2),
          gripper_threshold(1.65), threshold(1.75), gripper_threshold_n(2.30), threshold_n(2.20),
          r_c(0.6), d(1.0), d_n(1.0), eps_b_x(0.0), eps_b_y(0.0), theta(0.0),
          eps_t_x(0.0), eps_t_y(0.0), eps_c_x(0.0), eps_c_y(0.0), eps_c_n_x(0.0), eps_c_n_y(0.0),
          eps_n_x(0.0), eps_n_y(0.0), alpha(1.0), beta(1.0), rho(1.0), rho_n(1.0),
          is_desired_pose_reached(false), is_desired_orientation_updated(false), second_phase(false), set_init_rho(false) {
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
        base_odom_subscriber = nh.subscribe("/gazebo/model_states", 10, &BaseController::base_odom_callback, this);
        target_odom_subscriber = nh.subscribe("/gazebo/model_states", 10, &BaseController::target_odom_callback, this);
    }

    void base_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "tiago") {
                eps_b_x = msg->pose[i].position.x;
                eps_b_y = msg->pose[i].position.y;
                orientation_quaternion = Quaternion(
                    msg->pose[i].orientation.w,
                    msg->pose[i].orientation.x,
                    msg->pose[i].orientation.y,
                    msg->pose[i].orientation.z
                );
                std::vector<double> rad = orientation_quaternion.toEulerAngles();
                theta = rad[2];

                d = std::sqrt((eps_t_x - eps_b_x) * (eps_t_x - eps_b_x) + (eps_t_y - eps_b_y) * (eps_t_y - eps_b_y));
                d_n = std::sqrt((eps_n_x - eps_b_x) * (eps_n_x - eps_b_x) + (eps_n_y - eps_b_y) * (eps_n_y - eps_b_y));

                rho = (r_c > d) ? 0 : std::sqrt(d * d - r_c * r_c);
                alpha = std::atan2(eps_b_y - eps_c_y, eps_b_x - eps_c_x) + M_PI - theta;

                beta = std::atan2(eps_b_y - eps_c_n_y, eps_b_x - eps_c_n_x) + M_PI - theta;
                rho_n = (r_c > d_n) ? 0 : std::sqrt(d_n * d_n - r_c * r_c);

                history_alpha.push_back(alpha);
                history_beta.push_back(beta);
                history_rho.push_back(rho);
                history_rho_n.push_back(rho_n);
                history_x.push_back(eps_b_x);
                history_y.push_back(eps_b_y);

                auto [cx, cy] = tangent_point(eps_b_x, eps_b_y, eps_t_x, eps_t_y, r_c);
                eps_c_x = cx;
                eps_c_y = cy;
                auto [cnx, cny] = tangent_point(eps_b_x, eps_b_y, eps_n_x, eps_n_y, r_c);
                eps_c_n_x = cnx;
                eps_c_n_y = cny;
            }
        }
    }

    void close_gripper() {
        ros::Publisher pub_gripper_controller = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
        trajectory_msgs::JointTrajectory trajectory;
        trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint trajectory_points;
        trajectory_points.positions = {0.000, 0.000};
        trajectory_points.time_from_start = ros::Duration(1.0);
        trajectory.points.push_back(trajectory_points);
        pub_gripper_controller.publish(trajectory);
    }

    void open_gripper() {
        ros::Publisher pub_gripper_controller = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
        trajectory_msgs::JointTrajectory trajectory;
        trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint trajectory_points;
        trajectory_points.positions = {0.044, 0.044};
        trajectory_points.time_from_start = ros::Duration(1.0);
        trajectory.points.push_back(trajectory_points);
        pub_gripper_controller.publish(trajectory);
    }

    void target_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "unit_sphere") {
                eps_t_x = msg->pose[i].position.x;
                eps_t_y = msg->pose[i].position.y;
            } else if (msg->name[i] == "unit_box_1") {
                eps_n_x = msg->pose[i].position.x;
                eps_n_y = msg->pose[i].position.y;
            }
        }
    }

    void move_base_to_desired_orientation() {
        ros::Duration(2.5).sleep();
        ros::Time start_time = ros::Time::now();

        while (ros::ok() && !is_desired_pose_reached) {
            std::cout << "******************************************" << rho << std::endl;
            std::cout << "Base linear velocity  : " << v_b << std::endl;
            std::cout << "Base angular velocity : " << (k_alpha * alpha) * (v_b / rho) << std::endl;
            std::cout << "Angualr gain          : " << k_alpha << std::endl;
            std::cout << "Alpha                 : " << alpha << std::endl;
            std::cout << "Rho                   : " << rho << std::endl;
            std::cout << "Grasping target x     : " << eps_t_x << std::endl;
            std::cout << "Grasping target y     : " << eps_t_y << std::endl;
            std::cout << "Closest apporach x    : " << eps_c_x << std::endl;
            std::cout << "Closest apporach x    : " << eps_c_y << std::endl;
            std::cout << "******************************************" << rho << std::endl;
            
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = v_b;
            vel_msg.angular.z = (k_alpha * alpha) * (v_b / rho);
            velocity_publisher.publish(vel_msg);
            rate.sleep();

            // if (rho <= gripper_threshold) {
            //     close_gripper();
            // }

            // if (rho > threshold) {
            //     geometry_msgs::Twist vel_msg;
            //     vel_msg.linear.x = v_b;
            //     vel_msg.angular.z = (k_alpha * alpha) * (v_b / rho);
            //     velocity_publisher.publish(vel_msg);
            //     rate.sleep();
            // } else {
            //     if (rho_n <= gripper_threshold_n) {
            //         open_gripper();
            //     }

            //     if (rho_n > threshold_n) {
            //         geometry_msgs::Twist vel_msg;
            //         vel_msg.linear.x = v_b;
            //         vel_msg.angular.z = (k_beta * beta) * (v_b / rho_n);
            //         velocity_publisher.publish(vel_msg);
            //         rate.sleep();
            //     } else {
            //         is_desired_pose_reached = true;
            //         ros::Time end_time = ros::Time::now();
            //         std::cout << "********************" << (end_time - start_time).toSec() << std::endl;
            //         geometry_msgs::Twist vel_msg;
            //         vel_msg.linear.x = 0.0;
            //         vel_msg.angular.z = 0.0;
            //         velocity_publisher.publish(vel_msg);
            //         rate.sleep();
            //         is_desired_pose_reached = true;
            //     }
            // }
        }
    }
};
