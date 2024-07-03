/*************** Libraries ***************/
#include <iostream> //Print and read
#include <vector> // Dynamic dimension array    
#include <cmath> // Common math functions
#include <ros/ros.h> // Essential ROS instruments
#include <geometry_msgs/Twist.h> // Mobile base messages
#include <gazebo_msgs/ModelStates.h> // Environment element coordinates
#include <trajectory_msgs/JointTrajectory.h> // Gripper movement
#include <trajectory_msgs/JointTrajectoryPoint.h> // Gripper movement

/*************** Modules ***************/
#include "tangent_point.h" // Compute closest approach point
#include "quaternion.h" // Map quaternion to mobile base orientation

/*************** Mobile base controller ***************/
class BaseController {
private:
    // ROS topics
    ros::NodeHandle nh; // Handle to allow program to interact with ROS topics
    ros::Publisher pub_gripper_controller;
    ros::Publisher velocity_publisher;
    ros::Subscriber base_odom_subscriber;
    ros::Subscriber target_odom_subscriber;
    ros::Rate rate;

    // Variables for mobile base position and orientation
    double eps_b_x, eps_b_y, theta;
    Quaternion orientation_quaternion;
    double eps_t_x, eps_t_y, eps_c_x, eps_c_y, eps_c_n_x, eps_c_n_y;
    double eps_n_x, eps_n_y;

    // Controller parameters
    double alpha, beta, rho, rho_n;
    double v_b, k_alpha, k_beta;
    double gripper_threshold, threshold, gripper_threshold_n, threshold_n, r_c, d, d_n;

    // Flags for state management
    bool is_desired_pose_reached;
    bool is_desired_orientation_updated;
    bool second_phase;
    bool set_init_rho;

public:
    // Class constructor
    BaseController()
        : rate(10), 
          v_b(0.3), 
          k_alpha(4), 
          k_beta(2.2),
          gripper_threshold(0.58),
          threshold(1.45),
          gripper_threshold_n(0.65),
          threshold_n(0.65),
          r_c(0.6),
          d(1.0),
          d_n(1.0),
          eps_b_x(0.0),
          eps_b_y(0.0),
          theta(0.0),
          eps_t_x(0.0),
          eps_t_y(0.0),
          eps_c_x(0.0),
          eps_c_y(0.0),
          eps_c_n_x(0.0),
          eps_c_n_y(0.0),
          eps_n_x(0.0),
          eps_n_y(0.0),
          alpha(1.0),
          beta(1.0),
          rho(1.0),
          rho_n(1.0),
          is_desired_pose_reached(false),
          is_desired_orientation_updated(false),
          second_phase(false),
          set_init_rho(false) 
    {
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
        pub_gripper_controller = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
        base_odom_subscriber = nh.subscribe("/gazebo/model_states", 10, &BaseController::base_odom_callback, this);
        target_odom_subscriber = nh.subscribe("/gazebo/model_states", 10, &BaseController::target_odom_callback, this);
    }

    void base_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "tiago") {
                // Update robot's x and y positions
                eps_b_x = msg->pose[i].position.x;
                eps_b_y = msg->pose[i].position.y;
                
                // Update robot's orientation from message quaternion
                orientation_quaternion = Quaternion(
                    msg->pose[i].orientation.w,
                    msg->pose[i].orientation.x,
                    msg->pose[i].orientation.y,
                    msg->pose[i].orientation.z
                );

                // Compute robot's orientation angle theta in radians
                std::vector<double> rad = orientation_quaternion.toEulerAngles();
                theta = rad[2];

                // Compute Euclidean distance d between eps_t (grasping target) and eps_b (mobile base)
                d = std::sqrt((eps_t_x - eps_b_x) * (eps_t_x - eps_b_x) + (eps_t_y - eps_b_y) * (eps_t_y - eps_b_y));
                
                // Compute Euclidean distance d_n between eps_n (placing target) and eps_b (mobile base)
                d_n = std::sqrt((eps_n_x - eps_b_x) * (eps_n_x - eps_b_x) + (eps_n_y - eps_b_y) * (eps_n_y - eps_b_y));

                // Compute mobile position and orientation error
                rho = (r_c > d) ? 0 : std::sqrt(d * d - r_c * r_c);
                alpha = std::atan2(eps_b_y - eps_c_y, eps_b_x - eps_c_x) + M_PI - theta;
                beta = std::atan2(eps_b_y - eps_c_n_y, eps_b_x - eps_c_n_x) + M_PI - theta;
                rho_n = (r_c > d_n) ? 0 : std::sqrt(d_n * d_n - r_c * r_c);

                // Compute online the closest approach point to the targets
                auto [cx, cy] = tangent_point(eps_b_x, eps_b_y, eps_t_x, eps_t_y, r_c); // Variable type deducet by compiler
                eps_c_x = cx;
                eps_c_y = cy;

                auto [cnx, cny] = tangent_point(eps_b_x, eps_b_y, eps_n_x, eps_n_y, r_c); // Variable type deducet by compiler
                eps_c_n_x = cnx;
                eps_c_n_y = cny;
            }
        }
    }

    void close_gripper() {
        // Create a trajectory message for controlling the gripper
        trajectory_msgs::JointTrajectory trajectory;

        // Specify the joint names that will be controlled        
        trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        
        // Create a trajectory point for the gripper position
        trajectory_msgs::JointTrajectoryPoint trajectory_points;
        
        // Set the desired positions for the gripper joints (closed position)
        trajectory_points.positions = {0.000, 0.000};

        // Set the duration after which the gripper should reach the desired position
        trajectory_points.time_from_start = ros::Duration(0.3);

        // Add the trajectory point to the trajectory
        trajectory.points.push_back(trajectory_points);

        // Publish the trajectory message to the gripper controller topic
        pub_gripper_controller.publish(trajectory);
    }

    void open_gripper() {
        // Create a trajectory message for controlling the gripper        
        trajectory_msgs::JointTrajectory trajectory;

        // Specify the joint names that will be controlled        
        trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};

        // Create a trajectory point for the gripper position
        trajectory_msgs::JointTrajectoryPoint trajectory_points;

        // Set the desired positions for the gripper joints (open position)
        trajectory_points.positions = {0.044, 0.044};

        // Set the duration after which the gripper should reach the desired position
        trajectory_points.time_from_start = ros::Duration(0.3);

        // Add the trajectory point to the trajectory
        trajectory.points.push_back(trajectory_points);

        // Publish the trajectory message to the gripper controller topic
        pub_gripper_controller.publish(trajectory);
    }

    void target_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "unit_sphere") {
                // Update grasping target position coordinates eps_t_x and eps_t_y
                eps_t_x = msg->pose[i].position.x;
                eps_t_y = msg->pose[i].position.y;
            } else if (msg->name[i] == "unit_box_1") {
                // Update placing target position coordinates eps_n_x and eps_n_y
                eps_n_x = msg->pose[i].position.x;
                eps_n_y = msg->pose[i].position.y;
            }
        }
    }

    void move_base_to_desired_orientation() {
        // Pause execution for 2.5 seconds to allow initialization
        ros::Duration(2.5).sleep();
        
        // Get the current time to measure total simualtion time at the end
        ros::Time start_time = ros::Time::now();
        
        // Continue loop until final goal is reached or ROS is shutdown
        while (ros::ok() && !is_desired_pose_reached) {
            // Process any pending ROS callbacks
            ros::spinOnce();
            
            // Check if robot is close enough to the grasping target to close gripper
            if (rho <= gripper_threshold) close_gripper();
            
            // Grasping phase
            if (rho > threshold  && !second_phase) {
                // Calculate and publish velocity for grasping phase
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = v_b;
                vel_msg.angular.z = (k_alpha * alpha) * (v_b / rho);
                velocity_publisher.publish(vel_msg);
                rate.sleep();
            } else {
                // Placing phase
                second_phase = true;

                // Check if robot is close enough to the placing target to open gripper
                if (rho_n <= gripper_threshold_n) open_gripper();

                if (rho_n > threshold_n) {
                    // Calculate and publish velocity for placing phase
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = v_b;
                    vel_msg.angular.z = (k_beta * beta) * (v_b / rho_n);
                    velocity_publisher.publish(vel_msg);
                    rate.sleep();
                } else {
                    // Goal reached
                    ros::Time end_time = ros::Time::now();
                    ROS_INFO_STREAM("Total simulation time: " << (end_time - start_time).toSec() << " sec");
                    // Stop the robot
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = 0.0;
                    velocity_publisher.publish(vel_msg);
                    rate.sleep();

                    // Set flag indicating desired pose is reached
                    is_desired_pose_reached = true;
                }
            }
        }
    }
};
