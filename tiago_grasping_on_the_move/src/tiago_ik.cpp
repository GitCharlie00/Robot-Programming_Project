#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class ArmController {
public:
    ArmController() {
        // Node settings
        ros::NodeHandle nh;
        move_group = new moveit::planning_interface::MoveGroupInterface("arm_torso");
        base_odom_sub = nh.subscribe("/gazebo/model_states", 10, &ArmController::base_odom_callback, this);
        target_odom_sub = nh.subscribe("/gazebo/model_states", 10, &ArmController::target_odom_callback, this);

        // Controller settings
        init_d = 1.0;
        init_rho = 1.0;
        set_init_rho = false;

        // Target position
        eps_t_x = 0.0;
        eps_t_y = 0.0;
        eps_t_z = 0.0;

        // Base position
        eps_b_x = 0.0;
        eps_b_y = 0.0;
        theta = 0.0;

        // Closest approach point
        r_c = 0.6;
        eps_c_x = 0.0;
        eps_c_y = 0.0;
        d = 0.0;
        rho = 0.0;
        alpha = 0.0;

        // Grasping goal
        from_frame = "odom";
        to_frame = "base_footprint";
        ee_orientation_r = -1.5;
        ee_orientation_p = 0.5;
        ee_orientation_y = -0.0;

        // Trajectory planner
        planner = "PRMstarkConfigDefault";

        rate = new ros::Rate(10); // 10 Hz
    }

    ~ArmController() {
        delete move_group;
        delete rate;
    }

    void move_arm() {
        ros::Duration(1.6).sleep();

        geometry_msgs::PointStamped target_point;
        target_point.header.frame_id = from_frame;
        target_point.point.x = eps_t_x;
        target_point.point.y = eps_t_y;
        target_point.point.z = eps_t_z;

        geometry_msgs::Point transformed_coordinates = transform_point(target_point, from_frame, to_frame);

        tf2::Quaternion quaternion;
        quaternion.setRPY(ee_orientation_r, ee_orientation_p, ee_orientation_y);
        quaternion.normalize();

        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = to_frame;
        goal_pose.pose.position.x = transformed_coordinates.x;
        goal_pose.pose.position.y = transformed_coordinates.y;
        goal_pose.pose.position.z = transformed_coordinates.z;
        goal_pose.pose.orientation.x = quaternion.x();
        goal_pose.pose.orientation.y = quaternion.y();
        goal_pose.pose.orientation.z = quaternion.z();
        goal_pose.pose.orientation.w = quaternion.w();

        move_group->setPlannerId(planner);
        move_group->allowReplanning(false);
        move_group->setPoseReferenceFrame(from_frame);
        move_group->setPoseTarget(goal_pose);
        move_group->setStartStateToCurrentState();
        move_group->setMaxVelocityScalingFactor(1.0);

        ROS_INFO("Planning to move %s to a target pose expressed in %s",
            move_group->getEndEffectorLink().c_str(),
            move_group->getPlanningFrame().c_str());

        moveit::planning_interface::MoveGroupInterface::Plan grasping_plan;
        bool success = (move_group->plan(grasping_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success) {
            ROS_ERROR("No plan found");
            return;
        }

        ros::Duration((init_rho / 0.3) - 10 - 3).sleep(); // to move the arm after a certain delay

        ros::Time start_time = ros::Time::now();
        move_group->move();
        ROS_INFO("Motion duration: %f", (ros::Time::now() - start_time).toSec());
    }

private:
    moveit::planning_interface::MoveGroupInterface* move_group;
    ros::Subscriber base_odom_sub;
    ros::Subscriber target_odom_sub;
    ros::Rate* rate;

    double init_d;
    double init_rho;
    bool set_init_rho;

    double eps_t_x;
    double eps_t_y;
    double eps_t_z;

    double eps_b_x;
    double eps_b_y;
    double theta;

    double r_c;
    double eps_c_x;
    double eps_c_y;
    double d;
    double rho;
    double alpha;

    std::string from_frame;
    std::string to_frame;
    double ee_orientation_r;
    double ee_orientation_p;
    double ee_orientation_y;

    std::string planner;

    void base_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "tiago") {
                eps_b_x = msg->pose[i].position.x;
                eps_b_y = msg->pose[i].position.y;

                tf2::Quaternion orientation_quaternion(
                    msg->pose[i].orientation.x,
                    msg->pose[i].orientation.y,
                    msg->pose[i].orientation.z,
                    msg->pose[i].orientation.w
                );

                tf2::Matrix3x3 mat(orientation_quaternion);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);
                theta = yaw;
            }
        }
    }

    void target_odom_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "unit_sphere") {
                eps_t_x = msg->pose[i].position.x;
                eps_t_y = msg->pose[i].position.y;
                eps_t_z = msg->pose[i].position.z;
            }
        }
    }

    geometry_msgs::Point transform_point(const geometry_msgs::PointStamped& point, const std::string& from_frame, const std::string& to_frame) {
        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        geometry_msgs::PointStamped transformed_point;

        try {
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(1.0));
            tf2::doTransform(point, transformed_point, transform);
        }
        catch (tf2::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
            return geometry_msgs::Point();
        }

        return project_into_workspace(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
    }

    geometry_msgs::Point project_into_workspace(double x, double y, double z) {
        // Implement tangent_point() and related calculations here.

        // Placeholder return value.
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_arm_torso_ik");

    ArmController controller;
    controller.move_arm();

    ros::shutdown();
    return 0;
}
