#include "../src/tiago_ik.cpp"  // Ensure the path is correct

int main(int argc, char **argv) {
    ros::init(argc, argv, "tiago_arm");
    ros::NodeHandle nh;

    // Create an instance of the BaseController class
    ArmController controller;

    // Start the base movement to the desired orientation
    controller.move_arm();

    // Keep the node running
    ros::spin();

    return 0;
}