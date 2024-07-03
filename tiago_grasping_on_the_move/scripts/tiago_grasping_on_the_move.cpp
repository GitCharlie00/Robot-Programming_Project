#include "../src/base_controller.cpp"  // Ensure the path is correct

int amain(int argc, char **argv) {
    ros::init(argc, argv, "tiago_grasping_on_the_move");
    ros::NodeHandle nh;

    // Create an instance of the BaseController class
    BaseController controller;

    // Start the base movement to the desired orientation
    controller.move_base_to_desired_orientation();

    // Keep the node running
    ros::spin();

    return 0;
}
