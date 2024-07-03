#include "../src/base_controller.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "tiago_base_controller");
    ros::NodeHandle nh;

    // Create an instance of the BaseController class
    BaseController controller;

    // Start the base movement to the desired orientation
    controller.move_base_to_desired_orientation();

    // Keep the node running
    ros::spin();

    return 0;
}

