#include "../src/base_controller.cpp"  // Assicurati che il percorso sia corretto

int main(int argc, char **argv) {
    ros::init(argc, argv, "tiago_grasping_on_the_move");
    ros::NodeHandle nh;

    initializeController();

    ros::spin();
    return 0;
}
