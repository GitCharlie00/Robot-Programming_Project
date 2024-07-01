#include "base_controller.cpp" // Assumendo che il tuo file C++ del controller si chiami base_controller.cpp

int main(int argc, char **argv) {
    ros::init(argc, argv, "cust_base_vel_controller");
    BaseController controller;
    controller.move_base_to_desired_orientation();
    return 0;
}
