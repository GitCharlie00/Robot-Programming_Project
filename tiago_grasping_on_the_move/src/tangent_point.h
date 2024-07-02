#ifndef TANGENT_POINT_H
#define TANGENT_POINT_H

#include <cmath>
#include <limits>
#include <iostream>

inline std::pair<double, double> tangent_point(double x_b, double y_b, double x_t, double y_t, double r_c, bool debug = false) {
    int iters = 360;
    double theta, old_diff = std::numeric_limits<double>::max();
    double x_c = 0, y_c = 0;

    for (int i = 0; i < iters; ++i) {
        theta = 2 * M_PI * i / iters;
        double candidate_x_tang = x_t + r_c * std::cos(theta);
        double candidate_y_tang = y_t + r_c * std::sin(theta);

        double m = (std::sin(theta) == 0.0) ? M_PI / 2 : -std::cos(theta) / std::sin(theta);
        double b = candidate_y_tang - m * candidate_x_tang;
        double y_test = m * x_b + b;

        if (std::abs(y_test - y_b) < old_diff && std::abs(x_c - candidate_x_tang) > 0.2) {
            if (debug) {
                std::cout << "x = " << x_c << " || y = " << y_c << " || theta = " << theta << std::endl;
            }
            x_c = candidate_x_tang;
            y_c = candidate_y_tang;
            old_diff = std::abs(y_test - y_b);
        }
    }
    return {x_c, y_c};
}

#endif // TANGENT_POINT_H
