#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <vector>

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

#endif // QUATERNION_H
