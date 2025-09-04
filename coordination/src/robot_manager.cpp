#include"coordination/robot_manager.hpp"

Polygon make_circle(double cx, double cy, double radius, int num_segments) {
    Polygon poly;
    auto &outer = poly.outer();
    outer.reserve(num_segments + 1);

    for (int i = 0; i <= num_segments; i++) {
        double theta = 2.0 * M_PI * double(i) / double(num_segments);
        double x = cx + radius * cos(theta);
        double y = cy + radius * sin(theta);
        outer.emplace_back(x, y);
    }

    return poly;
}