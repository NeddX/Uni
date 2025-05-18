#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

struct Vector3 {
    double x, y, z;

    Vector3 operator+(const Vector3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vector3 operator-(const Vector3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }

    double dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    double norm() const { return std::sqrt(dot(*this)); }

    Vector3 normalize() const {
        double n = norm();
        return (n > 0) ? *this / n : Vector3{0, 0, 0};
    }
};

struct Ball {
    Vector3 position;
    Vector3 velocity;
    double mass;
    double radius;
};

bool check_collision(const Ball& a, const Ball& b) {
    Vector3 delta = a.position - b.position;
    double distance_squared = delta.dot(delta);
    double radius_sum = a.radius + b.radius;
    return distance_squared < radius_sum * radius_sum;
}

void resolve_collision(Ball& a, Ball& b) {
    Vector3 n = (b.position - a.position).normalize();

    double v1n = a.velocity.dot(n);
    double v2n = b.velocity.dot(n);

    double m1 = a.mass, m2 = b.mass;

    // New normal velocities
    double v1n_prime = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2);
    double v2n_prime = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2);

    // Tangential component remains unchanged
    Vector3 v1t = a.velocity - n * v1n;
    Vector3 v2t = b.velocity - n * v2n;

    a.velocity = v1t + n * v1n_prime;
    b.velocity = v2t + n * v2n_prime;

    // Optional: separate balls slightly
    //Vector3 correction = n * 0.01;
    //a.position = a.position - correction;
    //b.position = b.position + correction;
}

void print_state(int frame, const std::vector<Ball>& balls) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Frame " << frame << ":\n";
    for (size_t i = 0; i < balls.size(); ++i) {
        const auto& b = balls[i];
        std::cout << "  Ball " << i + 1 << " Pos: (" << b.position.x << ", " << b.position.y << ", " << b.position.z
                  << ") Vel: (" << b.velocity.x << ", " << b.velocity.y << ", " << b.velocity.z << ")\n";
    }
    std::cout << "\n";
}

int main() {
    const double dt = 1.0 / 60.0;  // Time step (60 FPS)
    const int total_frames = 100 * 60;

    // 3
    // m   r   x   y   z   v_x  v_y v_z
    // 1.0 1.0 0.0 0.0 0.0 1.00 1.0 1.0
    // 2.0 2.0 10.0 10.0 10.0 -1.0 -1.0 -1.0
    // 3.0 3.0 20.0 20.0 20.0 -0.5 -0.5 -0.5
    // 100

    std::vector<Ball> balls = {
        //{{0, 0, 0}, {0, 0, 0}, 1.0, 1.0},
        //{{10, 0, 0}, {-3, 0, 0}, 1.0, 1.0},

        { {.0, .0, .0}, {1.0, 1.0, 1.0 }, 1.0, 1.0},
        { {10, 10, 10}, {-1.0, -1.0, 1.0}, 2.0, 2.0},
        { {20, 20, 20}, {-.5, -.5, -.5}, 3.0, 3.0},
    };

    for (int frame = 0; frame <= total_frames; ++frame) {
        print_state(frame, balls);

        // Update all positions first
        for (auto& ball : balls) {
            ball.position = ball.position + ball.velocity * dt;
        }

        // Check and resolve all pairwise collisions
        for (size_t i = 0; i < balls.size(); ++i) {
            for (size_t j = i + 1; j < balls.size(); ++j) {
                if (check_collision(balls[i], balls[j])) {
                    resolve_collision(balls[i], balls[j]);
                }
            }
        }
    }

    return 0;
}

