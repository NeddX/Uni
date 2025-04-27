#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iomanip>
#include <iostream>
#include <thread>
#include <type_traits>

using i32   = std::int32_t;
using u32   = std::uint32_t;
using usize = std::size_t;
using f32   = float;

class vec3f {
private:
    f32 m_x = .0f;
    f32 m_y = .0f;
    f32 m_z = .0f;

public:
    constexpr vec3f(const f32 x = .0f, const f32 y = .0f,
                    const f32 z = .0f) noexcept
        : m_x(x)
        , m_y(y)
        , m_z(z) {}

public:
    [[nodiscard]] constexpr f32& x() noexcept { return m_x; }
    [[nodiscard]] constexpr f32& y() noexcept { return m_y; }
    [[nodiscard]] constexpr f32& z() noexcept { return m_z; }
    [[nodiscard]] constexpr f32  x() const noexcept {
        return const_cast<
                    std::remove_cv_t<std::remove_reference_t<decltype(*this)>>*>(
                   this)
            ->x();
    }
    [[nodiscard]] constexpr f32 y() const noexcept {
        return const_cast<
                   std::remove_cv_t<std::remove_reference_t<decltype(*this)>>*>(
                   this)
            ->y();
    }
    [[nodiscard]] constexpr f32 z() const noexcept {
        return const_cast<
                   std::remove_cv_t<std::remove_reference_t<decltype(*this)>>*>(
                   this)
            ->z();
    }

public:
    [[nodiscard]] constexpr vec3f operator+(const vec3f& other) const noexcept {
        return vec3f{ this->m_x + other.m_x, this->m_y + other.m_y,
                      this->m_z + other.m_z };
    }
    [[nodiscard]] constexpr vec3f operator-(const vec3f& other) const noexcept {
        return vec3f{ this->m_x - other.m_x, this->m_y - other.m_y,
                      this->m_z - other.m_z };
    }
    template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    [[nodiscard]] constexpr vec3f operator*(const T scalar) const noexcept {
        return vec3f{ this->m_x * scalar, this->m_y * scalar,
                      this->m_z * scalar };
    }
    constexpr vec3f& operator+=(const vec3f& other) noexcept {
        this->m_x += other.m_x, this->m_y += other.m_y, this->m_z += other.m_z;
        return *this;
    }
    constexpr vec3f& operator-=(const vec3f& other) noexcept {
        this->m_x -= other.m_x, this->m_y -= other.m_y, this->m_z -= other.m_z;
        return *this;
    }
    template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    constexpr vec3f& operator*=(const T scalar) noexcept {
        this->m_x *= scalar, this->m_y *= scalar, this->m_z *= scalar;
        return *this;
    }
    [[nodiscard]] constexpr bool operator==(const vec3f& other) const noexcept {
        return this->m_x == other.m_x && this->m_y == other.m_y &&
               this->m_z == other.m_z;
    }

public:
    [[nodiscard]] constexpr f32 magnitude() const noexcept {
        // Very inefficient for real-life physics applications.
        return std::sqrt((m_x * m_x) + (m_y * m_y) + (m_z * m_z));
    }
    [[nodiscard]] constexpr vec3f normalise() const noexcept {
        const auto mag = magnitude();
        auto       res = *this;
        res.m_x /= mag, res.m_y /= mag, res.m_z /= mag;
        return res;
    }
    [[nodiscard]] constexpr f32 dot(const vec3f& other) const noexcept {
        return this->m_x * other.m_x + this->m_y * other.m_y +
               this->m_z * other.m_z;
    }

public:
    friend std::ostream& operator<<(std::ostream& os,
                                    const vec3f&  vec) noexcept {
        os << std::format("({}, {}, {})", vec.m_x, vec.m_y, vec.m_z);
        return os;
    }
};

struct ball {
public:
    f32   mass  = {};
    f32   radii = {};
    vec3f pos   = {};
    vec3f vel   = {};

public:
    [[nodiscard]] constexpr bool operator==(const ball& other) const noexcept {
        return this->mass == other.mass && this->radii == other.radii &&
               this->pos == other.pos && this->vel == other.vel;
    }
    [[nodiscard]] constexpr bool operator!=(const ball& other) const noexcept {
        return !(this->operator==(other));
    }
};

constexpr auto FRAME_INTERVAL = 1.0f / 5.0f; // hz
constexpr auto BALL_COUNT     = 2;
constexpr auto SIM_DURATION   = 999; // seconds

static auto                         s_run = true;
static std::array<ball, BALL_COUNT> s_entities{
    ball{ .mass  = 1.0f,
          .radii = 1.0f,
          .pos   = { .0f, .0f, .0f },
          .vel   = { .0f, .0f, .0f } },
    ball{ .mass  = 1.0f,
          .radii = 1.0f,
          .pos   = { 10.0f, .0f, .0f },
          .vel   = { -1.0f, .0f, .0f } },
};
static auto s_time_passed = .0f;

void update(const f32 dt) {
    using clock = std::chrono::steady_clock;

    static auto t1          = clock::now();

    int i = 0;
    for (auto& e : s_entities) {
        // Apply the velocity and take delta time account.
        e.pos += e.vel * dt;
    }

    for (auto& a : s_entities) {
        // Collision detection and resolution
        for (auto& b : s_entities) {
                // Check if the two balls are possibly in a collision.
            if (a != b) {
                if ((a.pos - b.pos).magnitude() <= a.radii + b.radii) {
                    // Collision normal (x axies).
                    const auto norm  = (a.pos - b.pos).normalise();
                    const auto r_vel = a.vel - b.vel;

                    // Project relative velocity onto the normalised Collision
                    // axies (normalised to not interferre with our velocity).
                    const auto r_vel_n = norm.dot(r_vel);

                }
                else {
                }
            }

        }
    }

    {
        const auto dur = std::chrono::duration<f32>(clock::now() - t1).count();
        //std::cout << dur << "s" << std::endl;
        
        std::cout << "tps: " << 1.0f / dur << "hz" << std::endl;
        std::cout << "time: " << s_time_passed << "s" << std::endl;
        for (usize i = 0; i < s_entities.size(); ++i) {
            const auto e = s_entities[i];
            std::cout << "entity #" << i << std::endl;
            std::cout << " mass: " << e.mass << std::endl;
            std::cout << " radii: " << e.radii << std::endl;
            std::cout << " pos: " << e.pos << std::endl;
            std::cout << " vel: " << e.vel << std::endl;
        }
        std::cout << std::setfill('-') << std::setw(25) << "-" << std::endl;

        if ((s_time_passed += dur) >= SIM_DURATION) {
            s_run = false;
        } else {
            t1 = clock::now();
        }
    }
}

i32 main() {
    using clock = std::chrono::steady_clock;

    // Fixed sub-tick update based on FRAME_INTERVAL
    auto frame_start = clock::now();
    auto lag         = .0f;
    while (s_run) {
        const auto frame_now = clock::now();
        const auto local_frame_time =
            std::chrono::duration<f32>(clock::now() - frame_start).count();

        frame_start = frame_now;
        lag += local_frame_time;

        // update
        for (; lag >= FRAME_INTERVAL; lag -= FRAME_INTERVAL) {
            update(FRAME_INTERVAL);
        }

        if (local_frame_time < FRAME_INTERVAL) {
            std::this_thread::sleep_for(
                std::chrono::duration<f32>(FRAME_INTERVAL - local_frame_time));
        }
    }

    return 0;
}
