#include "position.hpp"

bool float_equal(float x, float y) {
    return (x - y < 0.00001) || (y - x < 0.00001);
}

Position::Position() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}

Position::Position(float x, float y, float z, float roll, float pitch, float yaw) :
    x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}

Position::Position(std::array<float,6> values) : x(values[0]), y(values[1]), z(values[2]),
    roll(values[3]), pitch(values[4]), yaw(values[5]) {}

float Position::operator[](int value) {
    switch(value) {
        case 0:
            return get_x();
        case 1:
            return get_y();
        case 2:
            return get_z();
        case 3:
            return get_roll();
        case 4:
            return get_pitch();
        case 5:
            return get_yaw();
        default:
            std::cerr << "Index out of bounds! Requested: " << value << "; valid range: [0,5]." << std::endl;
            exit(42);
    }
}

float Position::get_x() {
    return x;
}

float Position::get_y() {
    return y;
}

float Position::get_z() {
    return z;
}

float Position::get_roll() {
    return roll;
}

float Position::get_pitch() {
    return pitch;
}

float Position::get_yaw() {
    return yaw;
}

bool operator==(const Position& lhs, const Position& rhs) {
    return float_equal(lhs.x,rhs.x) && float_equal(rhs.y,rhs.y) && 
    float_equal(lhs.z,rhs.z) && float_equal(lhs.roll,rhs.roll) && 
    float_equal(lhs.pitch,rhs.pitch) && float_equal(lhs.yaw,rhs.yaw);
}

//bool operator==(const Position& lhs, const std::array<float, 6>& rhs) {
//    return float_equal(lhs.x,rhs[0]) && float_equal(lhs.y,rhs[1]) && 
//    float_equal(lhs.z,rhs[2]) && float_equal(lhs.roll,rhs[3]) && 
//    float_equal(lhs.pitch,rhs[4]) && float_equal(lhs.yaw,rhs[5]);
//}