#pragma once

#include <array>
#include <ostream>
#include <iostream>

class Position {
public:
    Position();
    Position(float x, float y, float z, float roll, float pitch, float yaw);
    Position(std::array<float, 6> values);

    float get_x();
    float get_y();
    float get_z();
    float get_roll();
    float get_pitch();
    float get_yaw();

    float operator[](int value);
    friend bool operator==(const Position& lhs, const Position& rhs);
    friend bool operator==(const Position& lhs, const std::array<float, 6>& rhs);
private:
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};