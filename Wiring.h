// William Barber

#pragma once

#include <unordered_map>

/// @brief What purpose the given pin is configured for
enum PinType {DigitalActiveLow, DigitalActiveHigh, HardwarePWM, SoftwarePWM};

/// @brief Whether a digital pin is currently low or high
enum DigitalPinStatus {Low, High};

/// @brief The parameters of a pwm pin's status (pulse width, frequency, and duty cycle)
struct PwmPinStatus {
    int pulseWidth;
    int frequency;
    int dutyCycle;
};

class WiringControl {
private:
    int serial = -1;
    std::unordered_map<int, PinType> pinTypes;
    std::unordered_map<int, PwmPinStatus> pwmPinStatuses;
    std::unordered_map<int, DigitalPinStatus> digitalPinStatuses;
public:
    /// @brief Perform necessary steps to initialize the GPIO of the Pi. Includes configuring serial connection to Pico.
    bool initializeGPIO();
    /// @brief Sets the pin with the given pin number to the purpose specified: either digital or pwm
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    /// @param pinType what the pin will be used for: one of either two types of digital pin or two types pwm pin
    void setPinType(int pinNumber, PinType pinType);
    /// @brief Set a digital pin to either high or low
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    /// @param digitalPinStatus a digital pin state, either high or low
    void digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus);
    /// @brief Read the specified digital pin status, either high or low. Does not actually read the pins directly: relies
    /// on cached status within the object
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    /// @return Whether the specified pin is high or low
    DigitalPinStatus digitalRead(int pinNumber);
    /// @brief Set a pwm pin to the specified frequency
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    /// @param pwmFrequency a pwm frequency between 1100 and 1900
    void pwmWrite(int pinNumber, int pwmFrequency);
    /// @brief Read the specified pwm pin status. Does not actually read the pins directly: relies on cached status
    /// within the object
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    /// @return The specified pin's frequency, pulse width, and duty cycle
    PwmPinStatus pwmRead(int pinNumber);
    /// @brief Set the specified pin the maximum pwm value (1900)
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    void pwmWriteMaximum(int pinNumber);
    /// @brief Set the specified pin to a pwm value that turns the thruster off (1500)
    /// @param pinNumber the GPIO number of the pin. See https://pinout.xyz/ or https://pico.pinout.xyz/
    void pwmWriteOff(int pinNumber);
    WiringControl() = default;
    ~WiringControl();
};
