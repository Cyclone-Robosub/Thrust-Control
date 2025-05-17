#include "Command_Interpreter.h"
#include <gtest/gtest.h>

#ifndef MOCK_RPI

#include "Serial.h"

#else

bool initializeSerial(int* serial) {
    return true;
}

int getSerialChar(int* serial) {
    return EOF;
}

#endif

TEST(CommandInterpreterTest, CreateCommandInterpreter) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial);

    auto pinNumbers = std::vector<int>{4, 5, 2, 3, 9, 7, 8, 6};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new HardwarePwmPin(pinNumber, std::cout, outLog, std::cerr));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    auto pinStatus = interpreter->readPins();
    std::string output = testing::internal::GetCapturedStdout();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" HardPwm\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial)) != EOF) {
        serialOutput.push_back((char) charRead);
    }

    ASSERT_EQ(pinStatus.size(), 8);
    ASSERT_EQ(pinStatus, (std::vector<int>{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, CreateCommandInterpreterWithDigitalPins) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial);

    auto pinNumbers = std::vector<int>{4, 5, 2, 3, 9, 7, 8, 6};

    auto pwmPins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pwmPins.push_back(new HardwarePwmPin(pinNumber, std::cout, outLog, std::cerr));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto digital1 = new DigitalPin(8, ActiveLow, std::cout, outLog, std::cerr);
    auto digital2 = new DigitalPin(9, ActiveHigh, std::cout, outLog, std::cerr);
    auto digitalPins = std::vector<DigitalPin *>{digital1, digital2};

    auto interpreter = new Command_Interpreter_RPi5(pwmPins, digitalPins, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" HardPwm\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Configure 8 Digital\nSet 8 Digital High\n");
    expectedOutput.append("Configure 9 Digital\nSet 9 Digital Low\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_EQ(pinStatus.size(), 10);
    ASSERT_EQ(pinStatus, (std::vector<int>{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1, 0}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, UntimedExecute) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial);

    const pwm_array pwms = {1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536};

    auto pinNumbers = std::vector<int>{4, 5, 2, 3, 9, 7, 8, 6};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new HardwarePwmPin(pinNumber, std::cout, outLog, std::cerr));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    interpreter->untimed_execute(pwms);
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" HardPwm\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Set 4 PWM 1900\n");
    expectedOutput.append("Set 5 PWM 1900\n");
    expectedOutput.append("Set 2 PWM 1100\n");
    expectedOutput.append("Set 3 PWM 1250\n");
    expectedOutput.append("Set 9 PWM 1300\n");
    expectedOutput.append("Set 7 PWM 1464\n");
    expectedOutput.append("Set 8 PWM 1535\n");
    expectedOutput.append("Set 6 PWM 1536\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_EQ(pinStatus, (std::vector<int>{1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, BlindExecuteHardwarePwm) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial);

    const CommandComponent acceleration = {1900, 1900, 1100,
                                           1250, 1300, 1464, 1535,
                                           1536, std::chrono::milliseconds(2000)};

    auto pinNumbers = std::vector<int>{4, 5, 2, 3, 9, 7, 8, 6};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new HardwarePwmPin(pinNumber, std::cout, outLog, std::cerr));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    auto startTime = std::chrono::system_clock::now();
    interpreter->blind_execute(acceleration);
    auto endTime = std::chrono::system_clock::now();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" HardPwm\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Set 4 PWM 1900\n");
    expectedOutput.append("Set 5 PWM 1900\n");
    expectedOutput.append("Set 2 PWM 1100\n");
    expectedOutput.append("Set 3 PWM 1250\n");
    expectedOutput.append("Set 9 PWM 1300\n");
    expectedOutput.append("Set 7 PWM 1464\n");
    expectedOutput.append("Set 8 PWM 1535\n");
    expectedOutput.append("Set 6 PWM 1536\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(2000) /
                                                                      std::chrono::milliseconds(1),
                std::chrono::milliseconds(10) / std::chrono::milliseconds(1));
    ASSERT_EQ(pinStatus, (std::vector<int>{1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, BlindExecuteSoftwarePwm) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial);

    const CommandComponent acceleration = {1100, 1900, 1100,
                                           1250, 1300, 1464, 1535,
                                           1536, std::chrono::milliseconds(2000)};

    auto pinNumbers = std::vector<int>{4, 5, 2, 3, 9, 7, 8, 6};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new SoftwarePwmPin(pinNumber, std::cout, outLog, std::cerr));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    auto startTime = std::chrono::system_clock::now();
    interpreter->blind_execute(acceleration);
    auto endTime = std::chrono::system_clock::now();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" SoftPwm\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Set 4 PWM 1100\n");
    expectedOutput.append("Set 5 PWM 1900\n");
    expectedOutput.append("Set 2 PWM 1100\n");
    expectedOutput.append("Set 3 PWM 1250\n");
    expectedOutput.append("Set 9 PWM 1300\n");
    expectedOutput.append("Set 7 PWM 1464\n");
    expectedOutput.append("Set 8 PWM 1535\n");
    expectedOutput.append("Set 6 PWM 1536\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(2000) /
                                                                      std::chrono::milliseconds(1),
                std::chrono::milliseconds(10) / std::chrono::milliseconds(1));

    ASSERT_EQ(pinStatus, (std::vector<int>{1100, 1900, 1100, 1250, 1300, 1464, 1535, 1536}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

