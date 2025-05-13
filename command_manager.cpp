// CommandManager.cpp
#include "command_manager.hpp"

CommandManager::CommandManager(
    std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr,
    std::ostream& output,
    std::ostream& error,
    std::ostream& logFile
) :
    _commandInterpreter_ptr(std::move(commandInterpreter_ptr)),
    _output(output),
    _error(error),
    _logFile(logFile) 
{
    // COMMENT: Need error handling if logFile is nullptr
    // COMMENT: Should consider initializing pins here if needed
    
    output << "[CommandManager] Initialized" << std::endl;
}

void CommandManager::execute_decision_loop() {
    // This is similar to ExecutiveLoop::executeDecisionLoop
    // But simplified to focus only on command execution
    
    // COMMENT: Need a running flag or condition variable to control loop execution
    // COMMENT: Need mutex for thread safety
    
    while (true) { // COMMENT: Should have a way to break this loop
        // Check if there are commands in the queue
        if (!commandQueue.empty()) {
            // COMMENT: Need mutex lock here to safely access queue
            
            // Get the next command
            currentCommand = std::move(commandQueue.front());
            commandQueue.pop();
            
            // COMMENT: Need a flag to indicate a command is running
            
            _output << "[CommandManager] Executing next command" << std::endl;
        }
        
        // COMMENT: Should have a condition variable wait here
        // to avoid busy waiting
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Temporary
    }
}

void CommandManager::send_thruster_command() {
    // This is similar to ExecutiveLoop::sendThrusterCommand
    // But simplified to focus only on sending commands to thrusters
    
    // COMMENT: Need a running flag or condition variable to control loop execution
    // COMMENT: Need mutex for thread safety
    
    while (true) { // COMMENT: Should have a way to break this loop
        // Check if there's a current command to execute
        if (currentCommand) {
            // COMMENT: Need mutex lock here to safely access currentCommand
            
            _output << "[CommandManager] Sending command to thrusters" << std::endl;
            
            try {
                // Execute the command
                currentCommand->execute(*_commandInterpreter_ptr);
                
                // Clear the current command after execution
                // COMMENT: Need mutex lock here
                currentCommand.reset();
                
                _output << "[CommandManager] Command execution completed" << std::endl;
            } catch (const std::exception& e) {
                _error << "[CommandManager] Error executing command: " << e.what() << std::endl;
                
                // Clear the failed command
                // COMMENT: Need mutex lock here
                currentCommand.reset();
            }
        }
        
        // COMMENT: Should have a condition variable wait here
        // to avoid busy waiting
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Temporary
    }
}

// COMMENT: The following functions would be useful to add:
// 1. void start() - Start the command execution threads
// 2. void stop() - Stop the command execution threads
// 3. void queueCommand(std::unique_ptr<Pwm_Command> command) - Add a command to the queue
// 4. void clearQueue() - Clear all pending commands
// 5. void haltCurrentCommand() - Interrupt the current command
// 6. bool isExecuting() const - Check if a command is currently executing
// 7. size_t getQueueSize() const - Get the number of commands in the queue
