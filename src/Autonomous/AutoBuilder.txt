#ifndef AUTOBUILDER 
#define AUTOBUILDER
#include <vector>
#include "Command.cpp"

class AutoBuilder //maybe implement chaining these sequential/parallel commands later
{
    public:
    std::vector<Command> auto_commands;
    int current_index = 0;

    //constructor 
    AutoBuilder() {

    }

    void add_commands(const std::vector<Command>& commands) {
        for (Command com : commands) {
            auto_commands.push_back(com); //or emplace_back??
        };
    }

    void add_command(const Command& command) {
        auto_commands.push_back(command);
    }

    bool run_commands() { //return false until the command is done
        if (auto_commands.size() == 0) {
            return true;
        }
        if (current_index >= auto_commands.size()) {
            return true;
        }
        Command current_command = auto_commands.at(current_index);
        bool next_command = current_command.run();
        if (next_command) {
            current_index += 1;
        }
        return false;
    }
};
#endif