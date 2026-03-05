#ifndef AUTOBUILDER 
#define AUTOBUILDER
#include <vector>
#include "command.h"

class AutoBuilder {
    public:
    std::vector<Command> auto_commands;
    int current_index = 0;
    AutoBuilder();
    void add_commands(const std::vector<Command>& commands);
    void add_command(const Command& command);
    bool run_commands();
};
#endif