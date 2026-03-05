#include "command.h"

Command::Command(std::function<bool()> operation)
{
    commands_to_run.push_back(operation);
    // or emplace_back
}

Command::Command(std::function<bool()> operation1, std::function<bool()> operation2, CommandType commandType)
{
    commands_to_run.push_back(operation1);
    commands_to_run.push_back(operation2);
    type = commandType;
    // or emplace_back
}

Command::Command(std::vector<std::function<bool()>> operations, CommandType commandType)
{
    for (std::function<bool()> com : operations)
    {
        commands_to_run.push_back(com);
    }
    type = commandType;
}

bool Command::run()
{
    // if there is nothing to run exit
    if (commands_to_run.size() == 0)
    {
        return true;
    }

    if (condition)
    {
        return true;
    }
    if (type == CommandType::Sequence)
    { // sequencial
        std::function<bool()> func = commands_to_run.at(0);
        condition = func();
    }
    else if (type == CommandType::Parallel)
    {
        bool temp_bool = true;
        for (std::function<bool()> func : commands_to_run)
        { // might want to remove the function from the list when returns true
            bool function_bool = func();
            temp_bool = temp_bool && function_bool;
        }
        condition = temp_bool;
    }
    else if (type == CommandType::ParallelRace)
    {
        for (std::function<bool()> func : commands_to_run)
        {
            bool function_bool = func();
            condition = condition || function_bool;
        }
    }
    return condition;
}