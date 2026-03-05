#ifndef COMMAND_AUTO
#define COMMAND_AUTO
#include <functional>
#include <vector>

enum class CommandType
{
    Parallel,     // 0
    ParallelRace, // 1
    Sequence      // 2
};

class Command
{
public:
    // Uses lambdas or functions to run later in commands.
    // This does not work with unique pointers I believe and would cause problems when copying unique pointers
    CommandType type = CommandType::Sequence;
    std::vector<std::function<bool()>> commands_to_run;
    bool condition = false;
    
    Command(std::function<bool()> operation);
    Command(std::function<bool()> operation1, std::function<bool()> operation2, CommandType commandType);
    Command(std::vector<std::function<bool()>> operations, CommandType commandType);
    bool run();
};
#endif
