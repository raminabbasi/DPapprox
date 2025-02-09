#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

enum LogLevel { DEBUG, INFO, WARNING, ERROR, NONE };

#ifndef LOG_LEVEL
#define LOG_LEVEL DEBUG  // Default log level (can be overridden at compile time)
#endif

class Logger {
public:
    Logger(LogLevel level);
    ~Logger();

    template <typename T>
    Logger& operator<<(const T& msg) {
        if (level >= LOG_LEVEL) {
            std::cout << msg;
        }
        return *this;
    }

private:
    LogLevel level;
};

#define Log(level) Logger(level)

#endif // LOGGER_H
