#include "logger.h"

Logger::Logger(LogLevel level) : level(level) {}

Logger::~Logger() {
    if (level >= LOG_LEVEL) {
        std::cout << std::endl;
    }
}
