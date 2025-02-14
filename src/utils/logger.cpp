#include "logger.h"

namespace DPapprox {
    Logger::Logger(LogLevel level) : level(level) {}

    Logger::~Logger() {
        if (level >= LOG_LEVEL) {
            std::cout << std::endl;
        }
    }
}