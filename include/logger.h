/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

namespace DPapprox{


enum LogLevel { DEBUG, INFO, WARNING, ERROR, NONE };

#ifndef LOG_LEVEL
#define LOG_LEVEL DEBUG
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

}

#endif 
