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

namespace DPapprox {

enum LogLevel { DEBUG, INFO, WARNING, ERROR, NONE };

class Logger {
public:
    static Logger& instance() {
        static Logger instance;
        return instance;
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void setThreshold(LogLevel level) {
        threshold = level;
    }

    LogLevel getThreshold() const {
        return threshold;
    }

    Logger& log(LogLevel level) {
        currentMsgLevel = level;
        return *this;
    }

    template <typename T>
    Logger& operator<<(const T& msg) {
        if (currentMsgLevel >= threshold) {
            std::cout << msg;
        }
        return *this;
    }

    Logger& operator<<(std::ostream& (*manip)(std::ostream&)) {
        std::cout << manip;
        return *this;
    }

private:
    Logger() : threshold(DEBUG), currentMsgLevel(DEBUG) {}

    LogLevel threshold;
    LogLevel currentMsgLevel;
};

inline Logger& Log = Logger::instance();
}

#endif
