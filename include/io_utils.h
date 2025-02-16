#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <vector>
#include <string>
#include "config.h"


std::vector<std::vector<double>> read_csv(const std::string& filename);


void write_csv(const std::string& filename, const std::vector<ProblemConfig::disc_vector>& data);


std::vector<double> get_column(const std::vector<std::vector<double>>& v, size_t col_index);
#endif 
