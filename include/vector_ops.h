#ifndef DPAPPROX_VECTOR_OPS_H
#define DPAPPROX_VECTOR_OPS_H

#include <vector>
#include "config.h"


namespace DPapprox {
    std::vector<double> operator+(const std::vector<double>&, const std::vector<double>&);
    std::vector<double> operator-(const ProblemConfig::vtype&, std::vector<double>);
    std::vector<double> operator*(const std::vector<double>&, double);
}
#endif //DPAPPROX_VECTOR_OPS_H
