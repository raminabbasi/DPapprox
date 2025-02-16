#include "vector_ops.h"
#include <stdexcept>

namespace DPapprox {
    
    std::vector<double> operator+(const std::vector<double> &c, const std::vector<double> &d) {
        if (c.size() == d.size()) {
            
            std::vector<double> result(c.size());
            for (size_t i = 0; i < c.size(); ++i) {
                result[i] = c[i] + d[i];
            }
            return result;
        } else if (c.size() == 1) {
            
            std::vector<double> result(d.size());
            for (size_t i = 0; i < d.size(); ++i) {
                result[i] = c[0] + d[i];
            }
            return result;
        } else if (d.size() == 1) {
            
            std::vector<double> result(c.size());
            for (size_t i = 0; i < c.size(); ++i) {
                result[i] = c[i] + d[0];
            }
            return result;
        } else {
            throw std::runtime_error("Vector addition error: incompatible sizes.");
        }
    }

    
    std::vector<double> operator-(const ProblemConfig::disc_vector &a, const std::vector<double>& b) {
        if (a.size() != b.size()) {
            throw std::runtime_error("Vector subtraction error: size mismatch.");
        }

        std::vector<double> result(a.size());
        for (size_t i = 0; i < a.size(); ++i) {
            result[i] = a[i] - b[i];
        }
        return result;
    }

    
    std::vector<double> operator*(const std::vector<double> &vec, double scalar) {
        std::vector<double> result(vec.size());
        for (size_t i = 0; i < vec.size(); ++i) {
            result[i] = vec[i] * scalar;
        }
        return result;
    }
}
