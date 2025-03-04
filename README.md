# DPapprox

DPapprox (`DP approximation') is a dynamic programming-inspired algorithm for approximating discrete inputs based on the continuous relaxation of a Mixed-Integer Optimal Control Problem (MIOCP). It supports customizable approximation strategies through flexible cost-to-go definitions and handles general dwell time constraints.

## Building DPapprox

```
git clone https://github.com/raminabbasi/DPapprox.git
cd DPapprox
mkdir build && cd build
cmake .. [-DWITH_EXAMPLES=ON] [-DWITH_TESTS=ON]
make
```
You can use CMake flags `-DWITH_EXAMPLES=ON` to build examples, and if you are building with examples, you can use `-DWITH_TESTS=ON` to add tests for them. If you wish to install DPapprox:

```
make install
```
### Running examples
Each program comes with its own data (relaxed solution) in the `data` folder. To run `trj` example
```
./trj <path/to/data>/trj.csv [<output>]
```
If you wish to save the results, specify an `<output>` file. 
## Docs

To use DPapprox, we need two ingredients:

1. A problem definition using `ProblemConfig` structure.
2. A wide two-dimensional vector as the relaxed solution.

### ProblemConfig
We will refer to the relaxed solution as `v_rel` from now on. When configuring a `ProblemConfig`, the following fields must be set [example]:

* `N` : Number of time discretization nodes (must be equal to `v_rel[0].size()`).
* `dt`: Fixed time step. 
* `v_feasible`: A two dimensional vector that indicates possible values of the discrete values, which are vectors themselves (`v_feasible.size()` must be equal to `N`, i.e., for each time node we need to provide a set of possible values)
  *  `{{{1}, {2}} , {{1}, {2}}, ...}` defines  a system with a single discrete input with two values of 1 and 2.

* `stage_cost(vi, ri, i, dt)`: A vector function that returns the running/stage cost of approximation as a function of the discrete input `vi`, the relaxed value `ri`, time node `i`, and time step `dt`. 
  * `|v_i - r_i|` provides Simple Rounding. *[Default]*
  * `v_i - r_i` provides SumUp Rounding for single or multiple inputs. 

* `objective(ci)`: A real function that receives cost of approximation and returns an objective for optimization. 
  * `c_i` is used for Simple Rounding. *[Default]*
  * `|c_i|` is used for SumUp Rounding of single inputs.
  * `|c_i|_\infty`  is used for SumUp Rounding with multiple inputs with SOS1 condition. 

* `dwell_time_cons`: A two dimensional vector that includes pairs of discrete input values and minimum dwell time for them. 
  * `{{{0}, {0.5, 0.5}} , {{0,1}, {0.2, 0.2}}}` defines two constraints:
    * on the value 0, a minimum dwell time of 0.5 for the first and the second discrete input.
    * on the sequence (0, 1) , a minimum dwell time of 0.2 for the first and the second discrete input.

* `dwell_time_init`: A two dimensional vector of optional value to set the starting value of timers for dwell_time_cons. Default is zero. (must have the same size as `dwell_time_cons`).

### ProblemConfig (experimental)
DPapprox can also keep track of system states. To do so, we set

* `include_state`: A boolean to indicate whether DPapprox should include states or not. Default is `false`.
* `state_transition(xi, vi, i, dt)`: A vector function integrator that receives `xi`, discrete input `vi`, time node `i`, and time step `dt` , and provides the next state `xni` . 
  * `f(x_i, t_i) dt + x_i` provides the explicit Euler integrator.
  * `xi`: Zero dynamics. *[Default]*
* `state_cost(xi, vi, i, dt)`: A vector function that returns a dynamic cost as a function of the state `xi` of the system.
  * `max {0, h(xi, t_i)}` can be used as a penalty function for a constraint `h(xi, t_i) <= 0`.
  * Returning `\infty` penalty for a state constraint violation enforces that constraint. 
  * `(xi - r_i)**2` penalty can be used for tracking a reference. 
  * `{0}`: Zero cost. *[Default]*

## Usage
Below is a minimum working example.

```c++
#include "DPapprox.h"
#include <vector>

int main(){

    DPapprox::ProblemConfig config;
    std::vector<std::vector<double>> v_rel{{0.2, 0.8, 0.4}};

    config.N = 3;
    config.v_feasible = {{{0}, {1}}, {{0}, {1}}, {{0}, {1}}};

    DPapprox::Solver solver(v_rel, config);
    solver.solve();

    for (auto& v : solver.solution.optimum_path){
        std::cout << v[0] << " ";
    }
    std::cout << std::endl;
    return 0;
}
```

with the output
```
Initializing Solver.
Solving...
Solved.
0 1 0 
```
## Citing
```
@software{DPapprox,
author = {Abbasi-Esfeden, Ramin},
license = {GPL-3.0},
month = Feb,
title = {DPapprox: Dynamic Programming-inspired discrete approximation algorithm},
url = {https://github.com/raminabbasi/DPapprox},
version = {0.1.0},
year = {2025}
}

@preprint{AbbasiEsfeden2024,
  title = {A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints},
  url = {http://dx.doi.org/10.2139/ssrn.5043263},
  DOI = {10.2139/ssrn.5043263},
  publisher = {Elsevier BV},
  author = {Abbasi Esfeden,  Ramin and Plate,  Christoph and Sager,  Sebastian and Swevers,  Jan},
  year = {2024}}
```


## Contributing
Contributions and feedback are welcomed!

## License
This software is under GPL-3.0 license, please check [LICENSE](https://github.com/raminabbasi/DPapprox/blob/main/LICENSE) for more details.