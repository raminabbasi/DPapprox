# DPapprox

Dynamic Programming-inspired algorithm for approximation of discrete inputs based on the continuous relaxed solution to a Mixed-Integer Optimal Control Problem (MIOCP). It is able to produce different approximation strategies through definition of its cost-to-go. General dwell time constraints can be included as well.


## Building DPapprox

```
git clone https://github.com/raminabbasi/DPapprox.git
cd DPapprox
mkdir build && cd build
cmake ..
make
```
and if you wish to install DPapprox
```
make install
```

## Building Examples
Examples can be built independently:
```
cd ..
cd examples
mkdir build && cd build
cmake ..
make
```
### Running examples
Each program comes with its own data (relaxed solution) in the `data` folder. To run `trj` example
```
./trj ../data/trj
```
The results are also saved in the `data` folder. 
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

* `running_cost(vi, ri, i, dt)`: A vector function that returns the running cost of approximation as a function of the discrete input `vi`, the relaxed value `ri`, time node `i`, and time step `dt`. 
  * `|v_i - r_i|` provides Simple Rounding.
  * `v_i - r_i` provides SumUp Rounding for single or multiple inputs. 

* `sort_key(ci)`: A real function that receives cost of approximation and returns an objective for optimization. 
  * `ci` is used for Simple Rounding.
  * `|ci|` is used for SumUp Rounding of single inputs.
  * `||ci||_\infty`  is used for SumUp Rounding with multiple inputs with SOS1 condition. 

* `dwell_time_cons`: A two dimensional vector that includes pairs of discrete input values and minimum dwell time for them. 
  * `{{{0}, {0.5, 0.5}} , {{0,1}, {0.2, 0.2}}}` defines two constraints:
    * on the value 0, a minimum dwell time of 0.5 for the first and the second discrete input.
    * on the sequence (0, 1) , a minimum dwell time of 0.2 for the first and the second discrete input.

* `dwell_time_init`: A two dimensional vector of optional value to set the starting value of timers for dwell_time_cons. Default is zero. (must have the same size as `dwell_time_cons`).

### ProblemConfig (experimental)
DPapprox can also keep track of system states. To do so, we set

* `next_state_f(xi, vi, i, dt)`: A vector function integrator that receives `xi`, discrete input `vi`, time node `i`, and time step `dt` , and provides the next state `xni` . 
  * `f(xi, t_i)*dt + xi` provides the explicit Euler integrator.
* `dynamic_cost(xi, vi, i, dt)`: A vector function that returns a dynamic cost as a function of the state `xi` of the system.
  * `max {0, h(x_i, t_i)}` can be used as a penalty function for a constraint `h(x_i, t_i) <= 0`.
  * Returning `\infty` penalty for a state constraint violation enforces that constraint. 
* `include_state`: A boolean to indicate whether DPapprox should include states or not. Default is `false`.

## Usage
Below is a minimum working example.

```c++
#include "DPapprox.h"
#include <vector>

using namespace DPapprox;

int main(){

    ProblemConfig config;
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
```


## Contributing
Contributions and feedback are welcomed!

## License
This software is under GPL-3.0 license, please check [LICENSE](https://github.com/raminabbasi/DPapprox/blob/main/LICENSE) for more details.