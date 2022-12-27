#include <iostream>
#include <vector>
#include <array>

#include "matplotlibcpp.h"
#include "eigen3/Eigen/Geometry"

#include "swarmAgent.hpp"
#include "environmentManager.hpp"

namespace plt = matplotlibcpp;

int main() {
    std::cout << "Hello World!" << "\n";

    plt::plot({1,3,2,4});
    plt::show();

    return 0;
}