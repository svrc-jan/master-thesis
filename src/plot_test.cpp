#include <iostream>

#include "utils/matplotlibcpp.h"

#include "tello/joystick.h"
#include "tello/tello.h"
#include "tello/timeutils.h"

namespace plt = matplotlibcpp;
int main() {
    plt::plot({1,3,2,4});
    plt::show();
}

