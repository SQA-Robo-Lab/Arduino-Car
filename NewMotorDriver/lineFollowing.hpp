#ifndef LINE_FOLLOWING_HPP
#define LINE_FOLLOWING_HPP

#include "src/driveController/DriveController.hpp"
#include "src/lineDetector/LineDetector.hpp"
#include "src/distanceSensor/DistanceSensor.hpp"

void lineFollowing(DriveController &dc, LineDetector &ld, DistanceSensor &distFront, DistanceSensor &distRear);

#endif