#include "vex.h"
#include "Lock.hpp"
#include "PurePursuit.hpp"
#include <vector>

using namespace vex;

PurePursuit::PurePursuit(SixWheelDrive& driveBase, RobotState& state) : driveBase(driveBase), state(state) {}

int PurePursuit::signOf(double x) {
    if (x >= 0) {
        return 1;
    } else {
        return -1;
    }
}

double PurePursuit::degSin(double deg) {
    double rad = deg * M_PI / 180;
    return sin(rad);
}

double PurePursuit::degaTan(double in) {
    double deg = atan(in) * 180 / M_PI;
    return deg;
}

double PurePursuit::distBetween(Coord a, Coord b) {
    double diffX = a.x - b.x;
    double diffY = a.y - b.y;
    return sqrt(pow(diffX, 2) + pow(diffY, 2));
}

double PurePursuit::getAngleDifference(double ref, double targ) {
    if (fabs(targ-ref) <= 180) {
        return targ - ref;
    } else {
        if (targ-ref > 0) {
            return targ-ref-360;
        } else {
            return targ-ref+360;
        }
    }
}

PPFeedBack PurePursuit::PPStep(std::vector<Coord> path, Coord currState, double lookAheadDist, int LFoundInd) {
    PPFeedBack feedback;

    int lastFoundIndex = LFoundInd;
    bool intersectFound = false;
    int startingIndex = lastFoundIndex;

    for (int i = startingIndex; i < path.size() - 1; i++) {
        double x1 = path[i].x - currState.x;
        double y1 = path[i].y - currState.y;
        double x2 = path[i+1].x - currState.x;
        double y2 = path[i+1].y - currState.y;
        double m = (y2 - y1)/(x2 - x1);
        double b = y1 - m * x1;
        double r = lookAheadDist;
        double A = pow(m,2) + 1;
        double B = 2 * m * b;
        double C = pow(b, 2) - pow(r, 2);

        double discriminant = pow(B, 2)- 4*A*C;
        if (discriminant < 0) {
            intersectFound = false;
            feedback.coord = path[lastFoundIndex+1];
            continue;
        } else {
            Coord solution;
            Coord sol1;
            Coord sol2;
            sol1.x = (-1 * B + sqrt(discriminant))/(2 * A);
            sol1.y = m * sol1.x + b;
            sol2.x = (-1 * B - sqrt(discriminant))/(2 * A);
            sol2.y = m * sol2.x + b;

            double maxX = std::max(path[i].x, path[i+1].x);
            double minX = std::min(path[i].x, path[i+1].x);
            double maxY = std::max(path[i].y, path[i+1].y);
            double minY = std::min(path[i].y, path[i+1].y);

            maxX = maxX - currState.x;
            minX = minX - currState.x;
            maxY = maxY - currState.y;
            minY = minY - currState.y;

            bool sol1Valid;
            bool sol2Valid;

            if (sol1.x < maxX && sol1.x > minX && sol1.y < maxY && sol1.y > minY) {
                sol1Valid = true;
            } else {
                sol1Valid = false;
            }

            if (sol2.x < maxX && sol2.x > minX && sol2.y < maxY && sol2.y > minY) {
                sol2Valid = true;
            } else {
                sol2Valid = false;
            }

            Coord offSetI1;
            offSetI1.x = path[i+1].x - currState.x;
            offSetI1.y = path[i+1].y - currState.y;

            if (sol1Valid && sol2Valid) {
                if (distBetween(sol1, offSetI1)<distBetween(sol2, offSetI1)) {
                    solution = sol1;
                } else {
                    solution = sol2;
                }
            } else if (sol1Valid) {
                solution = sol1;
            } else if (sol2Valid) {
                solution = sol2;
            } else {
                intersectFound = false;
                feedback.coord = path[lastFoundIndex+1];
                continue;
            }

            if (distBetween(currState, offSetI1) >= distBetween(solution, offSetI1)) {
                intersectFound = true;
                feedback.coord.x = solution.x + currState.x;
                feedback.coord.y = solution.y + currState.y;
                lastFoundIndex = i;
                break;
            } else {
                intersectFound = false;
                feedback.coord = path[lastFoundIndex+1];
                break;
            }
        }
    }

    
    feedback.turnError = calcTurnError(currState, feedback.coord);
    feedback.intersectFound = intersectFound;
    feedback.lastFoundIndex = lastFoundIndex;
    return feedback;
}

double PurePursuit::calcTurnVel(double linearVel, double lookAheadDist, double degError) {
    double turnRadius = (lookAheadDist/2)/degSin(degError);
    return driveBase.baseWidth / (2 * turnRadius) * linearVel;
}

double PurePursuit::calcTurnError(Coord currState, Coord targetPos) {
    Coord offSetSol;
    offSetSol.x = targetPos.x - currState.x;
    offSetSol.y = targetPos.y - currState.y;

    double angle = degaTan(offSetSol.x/offSetSol.y);
    if (angle < 0) {
        angle += 360;
    }

    double angleDiff = 0;
    if (currState.hasHeading) {
        angleDiff = getAngleDifference(currState.heading, angle);
    } else {
        printf("Issue in calcTurnError, input currState does not have a heading");
    }
    return angleDiff;
}

void PurePursuit::followPath(std::vector<Coord> path) {
    int index = 0;
    while (true) {
        if (index >= path.size() - 2) {
            break;
        }
        Coord currState();
        currState.x = 
        PPStep(path, )
    }

}