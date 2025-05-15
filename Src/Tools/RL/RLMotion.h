/*
    Handles interfacing with the walk controller for bhuman
    Allows clipping and movement factor updates
*/

#ifndef RLMOTION_H
#define RLMOTION_H

#include <vector>
#include <deque>
#include <numeric>

class RLMotion {
    private:
    std::vector<float> walkAction;
    std::deque<float> previousAngles;
    std::vector<float> movementCoefficients;
    std::vector<float> clips;

    public:
    RLMotion();

    std::vector<float> ClipWalkAction(std::vector<float> velocities);
    void updateWalkAction(std::vector<float> action);
    std::vector<float> getWalkAction();
};

#endif /* RLMOTION_H */
