#include "RLMotion.h"
#include "RLVars.h"

#include <iostream>
#include <string>

#include <cmath>
#include <vector>
#include <algorithm>

RLMotion::RLMotion() {
    // Format: {x, y, angle}
    walkAction = {0, 0, 0};
    previousAngles = {};
}

std::vector<float> RLMotion::ClipWalkAction(std::vector<float> action) {
    std::vector<float> actionScales = {1.8f, 0.8f, 1.2f};
    // scale action
    for (size_t v = 0; v < action.size(); ++v) {
        action[v] *= actionScales[v];
    }

    return action;
}

void RLMotion::updateWalkAction(std::vector<float> action) {
    std::vector scaledActions = ClipWalkAction(action);


    walkAction[0] = scaledActions[0];
    walkAction[1] = scaledActions[1];

     // Update previousAngles
    if (previousAngles.size() >= 4) { // Assuming the fixed size is 4
        previousAngles.pop_front(); // Remove the oldest element efficiently
    }
    previousAngles.push_back(scaledActions[2]); // Add the new angle

    // make walkAction[2] the average of previousAngles
    walkAction[2] = std::accumulate(previousAngles.begin(), previousAngles.end(), 0.0) / previousAngles.size();
}

std::vector<float> RLMotion::getWalkAction() {
    return walkAction;
}
