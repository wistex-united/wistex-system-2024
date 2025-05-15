
#ifndef RL_REWARDS
#define RL_REWARDS

#include <vector>
#include <string>

float getReward(
    const std::vector<float> cur_agent_loc,
    const std::vector<float> prev_agent_loc,
    const std::vector<float> cur_ball_loc,
    const std::vector<float> prev_ball_loc,
    const std::vector<float> action,
    const bool isGoal,
    const bool ballIsOutOfFieldBounds,
    const bool ballIsInGoalArea,
    const bool ballIsInPenaltyArea,
    const int framesSinceLastReward
);

#endif /* RL_REWARDS */
