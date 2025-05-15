#include "RLRewards.h"
#include "RLObs.h"
#include <vector>


float normAngle180(float angle) {
    return fmod(angle + 180.0f, 360.0f) - 180.0f;
}

float angleBetween(std::vector<float> agentLoc, std::vector<float> ballLoc) {
    // agentLoc is x, y, angle
    // ballLoc is x, y
    float x = ballLoc[0] - agentLoc[0];
    float y = ballLoc[1] - agentLoc[1];
    float angleBetweenRobotBodyAndBall = std::atan2(y, x) - agentLoc[2];
    angleBetweenRobotBodyAndBall *= 180.0f / Constants::pi;
    angleBetweenRobotBodyAndBall = fmod(angleBetweenRobotBodyAndBall, 360.0f);
    return angleBetweenRobotBodyAndBall;
}

float getAngleBetweenAgentAndBall(std::vector<float> agentLoc, std::vector<float> ballLoc) {
    float angle = angleBetween(agentLoc, ballLoc);
    return normAngle180(angle);
}

float clip(float min, float max, float val) {
    return std::max(min, std::min(max, val));
}

// Measures how well the agent is lined up with the ball and the goal.
// Returned angle is between 0 and 180. Closer to 0 means more lined up.
float getLineupAngleDifference(const std::vector<float>& agentLoc, const std::vector<float>& ballLoc) {
    float angleAgentBall = angleBetween(agentLoc, ballLoc);
    float angleAgentGoal = angleBetween(agentLoc, {4500, 0});
    return std::abs(normAngle180(angleAgentBall - angleAgentGoal));
}

// Returns distance between ball and end line in millimeters.
// If ball's y-coordinate is aligned with the goal, returns 0.
float getBallEndlinePotential(const std::vector<float>& ballLoc) {
    float absBallX = std::abs(ballLoc[0]);
    float absBallY = std::abs(ballLoc[1]);
    if (absBallY < 800) {
        // ball is aligned with the goal
        return 0;
    }

    // Linearly decrease reward as ball gets closer to the end line.
    // No reward penalty if the ball's x coord is 3900 or less.
    // 1 means ball is at the end line, 0 means ball is at least 3900 mm away from the end line.
    // larger value is worse.
    return std::max(0.0f, (absBallX - 3900)/ (4500 - 3900));
}

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
) {
    // weights
    float weight_dist_to_ball = 0.5 / std::max(1, framesSinceLastReward);
    float weight_ball_to_goal = 10 / std::max(1, framesSinceLastReward);
    float weight_body_pointed_to_ball = 2;
    float weight_goal = 1000;
    float weight_ball_oob = -2000;
    float weight_ball_outside_penalty_area = -100;
    float weight_time = -0.1;
    float weight_lined_up = 1;
    float weight_ball_away_from_endline = 10;
    float weight_ball_near_endline = -1;
    float weight_want_to_kick = -0.1;
    float weight_want_to_kick_but_cant = -1;
    float weight_action_turn = -0.1;
    float weight_unused_action = -0.1;

    float reward = 0;

    // reward getting closer to the ball
    float prev_dist = dist(prev_agent_loc, prev_ball_loc);
    float cur_dist = dist(cur_agent_loc, cur_ball_loc);
    reward += weight_dist_to_ball * (prev_dist - cur_dist);

    // reward facing the ball (robot's body, not head)
    float angle_to_ball = std::abs(getAngleBetweenAgentAndBall(cur_agent_loc, cur_ball_loc));
    float tolerance = 10;
    float max_angle_allowed = 40;
    reward += weight_body_pointed_to_ball * clip(
        0, 1, 1 - (angle_to_ball - tolerance) / (max_angle_allowed - tolerance)
    );

    if (cur_agent_loc[0] > cur_ball_loc[0]) {
        reward += -5;
    }

    // reward lining up with the goal; No reward if the agent is already lined up within 15 degrees
    float prev_lineup_gap = getLineupAngleDifference(prev_agent_loc, prev_ball_loc);
    float cur_lineup_gap = getLineupAngleDifference(cur_agent_loc, cur_ball_loc);
    if (prev_lineup_gap > 15 || cur_lineup_gap > 15) {
        float gap_reduction = prev_lineup_gap - cur_lineup_gap;
        reward += weight_lined_up * gap_reduction;
    }

    // reward moving the ball toward the goal
    std::vector<float> goal_post_left{4500, 800};
    std::vector<float> goal_post_right{4500, -800};
    float prev_dist_avg = (
        dist(prev_ball_loc, goal_post_left) +
        dist(prev_ball_loc, goal_post_right)
    ) / 2;
    float cur_dist_avg = (
        dist(cur_ball_loc, goal_post_left) +
        dist(cur_ball_loc, goal_post_right)
    ) / 2;
    reward += weight_ball_to_goal * (prev_dist_avg - cur_dist_avg);

    // punish for ball being close to the endline, and
    // reward for moving the ball away from the endline
    float ball_endline_cur = getBallEndlinePotential(cur_ball_loc);
    float ball_endline_prev = getBallEndlinePotential(prev_ball_loc);
    reward += weight_ball_near_endline * ball_endline_cur;
    // Note: we do perv - cur because we want this number to get smaller, and it's
    // intuitive to keep the weight positive.
    reward += weight_ball_away_from_endline * (ball_endline_prev - ball_endline_cur);

    // reward for turning to prevent thrashing angular velocities
    reward += weight_action_turn * std::pow(action[2], 2);

    if (isGoal) {
        reward += weight_goal;
    } else if (ballIsOutOfFieldBounds) {
        reward += weight_ball_oob;
    } else if (!ballIsInPenaltyArea) {
        reward += weight_ball_outside_penalty_area;
    }

    reward += weight_time;

    return reward;
}
