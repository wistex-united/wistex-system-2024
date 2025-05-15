/*
 *
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author BadgerRL
 */

#include "Tools/RL/RLAlg.h"
#include "Tools/RL/RLObs.h"
#include "Tools/RL/RLVars.h"
#include "Tools/RL/RLMotion.h"
#include "Tools/RL/RLMath.h"

#include <onnxruntime_cxx_api.h>

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Modules/BehaviorControl/SkillBehaviorControl/Helpers/configUtils.h"

#include "Streaming/InStreams.h"
#include "Streaming/Output.h"

#include "Tools/Modeling/Obstacle.h"
#include "Debugging/DebugDrawings.h"
#include "Libs/Debugging/ColorRGBA.h"

#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"

#include <cmath>
#include "Tools/BehaviorControl/Strategy/PositionRole.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <filesystem>
#include <vector>
#include <Eigen/Dense>
#include <map>
#include <sys/stat.h>

#undef state
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

#define PI 3.14159265

std::vector<int> robotNum;

std::map<int, int> robot_time;
std::map<int, int> ballsearch_time;


bool directoryExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Cannot access path
        return false;
    } else if (info.st_mode & S_IFDIR) {
        // Path exists and is a directory
        return true;
    } else {
        // Path exists but is not a directory
        return false;
    }
}
const std::string POLICY_PATH = directoryExists("/home/nao/Config/") ? "/home/nao/Config/Policies/" : "../Policies/";
const std::string RL_CFG = directoryExists("/home/nao/Config/") ? "/home/nao/Config/rl.cfg" : "../rl.cfg";
const std::string STRATEGY_CFG = directoryExists("/home/nao/Config/") ? "/home/nao/Config/strategy.cfg" : "../strategy.cfg";

std::vector<float> target;

// inital angle of the target pose
float target_angle = 0;

// radius around ball where the target pose is constrained
float action_radius = 350;

// how fast the target pose can rotate around the ball
float angle_disp_coef = .05;

bool fix_kick_angle = false;

const std::vector<float> OPP_GOAL = {4500, 0};
const std::vector<float> OUR_GOAL = {-4500, 0};

std::map<std::string, std::string> config = parse_cfg_file(RL_CFG);
std::map<int, std::vector<float>> strategy_positions = parse_strategy_file(STRATEGY_CFG);




std::map<int, std::string> robot_roles = {
    {1, "goalie"},
    {2, "off_ball"},
    {3, "off_ball"},
    {4, "off_ball"},
    {5, "on_ball"}
};

// robot home location (x, y)
std::map<int, std::vector<float>> robot_home = {
    {1, {-2.25, -1.5}},
    {2, {-2.25, 1.5}},
    {3, {2.25, -1.5}},
    {4, {0, 0}},
    {5, {0, 0}}
};

std::vector<float> normalizeVector(float x, float y) {
    float magnitude = std::hypot(x, y);
    return {x / magnitude, y / magnitude};
}

std::vector<float> transformToGlobalCoordinates(const std::vector<float>& local_position, const std::vector<float> robot_pos) {
        float global_x = local_position[0] * cos(robot_pos[2]) - local_position[1] * sin(robot_pos[2]) + robot_pos[0];

        float global_y = local_position[0] * sin(robot_pos[2]) + local_position[1] * cos(robot_pos[2]) + robot_pos[1];
        return {global_x, global_y};
}

// values being unloaded from the config file
float role_switch_distance = std::stof(config["role_switch_distance"]);
bool use_closest_input_vector = stringToBool(config["use_closest_input"]);
bool use_agent_id_vector = stringToBool(config["use_agent_id"]);
bool use_home_location = stringToBool(config["use_home_location"]);
std::string policy_input_type = config["policy_folder"];
std::string policy_name = config["policy_name"];
int role_switch_interval = std::stoi(config["role_switch_interval"]);
bool use_near_opp_goal_policy = stringToBool(config["use_near_opp_goal_policy"]);
float near_opp_goal_x_if_aligned = std::stof(config["near_opp_goal_x_if_aligned"]);
float near_opp_goal_x = std::stof(config["near_opp_goal_x"]);
float near_opp_goal_y = std::stof(config["near_opp_goal_y"]);
bool use_neural_goalie = stringToBool(config["use_neural_goalie"]);
int kickoff_setplay_duration_ms = std::stoi(config["kickoff_setplay_duration_ms"]);

int S = display_map(config);

float front_opponent_angle_threshold = PI/5;
float front_opponent_dis_threshold = 350;
float anywhere_opponent_dis_threshold = 350;

std::map<int, int> time_of_our_most_recent_kickoff = {
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0},
    {5, 0}
};

// maps self's robotNumber to the a vector with the following elements:
// 1. robotNumber that self believes is the kickoff forward robot
// 2. the number of teammates that were available at the time when self made the decision
std::map<int, std::vector<int>> kickoff_forward_robot_global = {
    {1, {-1, 0}},
    {2, {-1, 0}},
    {3, {-1, 0}},
    {4, {-1, 0}},
    {5, {-1, 0}}
};


// one hot encoding for player number (3 players supported)
std::map<int, std::vector<float>> player_to_one_hot= {
    {1, {1, 0, 0}},
    {2, {0, 1, 0}},
    {3, {0, 0, 1}},
    {4, {0, 0, 0}},
    {5, {0, 0, 0}}
};


// is the robot the closest to the ball?
std::map<int, std::vector<float>> closest_input_one_hot = {
    {1, {0, 1}},
    {2, {0, 1}},
    {3, {0, 1}},
    {4, {0, 1}},
    {5, {0, 1}}
};

// Assigns an integer value to each policy_type, used to set LEDs.
// TODO: use a real enum
std::map<std::string, int> policy_indices = {
    // If you change this mapping, make sure to change the LEDHandler accordingly!
    {"goalie", 0},
    {"off_ball", 111},
    {"on_ball", 222},
    {"near_opp_goal", 333}
};
// Goalie Search
enum GoalieState {
    GOALIE_LOCAL_SEARCH_RIGHT,
    GOALIE_LOCAL_SEARCH_LEFT,
    GOALIE_LOOK_FORWARD,
    GOALIE_MOVE_CENTRE,
    GOALIE_WALK_BACK,
    GOALIE_STAND_STILL
};

Policy policy;
Policy goalie_policy;
Policy on_ball_policy;
Policy off_ball_policy;
Policy near_opp_goal_policy;
Policy on_ball_dribble_policy;

int goalie_search_state = GOALIE_LOCAL_SEARCH_RIGHT;

RLMotion walkMotion;

float last_ball_to_robot;
bool kicking = false;

// Step 1: add direct
// Step 2: add all tangents
// Step 3: find most
// Step 4: Kick / Move



std::vector<float> calculateAvoidanceDirection(const std::vector<float>& agent_loc,
                                                const std::vector<float>& obstacle_loc) {
    float avoid_direction_x = obstacle_loc[0] - agent_loc[0];
    float avoid_direction_y = obstacle_loc[1] - agent_loc[1];
    return normalizeVector(avoid_direction_x, avoid_direction_y);
}

float crossProduct(const std::vector<float>& a, const std::vector<float>& b) {
    return a[0] * b[1] - a[1] * b[0];
}

std::vector<float> transformRotationGlobal(const float robot_rotation) {
    float direction_x = std::cos(robot_rotation);
    float direction_y = std::sin(robot_rotation);
    return normalizeVector(direction_x, direction_y);
}

float calculateAngleBetweenVectors(const std::vector<float>& a, const std::vector<float>& b) {
    float dot_product = a[0] * b[0] + a[1] * b[1];
    float magnitude_a = sqrt(a[0] * a[0] + a[1] * a[1]);
    float magnitude_b = sqrt(b[0] * b[0] + b[1] * b[1]);
    return acos(dot_product / (magnitude_a * magnitude_b));
}

bool isAlignedWithGoal(std::vector<float> self, std::vector<float> ball) {
    if (std::abs(ball[0] - self[0]) < 1) {
        // Avoid division by zero
        return false;
    }
    // Calculate the slope of the line formed by self and ball
    float m = (ball[1] - self[1]) / (ball[0] - self[0]);

    // Calculate the y-intercept of the line formed by self and ball
    float c = self[1] - m * self[0];

    // Calculate the x-coordinate of the intercept with the line y=4500, which is the goal line
    float intercept_goal_line = 4500 * m + c;

    // Check if the intercept is between the x-coordinates of self and ball.
    // Note: the goal post y position is +/- 800 but this function is conservative by using 600
    return std::abs(intercept_goal_line) < 600 && self[0] < ball[0];
}

bool shouldUserNearOppGoalPolicy(std::vector<float> self, std::vector<float> ball) {
    // Only call this if the robot has the OnBall policy after role switching logic.
    if (!use_near_opp_goal_policy) {
        // config says not to disable the NearOppGoal policy
        return false;
    }
    if (dist(self, ball) > 1000) {
        // robot is too far from the robot
        return false;
    }
    if (std::abs(ball[1]) > near_opp_goal_y) {
        // ball is not between goal posts in the y direction
        return false;
    }
    if (ball[0] > near_opp_goal_x) {
        // ball is in the goal area in the x direction
        return true;
    }
    if (ball[0] > near_opp_goal_x_if_aligned) {
        // ball is within penalty area in the x direction, so use the NearOppGoal
        // policy only if the robot and ball are aligned with the goal.
        return isAlignedWithGoal(self, ball);
    }
    return false;
}

bool will_be_blocked(
    std::vector<float> ball,
    float angle, // the abs. angle of the proposed kick
    const std::vector<std::vector<float>>& opponents
) {

    const float ball_radius = 50.f;
    const float opponent_radius = 100.f + 100.f;
    for (const auto& opponent : opponents) {

        float opponent_angle = std::atan2(opponent[1] - ball[1], opponent[0] - ball[0]);
        if (opponent_angle < -PI/2 || opponent_angle > PI/2) {
            continue;
        }
        float angle_diff = Angle::normalize(std::abs(angle - opponent_angle));

        float distance = std::sqrt(std::pow(opponent[0] - ball[0], 2) + std::pow(opponent[1] - ball[1], 2));
        float block_angle = std::asin((ball_radius + opponent_radius) / distance);

        if (angle_diff <= block_angle) {
            return true;
        }
    }
    return false;
}

float get_optimal_kick_angle(
    const std::vector<std::vector<float>>& opponents,
    std::vector<float> ball,
    float target_angle,
    float left_bound, // positive
    float right_bound, // negative
    float default_angle
) {
  const float angle_step = 0.03;
  for (int i = 0; ; ++i) {
    float left_angle = target_angle + i * angle_step;

    if (
        left_angle <= target_angle + left_bound
        && !will_be_blocked(ball, left_angle, opponents)
        )
      return left_angle;

    float right_angle = target_angle - i * angle_step;

    if (
        i > 0
        && right_angle >= target_angle + right_bound
        && !will_be_blocked(ball, right_angle, opponents)
        )
      return right_angle;

    if (
        left_angle > target_angle + left_bound
        && right_angle < target_angle + right_bound
        )
      break;
  }

  return default_angle;
}

bool shouldBeDoingKickoffSetPlay(const GameState& theGameState, const FrameInfo& theFrameInfo) {
    int timeSince = theFrameInfo.getTimeSince(time_of_our_most_recent_kickoff[theGameState.playerNumber]);
    return timeSince < kickoff_setplay_duration_ms;
}

int getRobotNumberClosestToPoint(
    const GameState& theGameState,
    const RobotPose& theRobotPose,
    const GlobalTeammatesModel& theGlobalTeammatesModel,
    std::vector<float> targetPoint,
    bool excludeGoalie
) {
    float min_dist = dist({theRobotPose.translation.x(), theRobotPose.translation.y()}, targetPoint);
    int closest_robot_number = theGameState.playerNumber;

    for(const auto& teammate : theGlobalTeammatesModel.teammates) {
        // do not include goalie
        if (excludeGoalie && teammate.playerNumber == 1) {
            continue;
        }

        float distance = dist(
            {teammate.pose.translation.x(), teammate.pose.translation.y()}, targetPoint
        );
        if (distance < min_dist) {
            min_dist = distance;
            closest_robot_number = teammate.playerNumber;
        }
    }
    return closest_robot_number;
}

bool isKickoffSetPlayRobot(const GameState& theGameState, const RobotPose& theRobotPose, const GlobalTeammatesModel& theGlobalTeammatesModel) {
    int numTeammatesAvailable = theGlobalTeammatesModel.teammates.size();

    if (
        theGameState.playerNumber == kickoff_forward_robot_global[theGameState.playerNumber][0] &&
        numTeammatesAvailable <= kickoff_forward_robot_global[theGameState.playerNumber][1]
    ) {
        // we've already decided that this robot is the kickoff forward robot for this kickoff, and we don't have more teammate info
        // available now than we did when we made that decision.
        return true;
    } else if (
        kickoff_forward_robot_global[theGameState.playerNumber][0] != -1 &&
        numTeammatesAvailable <= kickoff_forward_robot_global[theGameState.playerNumber][1]
    ) {
        // we've already decided the kickoff forward robot is someone else, and we don't have more teammate info
        // available now than we did when we made that decision.
        return false;
    }
    // Either this robot hasn't yet decided who is the kickoff forward robot, or it has access to more teammates
    // now and must re-evaluate.

    // Find the robot closest to this point that isn't the goalie and isn't the kicker
    int kicker_number = getRobotNumberClosestToPoint(theGameState, theRobotPose, theGlobalTeammatesModel, {0.f, 0.f}, true);

    float max_x = -99999;
    int best_robot_number = -1;
    for(const auto& teammate : theGlobalTeammatesModel.teammates) {
        // do not include goalie and kicker
        if (teammate.playerNumber == 1 || teammate.playerNumber == kicker_number)
            continue;

        // do not include robots on the left side of the kicker
        if (teammate.pose.translation.y() > 0) {
            continue;
        }

        float teammate_x = teammate.pose.translation.x();
        if (teammate_x > max_x) {
            max_x = teammate_x;
            best_robot_number = teammate.playerNumber;
        }
    }

    // consider self as a candidate for the kickoff forward robot
    if (
        theGameState.playerNumber != 1 &&
        theGameState.playerNumber != kicker_number &&
        theRobotPose.translation.y() < 0
    ) {
        float self_x = theRobotPose.translation.x();
        if (self_x > max_x) {
            max_x = self_x;
            best_robot_number = theGameState.playerNumber;
        }
    }

    if (kicker_number == best_robot_number) {
        // This shouldn't happen, but if it does, don't set the global kickoff_forward_robot_global variable
        // so we re-evaluate in the next frame, and just return false so we don't think this robot
        // is both the attacker and the forward.
        return false;
    }

    kickoff_forward_robot_global[theGameState.playerNumber][0] = best_robot_number;
    kickoff_forward_robot_global[theGameState.playerNumber][1] = numTeammatesAvailable;

    return kickoff_forward_robot_global[theGameState.playerNumber][0] == theGameState.playerNumber;
}


SKILL_IMPLEMENTATION(NeuralControlImpl, {,
    IMPLEMENTS(NeuralControl),
    REQUIRES(GameState),
    REQUIRES(LibWalk),
    REQUIRES(MotionInfo),
    REQUIRES(FieldBall),
    REQUIRES(RobotPose),
    REQUIRES(GlobalTeammatesModel),
    REQUIRES(GlobalOpponentsModel),
    REQUIRES(ObstacleModel),
    MODIFIES(BehaviorStatus),
    REQUIRES(TeamData),
    REQUIRES(StrategyStatus),
    REQUIRES(FieldDimensions),
    REQUIRES(BallSpecification),
    REQUIRES(PathPlanner),
    REQUIRES(FrameInfo),
    CALLS(LookForward),
    CALLS(LookActive),
    CALLS(LookAtAngles),
    CALLS(Stand),
    CALLS(PublishMotion),
    CALLS(WalkAtRelativeSpeed),
    CALLS(WalkToPose),
    CALLS(WalkToPoint),
    CALLS(WalkToBallAndKick),
    CALLS(GoToBallAndKick),
    CALLS(LookAtBall),
    CALLS(LookAtGlobalBall),
});

class NeuralControlImpl : public NeuralControlImplBase
{
private:

    void rotatePoint(float x, float y, float x1, float y1, float angle, float& xRot, float& yRot) {
        float rad = angle * M_PI / 180.0;  // Convert angle to radians
        xRot = (x - x1) * cos(rad) - (y - y1) * sin(rad) + x1;
        yRot = (x - x1) * sin(rad) + (y - y1) * cos(rad) + y1;
    }
    // ball search variable and helper methods setup
    // TODO TODO TODO TODO

    bool closest(float x, float y, auto self) {
        float my_distance = abs(std::hypot(self[0] - x, self[1] - y));
        for(const auto& teammate : theGlobalTeammatesModel
            .teammates) {
                float teammat_dis = abs(std::hypot(teammate.pose.translation.x() - x, teammate.pose.translation.y() - y));
                if (teammat_dis < my_distance) {
                    return false;
                }

                    }
        return true;
    }


    void kick_to_pos(float x, float y, auto self, int number) {
        float x_1 = theFieldBall.positionOnField.x();
        float y_1 = theFieldBall.positionOnField.y();
        float x_2 = x;
        float y_2 = y;
        float m = (y_2 - y_1) / (x_2 - x_1);
        float b = y_1 - m * x_1;
        float min_dis = 10000000;
        float final_m = m;
        float final_b = b;

        // Loop through all angles from -180 to 180 degrees
        for (int angle = -180; angle <= 180; ++angle) {
            float x1_rot, y1_rot, x2_rot, y2_rot;

            // Rotate both points around (x1, y1)
            rotatePoint(x_1, y_1, x_1, y_1, angle, x1_rot, y1_rot);
            rotatePoint(x_2, y_2, x_1, y_1, angle, x2_rot, y2_rot);

            // Calculate the new slope and intercept
            float new_m = (y2_rot - y1_rot) / (x2_rot - x1_rot);
            float new_b = y1_rot - new_m * x1_rot;
            bool okay = true;
            for(const auto& oppo : theGlobalOpponentsModel.opponents) {

                std::vector<float> obstacle_pos = {oppo.position.x(), oppo.position.y()};
                std::vector<float> true_obstacle_pos = transformToGlobalCoordinates(obstacle_pos, self);

                float distance = abs(new_m * true_obstacle_pos[0] - true_obstacle_pos[1] + new_b) / sqrt(new_m * new_m + 1);
                if (distance < 200) {
                    okay = false;
                }
            }

            for(const auto& teammate : theGlobalTeammatesModel
                .teammates) {

                    std::vector<float> obstacle_pos = {teammate.pose.translation.x(), teammate.pose.translation.y()};

                if(teammate.playerNumber == number) {
                    continue;
                }


                float distance = abs(new_m * obstacle_pos[0] - obstacle_pos[1] + new_b) / sqrt(new_m * new_m + 1);
                if (distance < 200) {
                    okay = false;
                }
            }

            if(!okay) {
                continue;
            }
            float new_dis = abs(new_m * x - y + new_b) / sqrt(new_m * new_m + 1);
            if (new_dis < min_dis) {
                min_dis = new_dis;
                final_b = new_b;
                final_m = new_m;
            }
        }

            float angle = calculateAngleBetweenVectors({1, 0}, normalizeVector(x_2 - x_1, final_m * x_2 + final_b - y_1));
            float cross = crossProduct({1, 0}, normalizeVector(x_2 - x_1, final_m * x_2 + final_b - y_1));
            if (cross > 0) {
                cross = 1;
            } else {
                cross = -1;
            }
            float ball_angle = angle * cross;
            auto rel_pos = theRobotPose.inverse() * Vector2f(x_1, y_1);
            float rel_angle = ball_angle - theRobotPose.rotation;
            const Pose2f speed(1.f, 1.f, 1.f);
            const Pose2f rel_target(rel_angle, rel_pos.x(), rel_pos.y());
            auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * rel_target, speed);
            float distance = abs(std::hypot(x_2 - x_1, final_m * x_2 + final_b - y_1));
            theWalkToBallAndKickSkill({
                .targetDirection = Angle::normalize(rel_angle),
                .obstacleAvoidance = obstacleAvoidance,
                .speed = speed,
                .kickType = KickInfo::forwardFastRightLong,
                .kickLength = distance,
            });

    }

    float euclideanDistance(float x1, float y1, float x2, float y2) {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    std::map<int, Eigen::Vector2f> calculateVoronoiCenters(
        const std::vector<float>& agent_loc,
        const std::vector<std::vector<float>>& teammate_loc,
        const std::vector<std::vector<float>>& opponent_loc) {

        // Field dimensions
        const float fieldLength = 9050;
        const float fieldWidth = 6000;
        const float gridSize = 50;
        const int gridRows = static_cast<int>(fieldWidth / gridSize);
        const int gridCols = static_cast<int>(fieldLength / gridSize);

        // Initialize Voronoi areas for each robot
        std::map<int, std::vector<Eigen::Vector2f>> voronoiAreas;

        // Collect all locations (agent, teammates, and opponents)
        std::vector<std::vector<float>> all_locs = {agent_loc};
        all_locs.insert(all_locs.end(), teammate_loc.begin(), teammate_loc.end());
        all_locs.insert(all_locs.end(), opponent_loc.begin(), opponent_loc.end());

        // Loop through each grid cell
        for (int i = 0; i < gridRows; ++i) {
            for (int j = 0; j < gridCols; ++j) {
                float x = (j - gridCols / 2) * gridSize;
                float y = (i - gridRows / 2) * gridSize;

                // Find the nearest robot/opponent
                float minDistance = std::numeric_limits<float>::max();
                int nearestRobot = -1;

                for (const auto& loc : all_locs) {
                    float distance = euclideanDistance(x, y, loc[0], loc[1]);
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestRobot = static_cast<int>(loc[2]);
                    }
                }

                // Assign this cell to the nearest robot's Voronoi area
                voronoiAreas[nearestRobot].emplace_back(x, y);
            }
        }

        // Initialize a map to store the Voronoi centers
        std::map<int, Eigen::Vector2f> voronoiCenters;

        // Calculate the center of each Voronoi area
        for (const auto& [robot, cells] : voronoiAreas) {
            Eigen::Vector2f sum(0.0f, 0.0f);
            int cellCount = static_cast<int>(cells.size());

            for (const auto& cell : cells) {
                sum += cell;
            }

            voronoiCenters[robot] = sum / cellCount;
        }

        return voronoiCenters;
    }

    void ball_search_goalie()
    {
        // Implementing state machine
        // Turn towards right  - GOALIE_LOCAL_SEARCH_RIGHT
        // Turn towards left - GOALIE_LOCAL_SEARCH_LEFT
        // Sidestep to the goal position - GOALIE_MOVE_CENTRE
        // Walk back - GOALIE WALK_BACK
        // Stand still - GOALIE_STAND_STILL
        switch(goalie_search_state)
        {
            case GOALIE_LOCAL_SEARCH_RIGHT:
            {
                // std::cout << "GOALIE_LOCAL_SEARCH_RIGHT" << std::endl;
                // std::cout << theRobotPose.rotation <<  " " << 90_deg << " " << theWalkAtRelativeSpeedSkill.isDone() << std::endl;
                theWalkAtRelativeSpeedSkill({.speed = {0.5f, 0.0f, 0.0f}});
                // theLookActiveSkill({.withBall = true});
                theLookAtAnglesSkill({.pan = 90_deg});
                if (theRobotPose.rotation > 110_deg)
                {
                    goalie_search_state = GOALIE_LOCAL_SEARCH_LEFT;
                }
                break;
            }
            case GOALIE_LOCAL_SEARCH_LEFT:
            {
                // std::cout << "GOALIE_LOCAL_SEARCH_LEFT" << std::endl;

                theWalkAtRelativeSpeedSkill({.speed = {-0.5f, 0.0f, 0.0f}});
                // theLookActiveSkill({.withBall = true});
                theLookAtAnglesSkill({.pan = -90_deg});
                // std::cout << theRobotPose.rotation <<  " " << -90_deg << " " << theWalkAtRelativeSpeedSkill.isDone() << std::endl;
                if (theRobotPose.rotation < -110_deg)
                {
                    goalie_search_state = GOALIE_LOOK_FORWARD;
                }
                break;
            }
            case GOALIE_LOOK_FORWARD:
            {
                // std::cout << "GOALIE_LOOK_FORWARD" << std::endl;
                theWalkAtRelativeSpeedSkill({.speed = {0.5f, 0.0f, 0.0f}});
                // theLookActiveSkill({.withBall = true});
                theLookAtAnglesSkill({.pan = 0_deg});
                if (theRobotPose.rotation > -5_deg)
                {
                    goalie_search_state = GOALIE_MOVE_CENTRE;
                }
                break;
            }
            case GOALIE_MOVE_CENTRE:
            {
                // std::cout << "GOALIE_MOVE_CENTRE" << std::endl;
                Pose2f robot_curr = theRobotPose;
                Pose2f target_pose = Pose2f(-0.0f, robot_curr.translation.x(), 0.0f);
                Pose2f relative_pose = robot_curr.inverse() * target_pose.translation;
                const Angle rel_angle = -theRobotPose.rotation;
                theWalkToPoseSkill({.target = Pose2f(rel_angle, relative_pose.translation.x(), relative_pose.translation.y())});
                if (abs(theRobotPose.translation.y()) < 50)
                {
                    goalie_search_state = GOALIE_WALK_BACK;
                }
                break;
            }
            case GOALIE_WALK_BACK:
            {
                // std::cout << "GOALIE_WALK_BACK" << std::endl;
                theWalkAtRelativeSpeedSkill({.speed = {0.0f, -0.5f, 0.0f}});
                if (theRobotPose.translation.x() < -4200)
                {
                    goalie_search_state = GOALIE_STAND_STILL;
                }
                break;
            }
            case GOALIE_STAND_STILL:
            {
                // std::cout << "GOALIE_STAND_STILL" << std::endl;
                theStandSkill();
                break;
            }
        }


        // Vector2f rel_pos = theRobotPose.inverse() * Vector2f(-4250, 0);
        // const Angle rel_angle = theRobotPose.rotation;
        // theWalkToPointSkill({.target = Pose2f(- rel_angle, rel_pos.x(), rel_pos.y())});
    }

public:
    option(NeuralControl)
    {
        cognitionLock.lock();

        if (!policy_initialized) {
            //std::cout << "POLICY_PATH: " << POLICY_PATH << std::endl;

            goalie_policy.init(POLICY_PATH + "Goalie/policy.onnx");
            off_ball_policy.init(POLICY_PATH + "OffBall/policy.onnx");
            on_ball_policy.init(POLICY_PATH + "OnBall/policy.onnx");
            near_opp_goal_policy.init(POLICY_PATH + "NearOppGoal/policy.onnx");
            on_ball_dribble_policy.init(POLICY_PATH + "OnBallDribble/policy.onnx");

            obs.init_strategy(strategy_positions);

            policy_initialized = true;
            robot_time[theGameState.playerNumber] = 0;

            // ball search variable setup
            ballsearch_time[theGameState.playerNumber] = 0;
        }

        robot_time[theGameState.playerNumber] += 1;

        std::vector<float> self = {
            theRobotPose.translation.x(),
            theRobotPose.translation.y(),
            theRobotPose.rotation
        };


        std::vector<float> robot_direction = transformRotationGlobal(theRobotPose.rotation);

        std::vector<float> ball = (theFieldBall.timeSinceBallWasSeen > 4000)
            ? std::vector<float>{theFieldBall.teamPositionOnField.x(), theFieldBall.teamPositionOnField.y()}
            : std::vector<float>{theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y()};

        target = get_target(ball, target_angle, action_radius);

        float ball_to_robot = dist(self, ball);
        float goalie_x = -10000; // make sure that we have a goalie ?
        float goalie_y = -10000;

        std::vector<std::vector<float>> teammates;
        for(const auto& teammate : theGlobalTeammatesModel.teammates) {
            // do not include goalie
            if (teammate.playerNumber == 1) {
                goalie_x = teammate.pose.translation.x();
                goalie_y = teammate.pose.translation.y();
            } else {


                teammates.push_back({
                    teammate.pose.translation.x(),
                    teammate.pose.translation.y(),
                    (float) teammate.playerNumber,
                });
            }
        }

        float self_ball = abs(std::hypot(ball[0] - self[0], ball[1] - self[1]));
        float self_goalie = abs(std::hypot(goalie_x - self[0], goalie_y - self[1]));
        float goalie_ball = abs(std::hypot(goalie_x - ball[0], goalie_y - ball[1]));

        std::vector<std::vector<float>> opponents;
        for(const auto& opponent : theObstacleModel.obstacles) {
            std::vector<float> oppo_pos = {opponent.center.x(), opponent.center.y()};
            std::vector<float> true_opponent_pos = transformToGlobalCoordinates(oppo_pos, self);
            opponents.push_back(true_opponent_pos);
        }

        std::vector<float> dis;
        std::vector<float> angles;
        for (const auto& opponent : opponents) {
            float distance = abs(std::hypot(opponent[0] - self[0], opponent[1] - self[1]));
            auto direction = calculateAvoidanceDirection(self, opponent);
            float angle = calculateAngleBetweenVectors(robot_direction, direction);
            dis.push_back(distance);
            angles.push_back(angle);
        }
        float min_distance = std::numeric_limits<float>::max();
        bool dribble = false;
        for (int i = 0; i < dis.size(); ++i) {
            if (dis[i] < min_distance) {
                min_distance = dis[i];
            }
        }

        float angle = calculateAngleBetweenVectors({1, 0}, normalizeVector(ball[0] - self[0], ball[1] - self[1]));
        float cross = crossProduct({1, 0}, normalizeVector(ball[0] - self[0], ball[1] - self[1]));



        if (cross > 0) {
            cross = 1;
        } else {
            cross = -1;
        }
        float ball_angle = angle * cross;

        // get angle between robot and ball from robots perspective
        float angle_to_ball = calculateAngleBetweenVectors(robot_direction, normalizeVector(ball[0] - self[0], ball[1] - self[1]));
        if (min_distance < anywhere_opponent_dis_threshold){
            dribble = true;
        }
        for (int i = 0; i < dis.size(); ++i) {
            if (dis[i] < front_opponent_dis_threshold && angles[i] < front_opponent_angle_threshold) {
                dribble = true;
            }
        }
        // check if within 500cm of the ball or
        Vector2 agentPos(self[0], self[1]);
        float agentOrientationRadians = self[2];
        Vector2 ballPos(ball[0], ball[1]);
        Vector2 goalPos(4500, 0);

        if (dist(self, ball) > 650 || calculateNormalizedAngle(agentPos, agentOrientationRadians, ballPos) > 0.7f) {
            dribble = false;
        }

        std::vector<std::vector<float>> sorted_teammates = teammates;
        int on_ball_robot = 2;
        if (robot_roles[theGameState.playerNumber] == "goalie") {

            // do nothing

        } else if (sorted_teammates.size() == 0) {

            // this robot is the only one on the field
            on_ball_robot = theGameState.playerNumber;
            robot_roles[theGameState.playerNumber] = "on_ball";

        } else if (robot_time[theGameState.playerNumber] % role_switch_interval == 0) {

            // compare with teammates' distances to the ball and potentially to determine which robot should be on ball

            // sort teammates by distance to ball
            std::sort(sorted_teammates.begin(), sorted_teammates.end(), [ball](std::vector<float> a, std::vector<float> b) {
                return dist(a, ball) < dist(b, ball);
            });

            std::vector<float> teammate_dist_values = {999999, 999999, 999999, 999999, 999999};

            float teammate_angle_to_ball_weight = 600/3.14;
            float fallen_weight = 9999999;

            for(const auto& teammate : theGlobalTeammatesModel.teammates) {
                if (teammate.playerNumber == 1)
                    continue;
                int player_number = teammate.playerNumber;

                float distance = abs(std::hypot(ball[0] - teammate.pose.translation.x(), ball[1] - teammate.pose.translation.y()));
                std::vector<float> teammate_pos = {teammate.pose.translation.x(), teammate.pose.translation.y()};
                float teammate_angle_to_ball = calculate_alignment_angle(teammate_pos, ball, OUR_GOAL);

                teammate_dist_values[player_number - 1] = distance + teammate_angle_to_ball * teammate_angle_to_ball_weight;
            }

            for(auto const& teammate : theTeamData.teammates)
            {
                if (!teammate.theRobotStatus.isUpright)
                    teammate_dist_values[teammate.number - 1] += fallen_weight;
            }

            float our_distance = abs(std::hypot(ball[0] - self[0], ball[1] - self[1]));
            float our_angle_to_ball = calculate_alignment_angle({self[0], self[1]}, ball, OUR_GOAL);

            float our_dist_value = our_distance + our_angle_to_ball * teammate_angle_to_ball_weight;

            if (robot_roles[theGameState.playerNumber] == "off_ball") {
                // check if lower value than lowest value teammate by role switch distance
                if (our_dist_value < *min_element(teammate_dist_values.begin(), teammate_dist_values.end())) {
                    robot_roles[theGameState.playerNumber] = "on_ball";
                    on_ball_robot = theGameState.playerNumber;
                }
            }
            else if (robot_roles[theGameState.playerNumber] == "on_ball") {
                // check if higher value than highest value teammate by role switch distance
                if (our_dist_value > *min_element(teammate_dist_values.begin(), teammate_dist_values.end()) + role_switch_distance) {
                    robot_roles[theGameState.playerNumber] = "off_ball";
                    on_ball_robot = min_element(teammate_dist_values.begin(), teammate_dist_values.end()) - teammate_dist_values.begin() + 1;
                }
            }
        }

        bool is_near_opp_goal = false;
        if (robot_roles[theGameState.playerNumber] == "on_ball"){
            if (shouldUserNearOppGoalPolicy(self, ball)) {
                policy = near_opp_goal_policy;
                is_near_opp_goal = true;

            }
            else if (dribble==true) {
                policy = on_ball_dribble_policy;
            }
             else {
                policy = on_ball_policy;
            }
        }
        else if (robot_roles[theGameState.playerNumber] == "goalie") // need to change this back
            policy = goalie_policy;
        else if (robot_roles[theGameState.playerNumber] == "off_ball")
            policy = off_ball_policy;

        std::string policy_type = is_near_opp_goal ? "near_opp_goal" : robot_roles[theGameState.playerNumber];

        std::map<std::string, int>::iterator it = policy_indices.find(policy_type);
        if (it != policy_indices.end()) {
            theBehaviorStatus.policyIndex = it->second;
        } else {
            theBehaviorStatus.policyIndex = -1;
        }

        std::vector<float> observation = obs.get_observation(
            self,
            ball,
            target,
            teammates,
            opponents,
            theGameState.playerNumber,
            policy_type,
            on_ball_robot,
            player_to_one_hot[theGameState.playerNumber],
            closest_input_one_hot[theGameState.playerNumber],
            robot_home[theGameState.playerNumber],
            use_closest_input_vector,
            use_agent_id_vector,
            use_home_location,
            dribble
        );

        std::vector<float> policy_action = policy.inference(observation);

        if (policy_type != "near_opp_goal") {
            // clip policy action to [-1, 1]
            for (size_t i = 0; i < policy_action.size(); ++i) {
                policy_action[i] = std::min(1.0f, std::max(-1.0f, policy_action[i]));
            }
        }
        // TODO check spinning
        // TODO goalie different ball search]
        // TODO corner kick?

        if (theGameState.isKickOff() && theGameState.isForOwnTeam()) {
            // theGameState.isKickOff() stays true for 10 seconds after Playing starts
            if (theFrameInfo.getTimeSince(time_of_our_most_recent_kickoff[theGameState.playerNumber]) > 10000) { // 10 seconds
                // the following block of code should only execute once per kickoff
                time_of_our_most_recent_kickoff[theGameState.playerNumber] = theFrameInfo.time;
                kickoff_forward_robot_global[theGameState.playerNumber] = {-1, 0};
            }
        }

        theLookAtBallSkill();

        // Ball search strategy:
        // 1: If you see the ball, use your own ball position.
        // 2: If you don't see the ball for ~3 seconds, use teammates' ball position.
        // 3: If it is still not found for ~2 seconds, Initialize state as SPINNING.
        // 4: SPINNING: If the ball is not found within ~4(goalie)/~3(others) seconds, enter STANDING state.
        // 5: STANDING: If the ball is not found within ~3(goalie)/~1(others) seconds, enter WALKING state.
        // 6: WALKING: If the ball is not found after approximate a return point with 700 distances
        //    (goalie use fixed point, others use voronoiCenters on field) or walking for ~3 seconds, re-enter SPINNING state.
        // 7: Repeat stages 4 to 6 until the ball is found.
        if (shouldBeDoingKickoffSetPlay(theGameState, theFrameInfo) && isKickoffSetPlayRobot(theGameState, theRobotPose, theGlobalTeammatesModel)) {
            float abs_x = 2600.0f; // millimeters
            float abs_y = -1000.0f; // millimeters
            float abs_angle = 3.14f/2.f; // radians
            auto rel_pos = theRobotPose.inverse() * Vector2f(abs_x, abs_y);
            float rel_angle = abs_angle - theRobotPose.rotation;
            const Pose2f speed(1.f, 1.f, 1.f);
            auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * rel_pos, speed);
            theWalkToPoseSkill({
                .target = {rel_angle, rel_pos.x(), rel_pos.y()},
                .obstacleAvoidance = obstacleAvoidance,
            });
        } else if (theFieldBall.timeSinceBallWasSeen > 5000 && !theFieldBall.teammatesBallIsValid) {
            ballsearch_time[theGameState.playerNumber] = ballsearch_time[theGameState.playerNumber]+ 1;
            if (theGameState.isForOwnTeam() && theGameState.isCornerKick() && (theGameState.playerNumber == 4 || theGameState.playerNumber == 5)) {
                if(theGameState.playerNumber == 4) {
                    if (theFieldBall.timeSinceBallWasSeen > 17500 ) {

                        float angle = calculateAngleBetweenVectors({1, 0}, normalizeVector(4500 - self[0], 3000 - self[1]));
                        float cross = crossProduct({1, 0}, normalizeVector(4500 - self[0], 3000 - self[1]));
                        if (cross > 0) {
                            cross = 1;
                        } else {
                            cross = -1;
                        }
                        float angle_to_turn = angle * cross  - theRobotPose.rotation;
                        float x_des = 3500;
                        float y_des = 2000;
                        auto rel_pos = theRobotPose.inverse() * Vector2f(x_des, y_des);
                        theWalkToPoseSkill({.target = Pose2f(angle_to_turn, rel_pos.x(), rel_pos.y())});
                    } else {
                        float angle = calculateAngleBetweenVectors({1, 0}, normalizeVector(4500 - self[0], -3000 - self[1]));
                        float cross = crossProduct({1, 0}, normalizeVector(4500 - self[0], -3000 - self[1]));

                        if (cross > 0) {
                            cross = 1;
                        } else {
                            cross = -1;
                        }
                        float angle_to_turn = angle * cross  - theRobotPose.rotation;
                        float x_des = 3500;
                        float y_des = -2000;
                        auto rel_pos = theRobotPose.inverse() * Vector2f(x_des, y_des);
                        theWalkToPoseSkill({.target = Pose2f(angle_to_turn, rel_pos.x(), rel_pos.y())});
                    }
                } else {
                    if (theFieldBall.timeSinceBallWasSeen > 17500 ) {
                        float angle = calculateAngleBetweenVectors({1, 0}, normalizeVector(4500 - self[0], -3000 - self[1]));
                        float cross = crossProduct({1, 0}, normalizeVector(4500 - self[0], -3000 - self[1]));

                        if (cross > 0) {
                            cross = 1;
                        } else {
                            cross = -1;
                        }
                        float angle_to_turn = angle * cross  - theRobotPose.rotation;
                        float x_des = 3500;
                        float y_des = -2000;
                        auto rel_pos = theRobotPose.inverse() * Vector2f(x_des, y_des);
                        theWalkToPoseSkill({.target = Pose2f(angle_to_turn, rel_pos.x(), rel_pos.y())});
                    } else {
                        float angle = calculateAngleBetweenVectors({1, 0}, normalizeVector(4500 - self[0], 3000 - self[1]));
                        float cross = crossProduct({1, 0}, normalizeVector(4500 - self[0], 3000 - self[1]));
                        if (cross > 0) {
                            cross = 1;
                        } else {
                            cross = -1;
                        }
                        float angle_to_turn = angle * cross  - theRobotPose.rotation;
                        float x_des = 3500;
                        float y_des = 2000;
                        auto rel_pos = theRobotPose.inverse() * Vector2f(x_des, y_des);
                        theWalkToPoseSkill({.target = Pose2f(angle_to_turn, rel_pos.x(), rel_pos.y())});
                    }
                }
            } else {
                std::vector<float> time_thresholds;
                Vector2f return_point;
                if (robot_roles[theGameState.playerNumber] == "goalie") {
                    ball_search_goalie();
                }
                else {
                    std::map<int, Eigen::Vector2f> voronoiCenters = calculateVoronoiCenters(self, teammates, opponents);
                    if (voronoiCenters.count(theGameState.playerNumber) && voronoiCenters[theGameState.playerNumber].allFinite() &&
                        std::abs(voronoiCenters[theGameState.playerNumber].x()) <= 9050/2 &&
                        std::abs(voronoiCenters[theGameState.playerNumber].y()) <= 6000/2) {
                        return_point = voronoiCenters[theGameState.playerNumber];
                    } else {
                        if (theRobotPose.translation.x() >= 1000) {
                            return_point = Eigen::Vector2f(300, 0);
                        } else if (theRobotPose.translation.x() < 1000 && theRobotPose.translation.x() > -1000) {
                            return_point = Eigen::Vector2f(0, 0);
                        } else {
                            return_point = Eigen::Vector2f(-2700, 0);
                        }
                    }
                    time_thresholds.push_back(700);
                    time_thresholds.push_back(400);
                    time_thresholds.push_back(300);

                    if (ballsearch_time[theGameState.playerNumber] > time_thresholds[0]){
                        ballsearch_time[theGameState.playerNumber] = 0;
                    } else if (ballsearch_time[theGameState.playerNumber] > time_thresholds[1]){
                        float distance_to_return = (theRobotPose.translation - return_point).norm();
                        auto rel_pos = theRobotPose.inverse() * Vector2f(target[0], target[1]);
                        if (distance_to_return < 600.0f) {
                            ballsearch_time[theGameState.playerNumber] = 0;
                        }
                        theWalkToPoseSkill({.target = Pose2f(0.f, rel_pos.x(), rel_pos.y())});
                    } else if (ballsearch_time[theGameState.playerNumber] > time_thresholds[2]) {
                        theStandSkill();
                    } else {
                        theWalkAtRelativeSpeedSkill({.speed = {0.8f, 0.0f, 0.0f}});
                    }
                }

            }
        }
        else if (theGameState.isForOpponentTeam() && ((theGameState.isFreeKick() || theGameState.isGoalKick() || theGameState.isCornerKick())) && policy_type != "goalie") {
            std::vector<float> goal = {-4500, 0};
            auto direct = normalizeVector(goal[0] - ball[0], goal[1] - ball[1]);
            direct[0] = 1200 * direct[0] + ball[0];
            if (direct[0] < -4000) {
                direct[0] = -4000;
            } else if (direct[0] > 4000) {
                direct[0] = 4000;
            }
            direct[1] = 1200 * direct[1] + ball[1];
            if (direct[1] < -2700) {
                direct[1] = -2700;
            } else if (direct[1] > 2700) {
                direct[1] = 2700;
            }
            float rel_angle = ball_angle - theRobotPose.rotation;
            auto rel_pos = theRobotPose.inverse() * Vector2f(direct[0], direct[1]);
            // theWalkAtRelativeSpeedSkill({.speed = {0.8f, 0.0f, 0.0f}});
            if (robot_roles[theGameState.playerNumber] == "on_ball") {
                float rel_angle = ball_angle - theRobotPose.rotation;


                theWalkToPoseSkill({.target = Pose2f(rel_angle, rel_pos.x(), rel_pos.y())});
            }
        } else if (theGameState.isGoalKick() && theGameState.isForOwnTeam() && policy_type == "on_ball") {
            float min_dis = 1000000;
            float x = 4500;
            float y = 0;
            auto number = -1;
            for(const auto& teammate : teammates) {

                float distance = abs(std::hypot(teammate[0], teammate[1]));
                if (distance < min_dis && teammate[2] != 1) {
                    min_dis = distance;
                    number = teammate[2];
                    x = teammate[0];
                    y = teammate[1];
                }
            }
            if (x > self[0]) {
                kick_to_pos(x, y, self, number);
            } else {
                kick_to_pos(4500, 0, self, -1);
            }
        } else if (theGameState.isGoalKick() && theGameState.isForOwnTeam() && policy_type == "off_ball" && closest(0, 0, self)) {
                auto rel_pos = theRobotPose.inverse() * Vector2f(0, 0);
                theWalkToPoseSkill({.target = Pose2f(0_deg, rel_pos.x(), rel_pos.y())});

            }
            else if ((theGameState.isCornerKick() || theGameState.isFreeKick() || theGameState.isKickIn()) && theGameState.isForOwnTeam() && policy_type == "off_ball" && closest(3500, 0, self)) {
                           auto rel_pos = theRobotPose.inverse() * Vector2f(3500, 0);
                           theWalkToPoseSkill({.target = Pose2f(0_deg, rel_pos.x(), rel_pos.y())});

            }
                else if ((theGameState.isCornerKick() || theGameState.isFreeKick() || theGameState.isKickIn()) && theGameState.isForOwnTeam() && policy_type == "on_ball") {

            float min_dis = 1000000;
            float x = 4500;
            float y = 0;
            auto number = -1;
            for(const auto& teammate : teammates) {

                float distance = abs(std::hypot(4500 - teammate[0], teammate[1]));
                if (distance < min_dis && teammate[2] != 1) {
                    min_dis = distance;
                    number = teammate[2];
                    x = teammate[0];
                    y = teammate[1];
                }
            }
            if (x > self[0] || theGameState.isCornerKick()) {
                if (theGameState.isCornerKick() && abs(x - 4500) < 0.0001 && abs(y) < 0.0001) {
                    kick_to_pos(3500, 0, self, -1);
                } else {
                    kick_to_pos(x, y, self, number);
                }
            } else {
                kick_to_pos(4500, 0, self, -1);
            }
        }

        else {

            ballsearch_time[theGameState.playerNumber] = 0;
            goalie_search_state = GOALIE_LOCAL_SEARCH_RIGHT;
            //std::cout << "finding balls" << std::endl;
            if (policy_type == "off_ball") {
                walkMotion.updateWalkAction(policy_action);
                if (policy_action[4] > 0.0f) {
                    theStandSkill();
                } else {
                    std::vector<float> walkAction = walkMotion.getWalkAction();

                    theWalkAtRelativeSpeedSkill({.speed = {
                        policy_action[2] * 0.5f,
                        policy_action[0] * 1.8f,
                        policy_action[1] * 0.7f
                    }});
                }
            } else if (policy_type == "on_ball") {
                float angle_to_turn = calculateAngleBetweenVectors({1, 0}, normalizeVector(goalie_x - self[0], goalie_y - self[1]));
                if (self_ball > goalie_ball && self_goalie < 1000 && goalie_x > -5000 && ball[0] < -3450 && ball[1] < 1000 && ball[1] > -1000) {

                    float angle_to_turn = calculateAngleBetweenVectors({1, 0}, normalizeVector(goalie_x - self[0], goalie_y - self[1]));
                    float crossproduct = crossProduct({1, 0}, normalizeVector(goalie_x - self[0], goalie_y - self[1]));
                    if (crossproduct > 0) {
                        crossproduct = 1;
                    } else {
                        crossproduct = -1;
                    }

                    angle_to_turn = angle_to_turn * crossproduct;

                    float turn_angle_when_trigger = angle_to_turn - theRobotPose.rotation;

                    auto vector_from = normalizeVector(self[0] - goalie_x, self[1] - goalie_y);
                    vector_from[0] = vector_from[0] * 1000 + goalie_x;
                    vector_from[1] = vector_from[1] * 1000 + goalie_y;

                    auto vec_rel = theRobotPose.inverse() * Vector2f(vector_from[0], vector_from[1]);
                    const Pose2f rel_target(turn_angle_when_trigger, vec_rel.x(), vec_rel.y());
                    theWalkToPoseSkill({
                                          .target = rel_target,
                                      });
                } else {
                    // update target angle
                    target_angle = target_angle + policy_action[0] * angle_disp_coef;
                    target_angle = std::fmod(target_angle + M_PI, 2 * M_PI) - M_PI;
                    target = get_target(ball, target_angle, action_radius);

                    float rel_angle = target[2] - theRobotPose.rotation;

                    if (dist(self, ball) < action_radius && std::abs(rel_angle) < 0.14) {
                        fix_kick_angle = true;
                    }

                    if (fix_kick_angle && (last_ball_to_robot - ball_to_robot) < -14) {
                        fix_kick_angle = false;
                    }
                    // std::cout << dribble << std::endl;

                    if (dribble){
                        fix_kick_angle = false;
                        theBehaviorStatus.onBallSkillIndex = 1;
                        theWalkAtRelativeSpeedSkill({.speed = {
                            policy_action[2] * 0.8f,
                            policy_action[0] * 1.8f,
                            policy_action[1] * 0.7f
                        }});
                    }
                    else {
                        theBehaviorStatus.onBallSkillIndex = 2;
                        auto rel_pos = theRobotPose.inverse() * Vector2f(ball[0], ball[1]);
                        const Pose2f rel_target(rel_angle, rel_pos.x(), rel_pos.y());

                        // kick parameters
                        const Pose2f speed(1.f, 1.f, 1.f);
                        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * rel_target, speed);

                        KickPrecision alignPrecisely = KickPrecision::notPrecise;
                        KickInfo::KickType kickType;
                        float kick_length;
                        Angle targetDirection;
                        float abs_kick_angle;

                        // long kicks during the game (overridden during kick off)
                        // (1) all teammates/opponents are 2m away
                        // (2) ball is more than 2m away from our goal
                        // (3) ball is more than 2m away from their goal

                        // long kick for kick off
                        if (theGameState.isKickOff()) {
                            // alignPrecisely = KickPrecision::precise;
                            // std::cout << "kick off" << std::endl;


                            // avoid opponents
                            float left_bound = std::atan2(800 - 0, 4500 - 0); // left goal post
                            float right_bound = -PI/2;
//                            abs_kick_angle = get_optimal_kick_angle(opponents, {0, 0}, 0.f, left_bound, right_bound, -0.35);

                            abs_kick_angle = -.35;

                            // std::cout << "L: " << left_bound << " | R: " << right_bound << std::endl;
                            // std::cout << "kick-off angle: " << abs_kick_angle << std::endl;

                            // std::cout << "abs angle: " << kick_off_angle << std::endl;
                            targetDirection = Angle::normalize(abs_kick_angle - theRobotPose.rotation);
                            float right_goal_post = std::atan2(-800 - 0, 4500 - 0);

                            // make kick weaker if decides to kick not at goal
                            if (abs_kick_angle < right_goal_post + 0.01) {
                                // std::cout << "WEAK" << std::endl;
                                kick_length = 2500.f;
                                kickType = KickInfo::forwardFastRight;
                            } else {
                                // std::cout << "STRONG" << std::endl;
                                kick_length = 3500.f;
                                kickType = KickInfo::forwardFastRight;
                            }
                        } else {
                            alignPrecisely = KickPrecision::notPrecise;
                            kickType = KickInfo::walkForwardsRightLong;
                            kick_length = 2500.f;


                            float left_bound = std::atan2(800 - ball[1], 4500 - ball[0]) - target[2]; // left goal post
                            float right_bound = std::atan2(-800 - ball[1], 4500 - ball[0]) - target[2]; // right goal post

                            // std::cout << "left bound: " << left_bound << " | right bound: " << right_bound << std::endl;

                            float target_angle = target[2];
                            if (
                                target[2] > std::atan2(800 - ball[1], 4500 - ball[0])
                                || target[2] < std::atan2(-800 - ball[1], 4500 - ball[0])
                            )
                                target_angle = std::atan2(0 - ball[1], 4500 - ball[0]);

                            abs_kick_angle = get_optimal_kick_angle(opponents, ball, target_angle, left_bound, right_bound, target_angle);


//                            std::cout << "policy_angle: " << target[2] << " | kick angle: " << abs_kick_angle << std::endl;
                            targetDirection = fix_kick_angle ? 0 : Angle::normalize(abs_kick_angle - theRobotPose.rotation);

                            if (
                                dist(ball, OUR_GOAL) > 2000
                                && dist(ball, OPP_GOAL) > 2000
                                && std::none_of(
                                                opponents.begin(),
                                                opponents.end(),
                                                [&self](const auto& opponent) {
                                                    return dist(opponent, self) < 2000;
                                                }
                            )
                            ) {
                                // alignPrecisely = KickPrecision::precise;
                                kickType = KickInfo::forwardFastRight;
                                kick_length = 3500.f;

                            }
                        }

                        // std::cout << "target angle: " << target[2] << std::endl;
                        // std::cout << "angle adjustment: " << abs_kick_angle - target[2] << std::endl;
                        // std::cout << "target direction: " << targetDirection << std::endl;

                        theWalkToBallAndKickSkill({
                            .targetDirection = targetDirection,
                            .obstacleAvoidance = obstacleAvoidance,
                            .speed = speed,
                            .kickType = kickType,
                            .kickLength = kick_length,
                            .alignPrecisely = alignPrecisely
                        });
                    }
                }
            } else if (policy_type == "near_opp_goal") {
                theWalkAtRelativeSpeedSkill({.speed = {
                    policy_action[2],
                    policy_action[0],
                    policy_action[1]
                }});
            } else if (policy_type == "goalie") {
                if(use_neural_goalie) {
                    // Perform walk
                    theWalkAtRelativeSpeedSkill({.speed = {
                        policy_action[2] * 0.5f,
                        policy_action[0],
                        policy_action[1]
                    }});
                }
                else {
                    bool is_our_goal_kick = theGameState.isGoalKick() && theGameState.isForOwnTeam();
                    bool is_goalie_in_position = theRobotPose.translation.x() < -4150 && theRobotPose.translation.x() > -4350 && std::abs(theRobotPose.translation.y()) < 50 && std::abs(theRobotPose.rotation) < 5_deg;
                    if(is_our_goal_kick && !is_goalie_in_position)
                    {
                        // Goalie should walk to the center of the goal
                        float x = -4250;
                        float y = 0;
                        auto rel_pos = theRobotPose.inverse() * Vector2f(x, y);
                        const Angle target_angle = 0_deg;
                        const Pose2f speed(1.f, 1.f, 1.f);
                        auto relative_angle = target_angle - theRobotPose.rotation;
                        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * rel_pos, speed);
                        theWalkToPoseSkill({.target = Pose2f(relative_angle, rel_pos.x(), rel_pos.y()), .obstacleAvoidance = obstacleAvoidance});
                        theLookAtBallSkill();

                    }
                    else {
                        std::vector<float> left_post = {-4500, 800};
                        std::vector<float> right_post = {-4500, -800};
                        float width_btwn_posts = 1500;
                        float kick_region = 2300;

                        float dist_ball_left_post = dist(ball, left_post);
                        float dist_ball_right_post = dist(ball, right_post);
                        float den = dist_ball_left_post + dist_ball_right_post;
                        float num = width_btwn_posts * dist_ball_right_post;
                        float dist_goal_area_bot_to_intersection = num / den;
                        std::vector<float> goal_area_int_bisector_loc = {right_post[0], right_post[1] + dist_goal_area_bot_to_intersection};
                        std::vector<float> goal_ball_vec = {ball[0] - goal_area_int_bisector_loc[0], ball[1] - goal_area_int_bisector_loc[1]};
                        float vec_norm = vector_norm(goal_ball_vec);
                        goal_ball_vec[0] /= vec_norm;
                        goal_ball_vec[1] /= vec_norm;
                        float goal_area_length = 600;
                        float goal_offset = goal_area_length;

                        std::vector<float> goal_point = {
                            goal_area_int_bisector_loc[0] + goal_ball_vec[0] * goal_offset,
                            goal_area_int_bisector_loc[1] + goal_ball_vec[1] * goal_offset
                        };

                        float near_post_dist = 400;
                        if(dist(goal_point, left_post) < near_post_dist || dist(goal_point, right_post) < near_post_dist) {
                            // adjust so that robot doesn't hit the post
                            goal_point[0] += goal_ball_vec[0] * 0;
                            goal_point[1] += goal_ball_vec[1] * 50;
                        }

                        if (goal_point[0] < -4350)
                        {
                            goal_point[0] = -4350;
                        }

                        auto goal_point_rel_robot = theRobotPose.inverse() * Vector2f(goal_point[0], goal_point[1]);

                        const Pose2f speed(1.f, 1.f, 1.f);
                        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * goal_point_rel_robot, speed);

                        int kick_region_min_x = -4500;
                        int kick_region_max_x = -3400;
                        int near_baseline_min_x = -4300;
                        int kick_region_y_abs = kick_region / 2;

                        bool ball_in_goal_region = (ball[0] >= kick_region_min_x && ball[0] <= kick_region_max_x)
                                                    && (std::abs(ball[1]) <= kick_region_y_abs);

                        float kick_angle = std::atan2(-ball[1], -ball[0]);
                        float kick_angle_rel_robot = kick_angle - self[2];
                        float distance = abs(std::hypot(ball[0] - self[0], ball[1] - self[1]));
                        // bool ball_easy_kick = false;
                        bool ball_easy_kick = (abs(kick_angle_rel_robot) < 5) && (distance < 250);
                        // Facing angle shuould roughly forward
                        bool self_facing_forward = abs(self[2]) <= PI/2 ;
                        bool no_kick_region = (ball[0] < near_baseline_min_x) && (std::abs(ball[1]) > (width_btwn_posts/2 + 50));
                        if((ball_in_goal_region || ball_easy_kick) && self_facing_forward) {
                            // safe to perform a very fast kicks
                            if (self_facing_forward){
                                // in a region that we definate want to kicks
                                theWalkToBallAndKickSkill({
                                    .targetDirection = 0,
                                    .kickType = KickInfo::walkForwardsRightLong,
                                    .kickLength = 2500.f,
                                });
                            }
                        }
                        else if (ball_in_goal_region){
                            theWalkToBallAndKickSkill({
                                .targetDirection = kick_angle_rel_robot,
                                .kickType = KickInfo::walkForwardsRightLong,
                                .kickLength = 2500.f,
                            });
                        }
                        else {
                            // walks to a blocking position
                            float dist_to_goal = vector_norm(goal_point_rel_robot[0], goal_point_rel_robot[1]);
                            float goal_pos_thresh = 10;
                            float rel_angle_to_ball = theFieldBall.recentBallPositionRelative().angle();
                            float goal_angle_thresh = 0.5;

                            if((dist_to_goal > goal_pos_thresh) || (rel_angle_to_ball > goal_angle_thresh)) {
                                theWalkToPoseSkill({
                                    .target = Pose2f(Angle::normalize(theFieldBall.recentBallPositionRelative().angle()), goal_point_rel_robot[0], goal_point_rel_robot[1]),
                                    .forceSideWalking = true,
                                    .obstacleAvoidance = obstacleAvoidance,
                                    .speed = speed,
                                });
                            }
                            else {
                                theStandSkill();
                            }
                        }
                    }
                }
            } else {
               // std::cout << "invalid policy type: " << policy_type << std::endl;
            }
        }
        last_ball_to_robot = ball_to_robot;
        cognitionLock.unlock();
    }
private:
    std::mutex cognitionLock;
    bool policy_initialized = false;
    Observation obs;
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);

