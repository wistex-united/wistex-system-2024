/*
 *
 * @file NeuralControlTraining.cpp
 *
 * This file implements an implementation for the NeuralControlTraining skill.
 *
 * @author BadgerRL
 */

#include "Tools/RL/RLAlg.h"
#include "Tools/RL/RLObs.h"
#include "Tools/RL/RLVars.h"
#include "Tools/RL/RLRewards.h"

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
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Communication/TeamData.h"

#include "Tools/Modeling/Obstacle.h"
#include "Debugging/DebugDrawings.h"
#include "Libs/Debugging/ColorRGBA.h"

#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"

#include <cmath>
#include "Tools/BehaviorControl/Strategy/PositionRole.h"

#include <stdio.h>
#include <iostream>
#include <filesystem>
#include <map>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <stdlib.h>
#include <random>

#undef state
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

int fd_pipe_observation;
int fd_pipe_action;
int fd_pipe_reward;
int fd_pipe_terminated;
int fd_pipe_truncated;
int fd_pipe_exit;
int ACTION_SIZE = 3;
const int TIME_LIMIT_SECONDS = 2*60;
const bool TRAIN = true;
const bool ADD_BALL_NOISE_TRAIN = false;
const bool ADD_BALL_NOISE_TEST = false;
const bool allowFrameSkipping = false;
// WARMUP_STEPS is the number of simulator steps that are run during reset. This
// is to allow the simulated robot to settle into a state in which it can respond
// to actions.
const int WARMUP_STEPS = 150;

// Create a random number generator
std::default_random_engine rng;

// Create a normal distribution with the given mean and standard deviation
float mean = 0; // mm
float stddev = 0.0001f; // mm
std::normal_distribution<float> normal_distribution(mean, stddev);

// for debugging
const bool OVERRIDE_ACTION_TO_STAY_STILL = false;

SKILL_IMPLEMENTATION(NeuralControlTrainingImpl,
                     {,
    IMPLEMENTS(NeuralControlTraining),
    REQUIRES(GameState),
    REQUIRES(LibWalk),
    REQUIRES(MotionInfo),
    REQUIRES(FieldBall),
    REQUIRES(RobotPose),
    REQUIRES(GlobalTeammatesModel),
    REQUIRES(GlobalOpponentsModel),
    REQUIRES(FrameInfo),
    REQUIRES(RobotModel),
    MODIFIES(BehaviorStatus),
    CALLS(LookForward),
    CALLS(Stand),
    CALLS(PublishMotion),
    CALLS(WalkAtRelativeSpeed),
    CALLS(WalkToPose),
    CALLS(WalkToBallAndKick),
    CALLS(LookAtBall),
    CALLS(LookAtGlobalBall),
});

void print_vector(const std::vector<float>& vec){
    std::cout << "[";
    for (const auto& val : vec){
        std::cout << val << " ";
    }
    std::cout << "]" << std::endl;
    std::cout << std::endl;
}

void write_vector_to_pipe(const std::vector<float>& observation, int fd) {
    // std::cout << "Sent ";
    // print_vector(observation);
    ssize_t bytesWritten = write(fd, observation.data(), observation.size() * sizeof(float));
    if (bytesWritten == -1) {
        std::cerr << "Error writing to pipe; Only wrote " << bytesWritten << " bytes." << std::endl;
    }
}

void write_float_to_pipe(const float reward, int fd) {
    // std::cout << "Sending reward (in func) " << reward << "..." << std::endl;
    ssize_t bytesWritten = write(fd, &reward, sizeof(float));
    if (bytesWritten == -1) {
        std::cerr << "Error writing to pipe; Only wrote " << bytesWritten << " bytes." << std::endl;
    }
}

std::vector<float> read_vector_from_pipe(int fd, int vector_size) {
    std::vector<float> vec;
    char buffer[sizeof(float)];
    for (int i = 0; i < vector_size; i++) {
        ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
        vec.push_back(*reinterpret_cast<float*>(buffer));
    }
    return vec;
}


class NeuralControlTrainingImpl : public NeuralControlTrainingImplBase
{
public:
    bool ballInGoal() {
        const float absX = std::abs(theFieldBall.positionOnField.x());
        const float absY = std::abs(theFieldBall.positionOnField.y());
        return (4500 < absX && absX < 5055) && (absY < 800);
    }

    bool ballInGoalArea() {
        const float absX = std::abs(theFieldBall.positionOnField.x());
        const float absY = std::abs(theFieldBall.positionOnField.y());
        return (3900 < absX && absX < 4500) && (absY < 1100);
    }

    bool ballInPenaltyArea() {
        const float absX = std::abs(theFieldBall.positionOnField.x());
        const float absY = std::abs(theFieldBall.positionOnField.y());
        return (2850 < absX && absX < 4500) && (absY < 2000);
    }

    bool ballOutOfFieldBounds() {
        const float absX = std::abs(theFieldBall.positionOnField.x());
        const float absY = std::abs(theFieldBall.positionOnField.y());
        return (absY > 3000 || absX > 4500) && !ballInGoal();
    }

    bool shouldTerminateEpisode() {
        return ballInGoal() || !ballInPenaltyArea();
    }

    bool shouldTruncateEpisode() {
        return theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) / 1000 > TIME_LIMIT_SECONDS;
    }

    void reset() {
        std::cout << "reset start" << std::endl;
        std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
        std::vector<float> ball_loc = getNoisyBallLoc(ADD_BALL_NOISE_TRAIN);
        std::vector<float> observation = obs.get_training_observation(agent_loc, ball_loc);

        std::cout << "Sending observation_before from C++ to Python (size = " << observation.size() << ")..." << std::endl;
        write_vector_to_pipe(observation, fd_pipe_observation);

        std::cout << "Receiving action from Python..." << std::endl;
        curAction = read_vector_from_pipe(fd_pipe_action, ACTION_SIZE);

        prev_agent_loc = agent_loc;
        prev_ball_loc = ball_loc;
        framesSinceLastAction = 0;

        std::cout << "reset end" << std::endl;
    }

    void step() {
        std::cout << "step start" << std::endl;
        std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
        std::vector<float> ball_loc = getNoisyBallLoc(ADD_BALL_NOISE_TRAIN);
        std::vector<float> observation = obs.get_training_observation(agent_loc, ball_loc);

        // std::cout << "Sending observation (from step)" << std::endl;
        write_vector_to_pipe(observation, fd_pipe_observation);

        float reward = getReward(
            agent_loc,
            prev_agent_loc,
            ball_loc,
            prev_ball_loc,
            curAction,
            ballInGoal(),
            ballOutOfFieldBounds(),
            ballInGoalArea(),
            ballInPenaltyArea(),
            framesSinceLastAction
        );
        std::cout << "Sending reward " << reward << std::endl;
        write_float_to_pipe(reward, fd_pipe_reward);

        bool terminated = shouldTerminateEpisode();
        std::cout << "Sending terminated " << terminated << std::endl;
        write_float_to_pipe(terminated, fd_pipe_terminated);

        bool truncated = shouldTruncateEpisode();
        std::cout << "Sending truncated " << truncated << std::endl;
        write_float_to_pipe(truncated, fd_pipe_truncated);

        if (terminated || truncated) {
            std::cout << "Blocking on the exit pipe..." << std::endl;
            read_vector_from_pipe(fd_pipe_exit, 1);
            std::cout << "Exiting" << std::endl;
            sleep(1);
            exit(0);
        }

        prev_agent_loc = agent_loc;
        prev_ball_loc = ball_loc;

        // On the python side, this would be the end of step i and the beginning of step i+1

        // std::cout << "Receiving action from Python..." << std::endl;
        curAction = read_vector_from_pipe(fd_pipe_action, ACTION_SIZE);

        // std::cout << "step end" << std::endl;
    }

    int getNumFramesToSkip() {
        if (!allowFrameSkipping) {
            return 0;
        }
        std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
        std::vector<float> ball_loc = {theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y()};
        float distToBall = dist(agent_loc, ball_loc);
        if (distToBall < 400) {
            return 0;
        } else if (distToBall < 800) {
            return 1;
        } else if (distToBall < 1300) {
            return 4;
        }
        return 8;
    }

    bool shouldGetNewAction() {
        // return true at the end of an episode to trigger the step() function so we send
        // the last reward back to the training code.
        return framesSinceLastAction >= getNumFramesToSkip() || shouldTerminateEpisode() || shouldTruncateEpisode();
    }

    const std::vector<float> getNoisyBallLoc(bool addNoise) {
        float noiseX = addNoise ? normal_distribution(rng) : 0;
        float noiseY = addNoise ? normal_distribution(rng) : 0;
        // std::cout << "adding noise " << noiseX << " " << noiseY << std::endl;
        return {
            theFieldBall.positionOnField.x() + noiseX,
            theFieldBall.positionOnField.y() + noiseY
        };
    }

    void executeCurrentAction() {
        if (OVERRIDE_ACTION_TO_STAY_STILL) {
            theLookAtGlobalBallSkill();
            theStandSkill();
            return;
        }

        if (curAction.empty() || theFieldBall.timeSinceBallWasSeen > 4000) {
            theLookAtGlobalBallSkill();
        } else {
            theWalkAtRelativeSpeedSkill({.speed = {
                curAction[2], // turn
                curAction[0], // forward
                curAction[1]  // side-to-side
            }});
        }
    }

    void openAllPipes() {
        // Open named pipes for training models in a different process
        std::cout << "Opening pipe_observation..." << std::endl;
        fd_pipe_observation = open("/Users/jkelle/BadgerRLSystem/pipe_observation", O_WRONLY);
        std::cout << "Opening pipe_reward..." << std::endl;
        fd_pipe_reward = open("/Users/jkelle/BadgerRLSystem/pipe_reward", O_WRONLY);
        std::cout << "Opening pipe_terminated..." << std::endl;
        fd_pipe_terminated = open("/Users/jkelle/BadgerRLSystem/pipe_terminated", O_WRONLY);
        std::cout << "Opening pipe_truncated..." << std::endl;
        fd_pipe_truncated = open("/Users/jkelle/BadgerRLSystem/pipe_truncated", O_WRONLY);
        std::cout << "Opening pipe_action..." << std::endl;
        fd_pipe_action = open("/Users/jkelle/BadgerRLSystem/pipe_action", O_RDONLY);
        std::cout << "Opening pipe_exit..." << std::endl;
        fd_pipe_exit = open("/Users/jkelle/BadgerRLSystem/pipe_exit", O_RDONLY);
    }

    option(NeuralControlTraining)
    {
        cognitionLock.lock();

        if (TRAIN && remainingWarmupFrames > 0) {
            remainingWarmupFrames--;
            theLookAtGlobalBallSkill();
            theStandSkill();
            cognitionLock.unlock();
            return;
        }

        if (!policy_initialized){
            if (TRAIN) {
                openAllPipes();
                reset();
            } else {
                training_policy.init("../Policies/Training/policy.onnx");
                std::cout << "Policy initialized" << std::endl;
            }
            policy_initialized = true;
        }

        theLookAtGlobalBallSkill();
        if (shouldGetNewAction()) {
            if (TRAIN) {
                step();
            } else {
                std::vector<float> agent_loc = {theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation};
                std::vector<float> ball_loc = getNoisyBallLoc(ADD_BALL_NOISE_TEST);
                std::vector<float> observation = obs.get_training_observation(agent_loc, ball_loc);
                curAction = training_policy.inference(observation);
                // // print out the curAction vector
                std::cout << "Received action from Python: ";
                print_vector(curAction);
            }
            framesSinceLastAction = 0;
        }
        framesSinceLastAction++;

        executeCurrentAction();
        cognitionLock.unlock();
    }
private:
    std::mutex cognitionLock;
    Policy training_policy;
    Observation obs;
    bool policy_initialized = false;

    int framesSinceLastAction;
    int remainingWarmupFrames = WARMUP_STEPS;
    std::vector<float> curAction;
    std::vector<float> prev_agent_loc;
    std::vector<float> prev_ball_loc;
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlTrainingImpl);
