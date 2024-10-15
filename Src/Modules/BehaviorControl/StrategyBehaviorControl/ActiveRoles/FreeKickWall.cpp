/**
 * @file FreeKickWall.cpp
 *
 * This file implements a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 */

#include "FreeKickWall.h"
#include <iostream>
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
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Modules/BehaviorControl/SkillBehaviorControl/Helpers/configUtils.h"

SkillRequest FreeKickWall::execute(const Agent&, const Agents&)
{
    
    return SkillRequest::Builder::empty();
}
