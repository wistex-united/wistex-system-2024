/**
 * @file SkillInterfaces.h
 *
 * This file declares all skills that are used by the current behavior.
 * It is included by two different files that interpret it differently.
 * Note that there is no #pragma once in this file!
 *
 * @author Probably many people who will not add themselves to this declaration.
 */

/** This skill turns off the joints of the robot. */
SKILL_INTERFACE(PlayDead);

/**
 * This skill makes the robot stand.
 * @param high Whether the knees should be stretched
 */
SKILL_INTERFACE(Stand, (bool)(false) high);

/**
 * This skill walks with a specified speed.
 * @param speed The walking speed in radians/s for the rotation and mm/s for the translation
 */
SKILL_INTERFACE(WalkAtAbsoluteSpeed, (const Pose2f&) speed);

/**
 * This skill walks with a specified speed relative to the configured maximum.
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 */
SKILL_INTERFACE(WalkAtRelativeSpeed, (const Pose2f&) speed);

/**
 * This skill walks to a (relative) target.
 * @param target The target pose in robot-relative coordinates
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param obstacleAvoidance The obstacle avoidance request
 * @param keepTargetRotation Whether the target rotation should be headed for all the time (instead of allowing motion to plan it)
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 * @param forceSideWalking If true, force sidewalking
 */
SKILL_INTERFACE(WalkToPose, (const Pose2f&) target, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed,
                (const MotionRequest::ObstacleAvoidance&)(MotionRequest::ObstacleAvoidance()) obstacleAvoidance,
                (bool)(false) keepTargetRotation, (const std::optional<Vector2f>&)(std::optional<Vector2f>()) targetOfInterest,
                (bool)(false) forceSideWalking);

/**
 * This skill walks to the ball and kicks it.
 * @param targetDirection The (robot-relative) direction in which the ball should go
 * @param kickType The type of kick that should be used
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll (in mm)
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param obstacleAvoidance The obstacle avoidance request
 * @param preStepAllowed Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
SKILL_INTERFACE(WalkToBallAndKick, (Angle) targetDirection, (KickInfo::KickType) kickType, (KickPrecision)(KickPrecision::notPrecise) alignPrecisely, (float)(std::numeric_limits<float>::max()) kickLength, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed, (const MotionRequest::ObstacleAvoidance&)(MotionRequest::ObstacleAvoidance()) obstacleAvoidance, (bool)(true) preStepAllowed, (bool)(true) turnKickAllowed, (bool)(false) shiftTurnKickPose, (const Rangea&)(Rangea(0_deg, 0_deg)) directionPrecision);

/**
 * This skill dribbles the ball.
 * @param targetDirection The (robot-relative) direction in which the ball should go
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param obstacleAvoidance The obstacle avoidance request
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll (in mm)
 * @param preStepAllowed Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
SKILL_INTERFACE(Dribble, (Angle) targetDirection, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed, (const MotionRequest::ObstacleAvoidance&)(MotionRequest::ObstacleAvoidance()) obstacleAvoidance, (KickPrecision)(KickPrecision::notPrecise) alignPrecisely, (float)(0.f) kickLength, (bool)(true) preStepAllowed, (bool)(true) turnKickAllowed, (const Rangea&)(Rangea(0_deg, 0_deg)) directionPrecision);

/**
 * This skill lets the robot execute a diving motion.
 * @param request The dive request
 */
SKILL_INTERFACE(Dive, (MotionRequest::Dive::Request) request);

/**
 * This skill lets the robot execute a special motion.
 * @param request The special request
 */
SKILL_INTERFACE(Special, (MotionRequest::Special::Request) request);

/**
 * This skill executes a key frame motion with both arms.
 * @param motion The motion that the arm should execute
 * @param fast Whether states should not be interpolated
 */
SKILL_INTERFACE(KeyFrameArms, (ArmKeyFrameRequest::ArmKeyFrameId) motion, (bool)(false) fast);

/**
 * This skill executes a key frame motion with a single arm.
 * @param motion The motion that the arm should execute
 * @param arm The arm that should execute the motion
 * @param fast Whether states should not be interpolated
 */
SKILL_INTERFACE(KeyFrameSingleArm, (ArmKeyFrameRequest::ArmKeyFrameId) motion, (Arms::Arm) arm, (bool)(false) fast);

/**
 * This skill lets one arm point at some point.
 * @param localPoint The point in robot-relative coordinates
 */
SKILL_INTERFACE(PointAt, (const Vector3f&) localPoint);

/**
 * This skill lets a specific arm point at some point.
 * @param localPoint The point in robot-relative coordinates
 * @param arm The arm that shall be used for pointing
 */
SKILL_INTERFACE(PointAtWithArm, (const Vector3f&) localPoint, (Arms::Arm) arm);

/** This skill reacts to arm contact by taking the arm away. */
SKILL_INTERFACE(ArmContact);

/**
 * This skill reacts to arm contact by taking the arm away (but only for one arm).
 * @param arm The arm for which the contact reaction shall be done
 */
SKILL_INTERFACE(ArmContactSingleArm, (Arms::Arm) arm);

/** This skill takes arms back to avoid obstacles. */
SKILL_INTERFACE(ArmObstacleAvoidance);

/**
 * This skill takes one arm back to avoid obstacles.
 * @param arm The arm for which the obstacle avoidance should be done
 */
SKILL_INTERFACE(ArmObstacleAvoidanceSingleArm, (Arms::Arm) arm);

/**
 * This skill moves the head to look at interesting points
 * @param withBall Whether the ball must be in the image
 * @param ignoreBall Whether the ball should be completely ignored
 * @param onlyOwnBall Whether to use only the own ball model and not the team ball model
 * @param fixTilt Whether to use a fix tilt or to allow the head control to calculate it
 */
SKILL_INTERFACE(LookActive, (bool)(false) withBall, (bool)(false) ignoreBall, (bool)(false) onlyOwnBall, (bool)(false) fixTilt);

/**
 * This skill moves the head so that a camera looks at specified angles.
 * @param pan The target pan angle
 * @param tilt The target tilt angle
 * @param speed The speed with which to move the head
 * @param camera The camera which should have the specified angles
 * @param calibrationMode Whether to set the mode to calibrationMode instead of panAndTiltMode, which disables clipping and interpolation of angles.
 */
SKILL_INTERFACE(LookAtAngles, (Angle) pan, (Angle) tilt, (float)(180_deg) speed, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera, (bool)(false) calibrationMode);

/**
 * This skill moves the head so that the ball is focused by one camera.
 * @param mirrored Whether to look at the mirrored (about the center of the field) ball position
 * @param forceOwnEstimate Whether to use only the own ball model and not the team ball model
 */
SKILL_INTERFACE(LookAtBall, (bool)(false) mirrored, (bool)(false) forceOwnEstimate);

/**
 * This skill moves the head so that the team ball is focused by one camera.
 * @param mirrored Whether to look at the mirrored (about the center of the field) ball position
 */
SKILL_INTERFACE(LookAtGlobalBall, (bool)(false) mirrored);

/**
 * This skill moves the head such that a specified (robot-relative) point is focused by one camera.
 * @param target The point to look at in robot-relative coordinates
 * @param camera The camera which should look at the point
 * @param speed The speed with which to move the head
 */
SKILL_INTERFACE(LookAtPoint, (const Vector3f&) target, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera, (Angle)(180_deg) speed);

/** This skill moves the head so that it looks forward.  */
SKILL_INTERFACE(LookForward);

/**
 * This skill moves the head alternately to the left and to the right
 * @param startLeft Whether to look left first (otherwise right)
 * @param maxPan The maximum pan angle
 * @param tilt The constant tilt angle during the motion
 * @param speed The speed with which to move the head
 */
SKILL_INTERFACE(LookLeftAndRight, (bool)(true) startLeft, (Angle)(50_deg) maxPan, (Angle)(23_deg) tilt, (Angle)(100_deg) speed);

/**
 * This skill moves the head while walking, to look at the ball and the target.
 * @param startTarget The starting position
 * @param speed the speed for moving the head
 * @param tilt The constant tilt angle during the motion
 * @param maxPan The maximum pan angle
 */
SKILL_INTERFACE(LookAtBallAndTarget, (bool)(false) startTarget, (Angle)(100_deg) speed, (Angle)(23_deg) tilt, (Angle)(10_deg) thresholdAngle, (Vector2f)(Vector2f::Zero()) walkingDirection, (Angle)(0_deg) ballPositionAngle);

/**
 * This skill moves the head in steps between the maximum pan and tilt, relative to the current orientation.
 * @param originalPan
 * @param originalTilt
 * @param maxPan The maximum absolute (left and right) horizontal deviation from the original orientation.
 * @param maxTilt The maximum absolute (up and down) vertical deviation form the original orientation.
 * @param panStep
 * @param tiltStep
 */
SKILL_INTERFACE(PanAndTiltGrid, (const HeadOrientation&) original, (const HeadOrientation&) maximum, (const Angle&) panStep, (const Angle&) tiltStep, (int) waitInPosition, (Angle)(100_deg) speed);

/**
 * This skill sets the passTarget member of the BehaviorStatus.
 * Caution: If the passTarget is set to -1, the ballTarget Vector is often used to communicate good angles instead of a position
 * @param passTarget The passTarget to set
 * @param ballTarget Optionally, the position where the ball should end up in robot-relative coordinates
 */
SKILL_INTERFACE(PassTarget, (int) passTarget, (const Vector2f&)(Vector2f::Zero()) ballTarget);

/**
 * This skill sets the calibrationFinished member of the BehaviorStatus.
 */
SKILL_INTERFACE(CalibrationFinished);

/**
 * This skill adds an annotation if it differs from the one that has been added in the last frame.
 * @param annotation The annotation message
 */
SKILL_INTERFACE(Annotation, (const std::string&) annotation);

/**
 * This skill plays a sound file if it differs from the one that has been played in the last frame.
 * @param force Play sound even if muted.
 * @param name The name of the sound file
 */
SKILL_INTERFACE(PlaySound, (const std::string&) name, (bool)(false) force);

/**
 * This skill makes the Nao say something if it differs from what was said in the last frame.
 * @param name The text to be synthesized and pronounced
 * @param force Talk even if muted.
 * @param speed Use speed < 1 to talk slower and speed > 1 to talk faster.
 */
SKILL_INTERFACE(Say, (const std::string&) text, (bool)(false) force, (float)(1.f) speed);

/** This skill makes the robot go to a ball and kick it for public demos. */
SKILL_INTERFACE(DemoGoToBallAndKick);

/** This skill makes the robot pose for public demos. */
SKILL_INTERFACE(DemoPose);

/** This skill makes the robot talk for public demos. */
SKILL_INTERFACE(DemoTalk);

/** This skill makes the robot wave for public demos. */
SKILL_INTERFACE(DemoWave);

/** This skill makes the robot search for the ball for public demos. */
SKILL_INTERFACE(DemoSearchForBall);

/** This skill controls the head for a keeper which guards its goal. */
SKILL_INTERFACE(GuardGoalHeadControl);

/**
 * This skill controls the head for a robot that walks to the ball.
 * @param distanceToTarget The distance to the kick pose (from which the ball can be kicked).
 * @param lookAtKickTarget Whether the robot is allowed to look at the kick target.
 * @param kickTargetRelative The kick target (only needed if the previous parameter is true).
 */
SKILL_INTERFACE(GoToBallHeadControl, (float) distanceToTarget, (bool)(false) lookAtKickTarget, (Vector2f)(Vector2f::Zero()) kickTargetRelative);

/**
 * This skill replays walk phases.
 */
SKILL_INTERFACE(ReplayWalk);

/**
 * This skill walks to a target using intelligent path planning.
 * @param target The target pose in robot-relative coordinates
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param reduceWalkingSpeed If true, the walking speed will get automatically reduced
 * @param rough Ignore obstacles if they prevent the robot from reaching its target
 * @param disableObstacleAvoidance Disables avoiding dynamic obstacles like robots and referees (keeps goal posts, own penalty area and field border avoidance enabled)
 * @param disableAligning Disables rotation to the walk target
 * @param disableStanding Disables standing at the target (i.e. if set to true, the robot will walk on the target spot)
 * @param disableAvoidFieldBorder Do not avoid the field border when walking.
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 * @param forceSideWalking If true, force sidewalking
 */
SKILL_INTERFACE(WalkToPoint, (const Pose2f&) target, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed, (bool)(true) reduceWalkingSpeed,
                (bool)(false) rough, (bool)(false) disableObstacleAvoidance, (bool)(false) disableAligning,
                (bool)(false) disableStanding, (bool)(false) disableAvoidFieldBorder, (const std::optional<Vector2f>&)(std::optional<Vector2f>()) targetOfInterest,
                (bool)(false) forceSideWalking);
/**
 * This skill walks to a target using intelligent path planning and shift the target pose to avoid obstacles
 * @param target The target pose in robot-relative coordinates
 * @param speed The walking speed as ratio of the maximum speed in [0, 1]
 * @param reduceWalkingSpeed If true, the walking speed will get automatically reduced
 * @param rough Ignore obstacles if they prevent the robot from reaching its target
 * @param disableObstacleAvoidance Disables avoiding dynamic obstacles like robots and referees (keeps goal posts, own penalty area and field border avoidance enabled)
 * @param disableAligning Disables rotation to the walk target
 * @param disableStanding Disables standing at the target (i.e. if set to true, the robot will walk on the target spot)
 * @param disableAvoidFieldBorder Do not avoid the field border when walking.
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 */
SKILL_INTERFACE(WalkToPointObstacle, (const Pose2f&) target, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed, (bool)(true) reduceWalkingSpeed,
                (bool)(false) rough, (bool)(false) disableObstacleAvoidance, (bool)(false) disableAligning,
                (bool)(false) disableStanding, (bool)(false) disableAvoidFieldBorder, (const std::optional<Vector2f>&)(std::optional<Vector2f>()) targetOfInterest,
                (bool)(false) forceSideWalking);

/**
 * This skill walks to a kick-off or penalty kick pose (i.e. in the ready state).
 * @param target The target pose in absolute field coordinates
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 */
SKILL_INTERFACE(WalkToPointReady, (const Pose2f&) target, (const std::optional<Vector2f>&)(std::optional<Vector2f>()) targetOfInterest);

/**
 * This skill walks to a target that is modified by a potential field.
 * @param target The target pose in absolute field coordinates
 * @param playerNumber The number of the player to support or -1
 * @param straight Walk straight to the target (ballFactor, useRotation and rotation are ignored)
 * @param ballFactor How much should the robot turn to the ball when moving to its target in [0, 1]
 * @param useRotation Use \c rotation when close the target instead of the rotation to the ball
 * @param rotation The rotation relative to the robot if \c useRotation is true
 * @param targetOfInterest If set, it is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. (Only applies for side walk)
 */
SKILL_INTERFACE(WalkPotentialField, (const Vector2f&) target, (int) playerNumber, (bool)(false) straight, (float)(0.5f) ballFactor, (bool)(false) useRotation, (float)(0.f) rotation, (const std::optional<Vector2f>&)(std::optional<Vector2f>()) targetOfInterest);

/**
 * This skill turns the robot on the spot by a specified angle.
 * @param angle The angle relative to the rotation that the robot had when the skill started
 * @param margin The tolerance for the skill to be done
 */
SKILL_INTERFACE(TurnAngle, (Angle) angle, (Angle)(5_deg) margin);

/**
 * This skill turns the robot to look forward at a target.
 * @param target The position in robot relative coordinates to turn to
 * @param margin The tolerance for the skill to be done
 */
SKILL_INTERFACE(TurnToPoint, (const Vector2f&) target, (Angle)(5_deg) margin);

/**
 * This skill walks to the ball and dribbles it from there.
 * @param targetDirection The direction to which the ball should be dribbled in robot-relative coordinates
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param kickLength The distance the ball shall roll
 * @param lookActiveWithBall If true, use LookActive but with the flag withBall = true
 * @param preStepAllowed Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
SKILL_INTERFACE(GoToBallAndDribble, (Angle) targetDirection, (KickPrecision)(KickPrecision::notPrecise) alignPrecisely, (float)(750.f) kickLength, (bool)(false) lookActiveWithBall, (bool)(true) preStepAllowed, (bool)(true) turnKickAllowed, (const Rangea&)(Rangea(0_deg, 0_deg)) directionPrecision);

/**
 * This skill dribbles the ball to the goal.
 */
SKILL_INTERFACE(DribbleToGoal);

/**
 * This skill walks to the ball and executes a kick there.
 * @param targetDirection The direction to which the ball should be kicked in robot-relative coordinates
 * @param kickType The kick type that should be executed there
 * @param lookActiveWithBall If true, use LookActive but with the flag withBall = true
 * @param alignPrecisely Whether the robot should align more precisely than usual
 * @param length The desired length of the kick (works only for certain types of kicks)
 * @param preStepAllowed Is a prestep for the InWalkKick allowed?
 * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
 * @param speed The walking speed
 * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the WalkToBallAndKickEngine uses its own precision.
 */
SKILL_INTERFACE(GoToBallAndKick, (Angle) targetDirection, (KickInfo::KickType) kickType, (bool)(false) lookActiveWithBall, (KickPrecision)(KickPrecision::notPrecise) alignPrecisely, (float)(std::numeric_limits<float>::max()) length, (bool)(true) preStepAllowed, (bool)(true) turnKickAllowed, (bool)(false) shiftTurnKickPose, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed, (const Rangea&)(Rangea(0_deg, 0_deg)) directionPrecision);

/**
 * This skill walks very carefully to a kick pose and executes a kick there.
 * @param kickPose The pose at which the kick should be executed in robot-relative coordinates
 * @param kickType The kick type that should be executed there
 * @param walkSpeed The walking speed as ratio of the maximum speed in [0, 1]
 */
SKILL_INTERFACE(PenaltyStrikerGoToBallAndKick, (const Pose2f&) kickPose, (KickInfo::KickType) kickType, (float) walkSpeed);

/**
 * This skill marks another robot (i.e. positioning between it and the ball to make it unattractive as pass target).
 * This robot will be facing the ball and thus probably not see the marked robot.
 * @param target The position of the marked robot in robot-relative coordinates
 */
SKILL_INTERFACE(Mark, (const Vector2f&) target);

/**
 * This skill blocks another robot (i.e. preventing it from moving, especially towards the ball).
 * This robot will be facing the other robot and thus probably not see the ball.
 * @param target The position of the blocked robot in robot-relative coordinates
 * @param useAlternativeBall Whether the alternative ball position should be used (instead of something taken directly from a model)
 * @param alternativeBall The alternative ball position in robot-relative coordinates
 */
SKILL_INTERFACE(Block, (const Vector2f&) target, (bool)(false) useAlternativeBall, (Vector2f)(Vector2f::Zero()) alternativeBall);

/**
 * This skill intercepts a rolling ball with the goal that it does not pass the y axis of this robot.
 * @param interceptionMethods A bit set of methods that may be used to intercept the ball (from Interception::Method).
 * @param allowGetUp Whether the robot is allowed to get up afterwards.
 * @param allowDive Whether the robot is allowed to actually dive. Otherwise, only sounds are played to indicate what it would do.
 */
SKILL_INTERFACE(InterceptBall, (unsigned) interceptionMethods, (bool)(true) allowGetUp, (bool)(true) allowDive);

/**
 * This skill observes a point by walking next to it and look at it.
 * @param target The point to observe.
 */
SKILL_INTERFACE(ObservePoint, (const Vector2f&) target);

/** Skill version of the KickAtGoalCard (combined with DribbleToGoalCard), only used in the 2021 behavior control. */
SKILL_INTERFACE(KickAtGoal);

/** Skill version of the HandleBallAtOwnGoalPostcard, only used in the 2021 behavior control. */
SKILL_INTERFACE(HandleBallAtOwnGoalPost);

/**
 * This skill passes the ball to a teammate.
 * @param playerNumber The player number of the pass target.
 */
SKILL_INTERFACE(PassToTeammate, (int) playerNumber);

/**
 * This skill receives the ball passed from a teammate, only used by the 2021 behavior control.
 * @param playerNumber The player number passing the ball.
 */
SKILL_INTERFACE(ReceivePass, (int) playerNumber);

/**
 * Skill for dueling an opponent.
 */
SKILL_INTERFACE(Zweikampf);

/**
 * This skill does just a kick-off into the opponent's half.
 */
SKILL_INTERFACE(DirectKickOff);

/**
 * This skill plays the ball under consideration of the skill request.
 */
SKILL_INTERFACE(PlayBall);

/**
 * This skill records the walkingTo target and desired speed in the BehaviorStatus
 * to be published to teammates.
 * @param target The target the robot is walking towards.
 * @param speed The desired relative speed.
 */
SKILL_INTERFACE(PublishMotion, (const Vector2f) target, (const Pose2f&)(Pose2f(1.f, 1.f, 1.f)) speed);

/**
 * This skill can be used to calibrate the robot.
 */
SKILL_INTERFACE(CalibrateRobot, (const CalibrationRequest&) request);

/**
 * Sets the motion request to calibration
 */
SKILL_INTERFACE(CalibrateFootSole);

/**
 * Lets the robot start the IMU calibration.
 */
SKILL_INTERFACE(AutomaticIMUCalibration);

/**
 * Lets the robot start the extrinsic camera calibration.
 */
SKILL_INTERFACE(AutonomousCameraCalibration);

/**
 * Lets the robot start the calibration process.
 */
SKILL_INTERFACE(CalibrationControl);

/**
 * Enables neural control
 */
SKILL_INTERFACE(NeuralControl, (const Pose2f&) target);


/**
 * Enables neural control
 */
SKILL_INTERFACE(NeuralControlTraining, (const Pose2f&) target);
