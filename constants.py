import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import PID, MotorControllerType, SwerveModuleConstants, SwerveModuleConfig, SwerveModuleLocation, PoseSensorConfig, PoseSensorLocation, LeadscrewModuleConstants, LeadscrewModuleConfig
from classes import LauncherRollersSpeeds, LauncherArmPositionTarget

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)

    kTranslationSpeedMax: units.meters_per_second = 6.32
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi # type: ignore

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 20,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 14,
      drivingMotorFreeSpeed = 6784,
      drivingMotorControllerType = MotorControllerType.SparkFlex,
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1, 0, 0)
    )

    kSwerveModuleConfigs: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -math.pi / 2, Translation2d(kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, math.pi, Translation2d(-kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, math.pi / 2, Translation2d(-kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants)
    )

    kDriveKinematics = SwerveDrive4Kinematics(*(c.translation for c in kSwerveModuleConfigs))

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPHolonomicDriveController(
      translation_constants = PIDConstants(5.0, 0, 0),
      rotation_constants = PIDConstants(5.0, 0, 0)
    )
    kPathFindingConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kDriftCorrectionControllerPID = PID(0.01, 0, 0)
    kDriftCorrectionPositionTolerance: float = 0.5
    kDriftCorrectionVelocityTolerance: float = 0.5

    kTargetAlignmentControllerPID = PID(0.075, 0, 0)
    kTargetAlignmentPositionTolerance: float = 1.0
    kTargetAlignmentVelocityTolerance: float = 1.0
    kTargetAlignmentCarpetFrictionCoeff: float = 0.2
    kTargetAlignmentHeadingAdjustment: units.degrees = 180.0

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

  class Localization:
    kSingleTagStandardDeviations: tuple[float, ...] = (1.0, 1.0, 2.0)
    kMultiTagStandardDeviations: tuple[float, ...] = (0.5, 0.5, 1.0)
    kMaxPoseAmbiguity: units.percent = 0.2
 
  class Intake:
    kRollerMotorCANId: int = 11
    kTopMotorCANId: int = 12
    kBottomMotorCANId: int = 13

    kMotorCurrentLimit: units.amperes = 60

    kSpeedIntake: units.percent = 0.8
    kSpeedAlign: units.percent = 0.1
    kSpeedEject: units.percent = 0.3
    kSpeedLaunch: units.percent = 1.0
    kSpeedScoreAmp: units.percent = 0.5

    kDistanceIntake: units.millimeters = 40.0
    kDistanceLauncherReadyMin: units.millimeters = 1.0
    kDistanceLauncherReadyMax: units.millimeters = 220.0

    kAlignTimeout: units.seconds = 0.1
    kReloadTimeout: units.seconds = 0.2

  class Launcher:
    class Arm:
      _leadscrewModuleConstants = LeadscrewModuleConstants(
        leadscrewTravelDistance = 1.0,
        motorControllerType = MotorControllerType.SparkFlex,
        motorCurrentLimit = 60,
        motorReduction = 3.0,
        motorPID = PID(0.0003, 0, 0.00015),
        motorSmartMotionMaxVelocityRate = 33.0,
        motorSmartMotionMaxAccelerationRate = 66.0,
        motorSoftLimitForward = 22.50,
        motorSoftLimitReverse = 0,
        motorResetSpeed = 0.1
      )

      kLeadScrewModuleConfigLeft = LeadscrewModuleConfig("Launcher/Arm/Leadscrews/Left", 15, _leadscrewModuleConstants)
      kLeadScrewModuleConfigRight = LeadscrewModuleConfig("Launcher/Arm/Leadscrews/Right", 14, _leadscrewModuleConstants)
      
      kPositionIntake: float = 0.05
      kPositionSubwoofer: float = 6.90
      kPositionPodium: float = 2.30
      kPositionAmp: float = 22.50
      kPositionShuttle: float = 4.00
      kPositionClimbUp: float = 22.50
      kPositionClimbDown: float = 4.40
      kPositionTargets: tuple[LauncherArmPositionTarget, ...] = (
        LauncherArmPositionTarget(0.00, 7.20),
        LauncherArmPositionTarget(0.50, 7.00),
        LauncherArmPositionTarget(1.25, kPositionSubwoofer),
        LauncherArmPositionTarget(2.00, 3.90),
        LauncherArmPositionTarget(2.50, 2.80),
        LauncherArmPositionTarget(3.00, kPositionPodium),
        LauncherArmPositionTarget(3.45, 1.90),
        LauncherArmPositionTarget(4.00, 1.40),
        LauncherArmPositionTarget(4.75, 1.00),
        LauncherArmPositionTarget(5.70, 0.80),
        LauncherArmPositionTarget(6.00, 0.50),
        LauncherArmPositionTarget(7.00, 0.30),
        LauncherArmPositionTarget(8.00, 0.05)
      )

      kTargetAlignmentPositionTolerance: float = 0.05

      kInputLimit: units.percent = 0.5

    class Rollers:
      kBottomMotorCANId: int = 16
      kTopMotorCANId: int = 17

      kMotorFreeSpeed: units.revolutions_per_minute = 6784
      kMotorCurrentLimit: units.amperes = 120

      kSpeedsDefault = LauncherRollersSpeeds(1.0, 1.0)
      kSpeedsShuttle = LauncherRollersSpeeds(0.65, 0.65)
      kSpeedsDemo = LauncherRollersSpeeds(0.3, 0.3)

      kLaunchSpeedDeltaMin: units.percent = 0.90

class Sensors:
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    _fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        PoseSensorLocation.Front,
        Transform3d(
          Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(21.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-22.3), units.degreesToRadians(0.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        PoseSensorLocation.Rear,
        Transform3d(
          Translation3d(units.inchesToMeters(5.49), units.inchesToMeters(0.0), units.inchesToMeters(20.60)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.2), units.degreesToRadians(-180.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        PoseSensorLocation.Left,
        Transform3d(
          Translation3d(units.inchesToMeters(8.24), units.inchesToMeters(12.40), units.inchesToMeters(17.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-29.4), units.degreesToRadians(90.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        PoseSensorLocation.Right,
        Transform3d(
          Translation3d(units.inchesToMeters(8.16), units.inchesToMeters(-12.375), units.inchesToMeters(17.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-21.2), units.degreesToRadians(-90.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      )
    )

  class Distance:
    class Launcher:
      kSensorName = "Launcher"
      kMinTargetDistance: units.millimeters = 1
      kMaxTargetDistance: units.millimeters = 320

  class Object:
    class Intake:
      kCameraName = "Intake"

  class Camera:
    kStreams: dict[str, str] = {
      "Rear": "http://10.28.81.6:1182/?action=stream",
      "Front": "http://10.28.81.6:1184/?action=stream",
      "Left": "http://10.28.81.7:1186/?action=stream",
      "Right": "http://10.28.81.7:1184/?action=stream",
      "Driver": "http://10.28.81.6:1188/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kScoringAlignmentTimeout: units.seconds = 0.8
    kScoringLaunchTimeout: units.seconds = 1.0
    kAutoPickupTimeout: units.seconds = 4.0
    kAutoMoveTimeout: units.seconds = 4.0

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kBlueTarget = APRIL_TAG_FIELD_LAYOUT.getTagPose(7) or Pose3d()
      kRedTarget = APRIL_TAG_FIELD_LAYOUT.getTagPose(4) or Pose3d()

      kTargetTransform = Transform3d(
        units.inchesToMeters(6.0),
        units.inchesToMeters(12.0),
        units.inchesToMeters(24),
        Rotation3d()
      )
