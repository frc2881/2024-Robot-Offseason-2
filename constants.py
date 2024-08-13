import math
from wpilib import SerialPort
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from photonlibpy.photonPoseEstimator import PoseStrategy
from extras.pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from extras.pathplannerlib.pathfinding import PathConstraints
from extras.pathplannerlib.path import PathPlannerPath
from lib.classes import PIDConstants
from classes import AutoPath, LauncherRollersSpeeds, LauncherArmPositionTarget

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)
    kDriveBaseRadius: units.meters = Translation2d().distance(Translation2d(kWheelBase / 2, kTrackWidth / 2))

    kTranslationSpeedMax: units.meters_per_second = 6.32
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi

    kInputLimit: units.percent = 0.6
    kInputRateLimit: units.percent = 0.5

    kDriftCorrectionThetaControllerPIDConstants = PIDConstants(0.01, 0, 0, 0)
    kDriftCorrectionThetaControllerPositionTolerance: float = 0.5
    kDriftCorrectionThetaControllerVelocityTolerance: float = 0.5

    kTargetAlignmentThetaControllerPIDConstants = PIDConstants(0.075, 0, 0, 0)
    kTargetAlignmentThetaControllerPositionTolerance: float = 1.0
    kTargetAlignmentThetaControllerVelocityTolerance: float = 1.0
    kTargetAlignmentCarpetFrictionCoeff: float = 0.15
    kTargetAlignmentHeadingInversion: units.degrees = 180.0

    kPathFollowerTranslationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFollowerRotationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFindingConstraints = PathConstraints(4.8, 3.2, units.degreesToRadians(540), units.degreesToRadians(720))

    kSwerveModuleFrontLeftDrivingMotorCANId: int = 3
    kSwerveModuleFrontLeftTurningMotorCANId: int = 4
    kSwerveModuleFrontRightDrivingMotorCANId: int = 7 
    kSwerveModuleFrontRightTurningMotorCANId: int = 8 
    kSwerveModuleRearLeftDrivingMotorCANId: int = 5 
    kSwerveModuleRearLeftTurningMotorCANId: int = 6 
    kSwerveModuleRearRightDrivingMotorCANId: int = 9 
    kSwerveModuleRearRightTurningMotorCANId: int = 10 

    kSwerveModuleFrontLeftTurningOffset: units.radians = -math.pi / 2
    kSwerveModuleFrontRightTurningOffset: units.radians = 0
    kSwerveModuleRearLeftTurningOffset: units.radians = math.pi
    kSwerveModuleRearRightTurningOffset: units.radians = math.pi / 2

    kSwerveModuleFrontLeftTranslation = Translation2d(kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleFrontRightTranslation = Translation2d(kWheelBase / 2, -kTrackWidth / 2)
    kSwerveModuleRearLeftTranslation = Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleRearRightTranslation = Translation2d(-kWheelBase / 2, -kTrackWidth / 2)

    kSwerveDriveKinematics = SwerveDrive4Kinematics(
      kSwerveModuleFrontLeftTranslation, 
      kSwerveModuleFrontRightTranslation, 
      kSwerveModuleRearLeftTranslation, 
      kSwerveModuleRearRightTranslation
    )

    class SwerveModule:
      kFreeSpeed: units.revolutions_per_minute = 6784
      kWheelDiameter: units.meters = units.inchesToMeters(3.0)
      kWheelCircumference: units.meters = kWheelDiameter * math.pi
      kDrivingMotorPinionTeeth: int = 14
      kDrivingMotorReduction: float = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15)
      kDrivingMotorFreeSpeedRps: float = kFreeSpeed / 60
      kDriveWheelFreeSpeedRps: float = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction 
      kDrivingEncoderPositionConversionFactor: float = (kWheelDiameter * math.pi) / kDrivingMotorReduction
      kDrivingEncoderVelocityConversionFactor: float = ((kWheelDiameter * math.pi) / kDrivingMotorReduction) / 60.0
      kTurningEncoderInverted: bool = True
      kTurningEncoderPositionConversionFactor: float = 2 * math.pi
      kTurningEncoderVelocityConversionFactor: float = (2 * math.pi) / 60.0
      kTurningEncoderPositionPIDMinInput: float = 0
      kTurningEncoderPositionPIDMaxInput: float = kTurningEncoderPositionConversionFactor
      kDrivingMotorCurrentLimit: units.amperes = 80
      kDrivingMotorMaxReverseOutput: units.percent = -1.0
      kDrivingMotorMaxForwardOutput: units.percent = 1.0
      kDrivingMotorPIDConstants = PIDConstants(0.04, 0, 0, 1 / kDriveWheelFreeSpeedRps)
      kTurningMotorCurrentLimit: units.amperes = 20
      kTurningMotorMaxReverseOutput: units.percent = -1.0
      kTurningMotorMaxForwardOutput: units.percent = 1.0
      kTurningMotorPIDConstants = PIDConstants(1, 0, 0, 0)
 
  class Intake:
    kRollerMotorCANId: int = 11
    kTopMotorCANId: int = 12
    kBottomMotorCANId: int = 13

    kMotorCurrentLimit: units.amperes = 60
    kMotorMaxForwardOutput: units.percent = 1.0
    kMotorMaxReverseOutput: units.percent = -1.0

    kSpeedIntake: units.percent = 0.8
    kSpeedAlign: units.percent = 0.1
    kSpeedEject: units.percent = 0.3
    kSpeedLaunch: units.percent = 1.0
    kSpeedScoreAmp: units.percent = 0.3

    kDistanceIntake: units.millimeters = 40.0
    kDistanceLauncherReadyMin: units.millimeters = 1.0
    kDistanceLauncherReadyMax: units.millimeters = 220.0

    kAlignTimeout: units.seconds = 0.2
    kReloadTimeout: units.seconds = 0.4

  class Launcher:
    class Arm:
      kMotorCANId: int = 14

      kMotorCurrentLimit: units.amperes = 60
      kMotorMaxReverseOutput: units.percent = -1.0
      kMotorMaxForwardOutput: units.percent = 1.0
      kMotorPIDConstants = PIDConstants(0.0003, 0, 0.00015, 1 / 16.8)
      kMotorForwardSoftLimit: float = 22.5
      kMotorReverseSoftLimit: float = 0
      kMotorPositionConversionFactor: float = 1.0 / 3.0
      kMotorVelocityConversionFactor: float = kMotorPositionConversionFactor / 60.0
      kMotorSmartMotionMaxVelocity: float = (33.0 / kMotorPositionConversionFactor) * 60
      kMotorSmartMotionMaxAccel: float = 100.0 / kMotorVelocityConversionFactor 

      kInputLimit: units.percent = 0.5
      kResetSpeed: units.percent = 0.1

      kTargetAlignmentPositionTolerance: float = 0.05

      kPositionSubwoofer: float = 7.5
      kPositionPodium: float = 4.7
      kPositionAmp: float = 23.0
      kPositionShuttle: float = 5.0
      kPositionIntake: float = 0.00

      kPositionTargets: list[LauncherArmPositionTarget] = [
        LauncherArmPositionTarget(0.00, 8.2),
        LauncherArmPositionTarget(0.75, 7.9),
        LauncherArmPositionTarget(1.25, kPositionSubwoofer),
        LauncherArmPositionTarget(2.50, 5.3),
        LauncherArmPositionTarget(2.90, kPositionPodium),
        LauncherArmPositionTarget(3.45, 3.80),
        LauncherArmPositionTarget(4.00, 3.20),
        LauncherArmPositionTarget(4.75, 2.30),
        LauncherArmPositionTarget(5.15, 1.85),
        LauncherArmPositionTarget(6.05, 1.60),
        LauncherArmPositionTarget(7.00, 1.40)
      ]

    class Rollers:
      kBottomMotorCANId: int = 15
      kTopMotorCANId: int = 16

      kMotorFreeSpeed: units.revolutions_per_minute = 6784

      kMotorCurrentLimit: units.amperes = 100
      kMotorMaxForwardOutput: units.percent = 1.0
      kMotorMaxReverseOutput: units.percent = -1.0

      kSpeedsDefault = LauncherRollersSpeeds(1.0, 1.0)
      kSpeedsShuttle = LauncherRollersSpeeds(1.0, 1.0)

      kLaunchSpeedDeltaMin: units.percent = 0.95

class Sensors:
  class Gyro:
    class NAVX2:
      kSerialPort = SerialPort.Port.kUSB1

  class Pose:
    kPoseSensors: dict[str, Transform3d] = {
      "Rear": Transform3d(
        Translation3d(units.inchesToMeters(5.5), units.inchesToMeters(0.0), units.inchesToMeters(21.0)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.3), units.degreesToRadians(-180.0))
      ),
      "Front": Transform3d(
        Translation3d(units.inchesToMeters(8.75), units.inchesToMeters(4.25), units.inchesToMeters(21.75)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.0), units.degreesToRadians(0.0))
      )
      # "Left": Transform3d(
      #   Translation3d(units.inchesToMeters(8.75), units.inchesToMeters(4.25), units.inchesToMeters(21.75)),
      #   Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.0), units.degreesToRadians(0.0))
      # ),
      # "Right": Transform3d(
      #   Translation3d(units.inchesToMeters(8.75), units.inchesToMeters(4.25), units.inchesToMeters(21.75)),
      #   Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.0), units.degreesToRadians(0.0))
      # )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kVisionSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kVisionMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kVisionMaxPoseAmbiguity: units.percent = 0.2

  class Distance:
    class Launcher:
      kSensorName = "Launcher"
      kMinTargetDistance: units.millimeters = 1
      kMaxTargetDistance: units.millimeters = 320

  class Camera:
    kStreams: dict[str, str] = {
      "Rear": "http://10.28.81.6:1182/?action=stream",
      "Front": "http://10.28.81.6:1184/?action=stream",
      "Left": "http://10.28.81.7:1184/?action=stream",
      "Right": "http://10.28.81.6:1186/?action=stream",
      "Driver": "http://10.28.81.7:1182/?action=stream"
    }

_aprilTagFieldLayout = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

class Game:
  class Commands:
    kScoringAlignmentTimeout: units.seconds = 1.2
    kScoringLaunchTimeout: units.seconds = 1.35

  class Field:
    kAprilTagFieldLayout = _aprilTagFieldLayout
    kLength = kAprilTagFieldLayout.getFieldLength()
    kWidth = kAprilTagFieldLayout.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:  
      kBlueSpeaker = _aprilTagFieldLayout.getTagPose(7) or Pose3d()
      kRedSpeaker = _aprilTagFieldLayout.getTagPose(4) or Pose3d()
      kBlueAmp = _aprilTagFieldLayout.getTagPose(5) or Pose3d()
      kRedAmp = _aprilTagFieldLayout.getTagPose(6) or Pose3d()

      kSpeakerTargetTransform = Transform3d(
        units.inchesToMeters(6.0),
        units.inchesToMeters(-6.0),
        units.inchesToMeters(24),
        Rotation3d()
      )

  class Auto:
    kPaths: dict[AutoPath, PathPlannerPath] = {
      AutoPath.Test: PathPlannerPath.fromPathFile(AutoPath.Test.name),
      AutoPath.ScorePreload1: PathPlannerPath.fromPathFile(AutoPath.ScorePreload1.name),
      AutoPath.ScorePreload2: PathPlannerPath.fromPathFile(AutoPath.ScorePreload2.name),
      AutoPath.ScorePreload3: PathPlannerPath.fromPathFile(AutoPath.ScorePreload3.name),
      AutoPath.Pickup1: PathPlannerPath.fromPathFile(AutoPath.Pickup1.name),
      AutoPath.Pickup12: PathPlannerPath.fromPathFile(AutoPath.Pickup12.name),
      AutoPath.Pickup2: PathPlannerPath.fromPathFile(AutoPath.Pickup2.name),
      AutoPath.Pickup21: PathPlannerPath.fromPathFile(AutoPath.Pickup21.name),
      AutoPath.Pickup23: PathPlannerPath.fromPathFile(AutoPath.Pickup23.name),
      AutoPath.Pickup3: PathPlannerPath.fromPathFile(AutoPath.Pickup3.name),
      AutoPath.Pickup32: PathPlannerPath.fromPathFile(AutoPath.Pickup32.name),
      AutoPath.Pickup4: PathPlannerPath.fromPathFile(AutoPath.Pickup4.name),
      AutoPath.Pickup5: PathPlannerPath.fromPathFile(AutoPath.Pickup5.name),
      AutoPath.Pickup61: PathPlannerPath.fromPathFile(AutoPath.Pickup61.name),
      AutoPath.Pickup62: PathPlannerPath.fromPathFile(AutoPath.Pickup62.name),
      AutoPath.Pickup63: PathPlannerPath.fromPathFile(AutoPath.Pickup63.name),
      AutoPath.Pickup72: PathPlannerPath.fromPathFile(AutoPath.Pickup72.name),
      AutoPath.Pickup73: PathPlannerPath.fromPathFile(AutoPath.Pickup73.name),
      AutoPath.Pickup8: PathPlannerPath.fromPathFile(AutoPath.Pickup8.name),
      AutoPath.ScoreStage1: PathPlannerPath.fromPathFile(AutoPath.ScoreStage1.name),
      AutoPath.ScoreStage2: PathPlannerPath.fromPathFile(AutoPath.ScoreStage2.name),
      AutoPath.ScoreStage3: PathPlannerPath.fromPathFile(AutoPath.ScoreStage3.name)
    }

    kPickupTimeout: units.seconds = 4.0
