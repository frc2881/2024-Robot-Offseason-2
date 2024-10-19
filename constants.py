import math
from wpilib import SerialPort
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from pathplannerlib.pathfinding import PathConstraints
from lib.classes import PIDConstants, MotorControllerType, ChassisLocation, SwerveModuleConfig
from classes import LauncherRollersSpeeds, LauncherArmPositionTarget

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)
    kDriveBaseRadius: units.meters = Translation2d().distance(Translation2d(kWheelBase / 2, kTrackWidth / 2))

    kTranslationSpeedMax: units.meters_per_second = 6.32
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi # type: ignore

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

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
    kPathFindingConstraints = PathConstraints(4.2, 3.6, units.degreesToRadians(360), units.degreesToRadians(720))

    kSwerveModules = (
      SwerveModuleConfig(ChassisLocation.FrontLeft, 3, 4, -math.pi / 2, Translation2d(kWheelBase / 2, kTrackWidth / 2)),
      SwerveModuleConfig(ChassisLocation.FrontRight, 7, 8, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2)),
      SwerveModuleConfig(ChassisLocation.RearLeft, 5, 6, math.pi, Translation2d(-kWheelBase / 2, kTrackWidth / 2)),
      SwerveModuleConfig(ChassisLocation.RearRight, 9, 10, math.pi / 2, Translation2d(-kWheelBase / 2, -kTrackWidth / 2))
    )

    kSwerveDriveKinematics = SwerveDrive4Kinematics(
      kSwerveModules[0].translation,
      kSwerveModules[1].translation, 
      kSwerveModules[2].translation,
      kSwerveModules[3].translation
    )

    class SwerveModule:
      kWheelDiameter: units.meters = units.inchesToMeters(3.0)
      kWheelCircumference: units.meters = kWheelDiameter * math.pi
      kWheelBevelGearTeeth: int = 45
      kWheelSpurGearTeeth: int = 20
      kWheelBevelPinionTeeth: int = 15
      kDrivingMotorControllerType = MotorControllerType.SparkFlex
      kDrivingMotorFreeSpeed: units.revolutions_per_minute = 6784
      kDrivingMotorPinionTeeth: int = 14
      kDrivingMotorReduction: float = (kWheelBevelGearTeeth * kWheelSpurGearTeeth) / (kDrivingMotorPinionTeeth * kWheelBevelPinionTeeth)
      kDrivingMotorFreeSpeedRps: float = kDrivingMotorFreeSpeed / 60
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
    kSpeedScoreAmp: units.percent = 0.5

    kDistanceIntake: units.millimeters = 40.0
    kDistanceLauncherReadyMin: units.millimeters = 1.0
    kDistanceLauncherReadyMax: units.millimeters = 220.0

    kAlignTimeout: units.seconds = 0.1
    kReloadTimeout: units.seconds = 0.2

  class Launcher:
    class Arm:
      kRightMotorCANId: int = 14
      kLeftMotorCANId: int = 15

      kMotorCurrentLimit: units.amperes = 60
      kMotorMaxForwardOutput: units.percent = 1.0
      kMotorMaxReverseOutput: units.percent = -1.0
      kMotorPIDConstants = PIDConstants(0.0003, 0, 0.00015, 1 / 565)
      kMotorForwardSoftLimit: float = 22.50
      kMotorReverseSoftLimit: float = 0
      kMotorPositionConversionFactor: float = 1.0 / 3.0
      kMotorVelocityConversionFactor: float = kMotorPositionConversionFactor / 60.0
      kMotorSmartMotionMaxVelocity: float = (33.0 / kMotorPositionConversionFactor) * 60
      kMotorSmartMotionMaxAccel: float = 66.0 / kMotorVelocityConversionFactor 

      kInputLimit: units.percent = 0.5
      kResetSpeed: units.percent = 0.1

      kTargetAlignmentPositionTolerance: float = 0.05

      kPositionIntake: float = 0.05
      kPositionSubwoofer: float = 6.90
      kPositionPodium: float = 2.30
      kPositionAmp: float = 22.50
      kPositionShuttle: float = 4.00
      kPositionClimbUp: float = 22.50
      kPositionClimbDown: float = 4.40

      kPositionTargets: list[LauncherArmPositionTarget] = [
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
      ]

    class Rollers:
      kBottomMotorCANId: int = 16
      kTopMotorCANId: int = 17

      kMotorFreeSpeed: units.revolutions_per_minute = 6784
      kMotorCurrentLimit: units.amperes = 120
      kMotorMaxForwardOutput: units.percent = 1.0
      kMotorMaxReverseOutput: units.percent = -1.0

      kSpeedsDefault = LauncherRollersSpeeds(1.0, 1.0)
      kSpeedsShuttle = LauncherRollersSpeeds(0.65, 0.65)
      kSpeedsDemo = LauncherRollersSpeeds(0.3, 0.3)

      kLaunchSpeedDeltaMin: units.percent = 0.90

class Sensors:
  class Gyro:
    class NAVX2:
      kSerialPort = SerialPort.Port.kUSB1

  class Pose:
    kPoseSensors: dict[ChassisLocation, Transform3d] = {
      ChassisLocation.Rear: Transform3d(
        Translation3d(units.inchesToMeters(5.49), units.inchesToMeters(0.0), units.inchesToMeters(20.60)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.2), units.degreesToRadians(-180.0))
      ),
      ChassisLocation.Front: Transform3d(
        Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(21.25)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-22.3), units.degreesToRadians(0.0))
      ),
      ChassisLocation.Left: Transform3d(
        Translation3d(units.inchesToMeters(8.24), units.inchesToMeters(12.40), units.inchesToMeters(17.25)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-29.4), units.degreesToRadians(90.0))
      ),
      ChassisLocation.Right: Transform3d(
        Translation3d(units.inchesToMeters(8.16), units.inchesToMeters(-12.375), units.inchesToMeters(17.25)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-21.2), units.degreesToRadians(-90.0))
      )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kMaxPoseAmbiguity: units.percent = 0.2

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

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

class Game:
  class Commands:
    kScoringAlignmentTimeout: units.seconds = 0.8
    kScoringLaunchTimeout: units.seconds = 1.0
    kAutoPickupTimeout: units.seconds = 4.0

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