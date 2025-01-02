from typing import Callable
import math
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkFlex, ClosedLoopConfig
from lib import logger, utils
import constants

class LauncherArmSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Launcher.Arm

    self._hasInitialZeroReset: bool = False
    self._isAlignedToTarget: bool = False
    self._targetDistances = tuple(t.distance for t in self._constants.kPositionTargets)
    self._targetPositions = tuple(t.position for t in self._constants.kPositionTargets)
    
    SmartDashboard.putString("Robot/Launcher/Arm/Positions", utils.toJson(self._constants.kPositionTargets))

    self._armLeftMotor = SparkFlex(self._constants.kLeftMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._armLeftMotorConfig = SparkBaseConfig()
    (self._armLeftMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    (self._armLeftMotorConfig.encoder
      .positionConversionFactor(self._constants.kMotorPositionConversionFactor)
      .velocityConversionFactor(self._constants.kMotorVelocityConversionFactor))
    (self._armLeftMotorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._constants.kMotorPID)
      .velocityFF(self._constants.kMotorVelocityFeedForward)
      .outputRange(self._constants.kMotorMaxReverseOutput, self._constants.kMotorMaxForwardOutput)
      .smartMotion
        .maxVelocity(self._constants.kMotorSmartMotionMaxVelocity)
        .maxAcceleration(self._constants.kMotorSmartMotionMaxAcceleration))
    (self._armLeftMotorConfig.softLimit
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(self._constants.kMotorForwardSoftLimit)
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(self._constants.kMotorReverseSoftLimit))
    utils.setSparkConfig(
      self._armLeftMotor.configure(
        self._armLeftMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._armLeftClosedLoopController = self._armLeftMotor.getClosedLoopController()    
    self._armLeftEncoder = self._armLeftMotor.getEncoder()
    self._armLeftEncoder.setPosition(0)

    self._armRightMotor = SparkFlex(self._constants.kRightMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._armRightMotorConfig = SparkBaseConfig()
    (self._armRightMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    (self._armRightMotorConfig.encoder
      .positionConversionFactor(self._constants.kMotorPositionConversionFactor)
      .velocityConversionFactor(self._constants.kMotorVelocityConversionFactor))
    (self._armRightMotorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._constants.kMotorPID)
      .velocityFF(self._constants.kMotorVelocityFeedForward)
      .outputRange(self._constants.kMotorMaxReverseOutput, self._constants.kMotorMaxForwardOutput)
      .smartMotion
        .maxVelocity(self._constants.kMotorSmartMotionMaxVelocity)
        .maxAcceleration(self._constants.kMotorSmartMotionMaxAcceleration))
    (self._armRightMotorConfig.softLimit
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(self._constants.kMotorForwardSoftLimit)
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(self._constants.kMotorReverseSoftLimit))
    self._armRightMotorConfig.follow(self._constants.kLeftMotorCANId)
    utils.setSparkConfig(
      self._armRightMotor.configure(
        self._armRightMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._armRightClosedLoopController = self._armRightMotor.getClosedLoopController()    
    self._armRightEncoder = self._armRightMotor.getEncoder()
    self._armRightEncoder.setPosition(0)

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: [
        self._armLeftMotor.set(getInput() * self._constants.kInputLimit)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:Run")

  def alignToPositionCommand(self, position: float) -> Command:
    return self.run(
      lambda: [
        self._armLeftClosedLoopController.setReference(position, SparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self._alignToIntake()
    ).withName("LauncherArmSubsystem:AlignToPosition")
  
  def alignToTargetCommand(self, getTargetDistance: Callable[[], units.meters]) -> Command:
    return self.run(
      lambda: [
        position := self._getTargetPosition(getTargetDistance()),
        self._armLeftClosedLoopController.setReference(position, SparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self._alignToIntake()
    ).withName("LauncherArmSubsystem:AlignToTarget")

  def _alignToIntake(self) -> Command:
    self._armLeftClosedLoopController.setReference(self._constants.kPositionIntake, SparkBase.ControlType.kSmartMotion),
    self.clearTargetAlignment()

  def _getTargetPosition(self, targetDistance: units.meters) -> float:
    targetPosition = utils.getInterpolatedValue(targetDistance, self._targetDistances, self._targetPositions)
    if utils.isValueInRange(targetPosition, self._constants.kMotorReverseSoftLimit, self._constants.kMotorForwardSoftLimit):
      return targetPosition
    else:
      return self._constants.kPositionSubwoofer

  def _setIsAlignedToTarget(self, position: float) -> None:
    self._isAlignedToTarget = math.fabs(self._armRightEncoder.getPosition() - position) <= self._constants.kTargetAlignmentPositionTolerance

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        utils.setSparkSoftLimitsEnabled(self._armLeftMotor, False),
        utils.setSparkSoftLimitsEnabled(self._armRightMotor, False),
        self._armLeftMotor.set(-self._constants.kResetSpeed),
        self._armRightMotor.set(-self._constants.kResetSpeed)
      ],
      lambda: [
        self._armLeftMotor.stopMotor(),
        self._armRightMotor.stopMotor(),
        self._armLeftEncoder.setPosition(0),
        self._armRightEncoder.setPosition(0),
        utils.setSparkSoftLimitsEnabled(self._armLeftMotor, True),
        utils.setSparkSoftLimitsEnabled(self._armRightMotor, True),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("LauncherArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._armLeftMotor.stopMotor()
    self._armRightMotor.stopMotor()
    self.clearTargetAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", self._armLeftEncoder.getPosition())
    SmartDashboard.putBoolean("Robot/Launcher/Arm/IsAlignedToTarget", self._isAlignedToTarget)