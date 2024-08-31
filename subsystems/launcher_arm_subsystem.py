from typing import Callable
import math
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import CANSparkBase, CANSparkLowLevel, CANSparkFlex
from lib import utils, logger
import constants

class LauncherArmSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Launcher.Arm

    self._hasInitialZeroReset: bool = False
    self._isAlignedToTarget: bool = False
    self._targetDistances = list(map(lambda p: p.distance, self._constants.kPositionTargets))
    self._targetPositions = list(map(lambda p: p.position, self._constants.kPositionTargets))
    
    SmartDashboard.putString("Robot/Launcher/Arm/Positions", utils.toJson(self._constants.kPositionTargets))

    self._armRightMotor = CANSparkFlex(self._constants.kRightMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armRightEncoder = self._armRightMotor.getEncoder()
    self._armRightPIDController = self._armRightMotor.getPIDController()
    utils.validateParam(self._armRightMotor.restoreFactoryDefaults())
    utils.validateParam(self._armRightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._armRightMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._armRightMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._armRightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armRightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.kMotorForwardSoftLimit))
    utils.validateParam(self._armRightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armRightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.kMotorReverseSoftLimit))
    utils.validateParam(self._armRightEncoder.setPositionConversionFactor(self._constants.kMotorPositionConversionFactor))
    utils.validateParam(self._armRightEncoder.setVelocityConversionFactor(self._constants.kMotorVelocityConversionFactor))
    utils.validateParam(self._armRightPIDController.setFeedbackDevice(self._armRightEncoder))
    utils.validateParam(self._armRightPIDController.setP(self._constants.kMotorPIDConstants.P))
    utils.validateParam(self._armRightPIDController.setI(self._constants.kMotorPIDConstants.I))
    utils.validateParam(self._armRightPIDController.setD(self._constants.kMotorPIDConstants.D))
    utils.validateParam(self._armRightPIDController.setFF(self._constants.kMotorPIDConstants.FF))
    utils.validateParam(self._armRightPIDController.setOutputRange(self._constants.kMotorMaxReverseOutput, self._constants.kMotorMaxForwardOutput))
    utils.validateParam(self._armRightPIDController.setSmartMotionMaxVelocity(self._constants.kMotorSmartMotionMaxVelocity))
    utils.validateParam(self._armRightPIDController.setSmartMotionMaxAccel(self._constants.kMotorSmartMotionMaxAccel))
    utils.validateParam(self._armRightMotor.burnFlash())

    self._armLeftMotor = CANSparkFlex(self._constants.kLeftMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armLeftEncoder = self._armLeftMotor.getEncoder()
    self._armLeftPIDController = self._armLeftMotor.getPIDController()
    utils.validateParam(self._armLeftMotor.restoreFactoryDefaults())
    utils.validateParam(self._armLeftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._armLeftMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._armLeftMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._armLeftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armLeftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.kMotorForwardSoftLimit))
    utils.validateParam(self._armLeftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armLeftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.kMotorReverseSoftLimit))
    utils.validateParam(self._armLeftEncoder.setPositionConversionFactor(self._constants.kMotorPositionConversionFactor))
    utils.validateParam(self._armLeftEncoder.setVelocityConversionFactor(self._constants.kMotorVelocityConversionFactor))
    utils.validateParam(self._armLeftPIDController.setFeedbackDevice(self._armLeftEncoder))
    utils.validateParam(self._armLeftPIDController.setP(self._constants.kMotorPIDConstants.P))
    utils.validateParam(self._armLeftPIDController.setI(self._constants.kMotorPIDConstants.I))
    utils.validateParam(self._armLeftPIDController.setD(self._constants.kMotorPIDConstants.D))
    utils.validateParam(self._armLeftPIDController.setFF(self._constants.kMotorPIDConstants.FF))
    utils.validateParam(self._armLeftPIDController.setOutputRange(self._constants.kMotorMaxReverseOutput, self._constants.kMotorMaxForwardOutput))
    utils.validateParam(self._armLeftPIDController.setSmartMotionMaxVelocity(self._constants.kMotorSmartMotionMaxVelocity))
    utils.validateParam(self._armLeftPIDController.setSmartMotionMaxAccel(self._constants.kMotorSmartMotionMaxAccel))
    utils.validateParam(self._armLeftMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: [
        self._armRightMotor.set(getInput() * self._constants.kInputLimit),
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
        self._armRightPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._armLeftPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
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
        self._armRightPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._armLeftPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self._alignToIntake()
    ).withName("LauncherArmSubsystem:AlignToTarget")

  def _alignToIntake(self) -> Command:
    self._armRightPIDController.setReference(self._constants.kPositionIntake, CANSparkBase.ControlType.kSmartMotion),
    self._armLeftPIDController.setReference(self._constants.kPositionIntake, CANSparkBase.ControlType.kSmartMotion),
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
        utils.enableSoftLimits(self._armRightMotor, False),
        utils.enableSoftLimits(self._armLeftMotor, False),
        self._armRightMotor.set(-self._constants.kResetSpeed),
        self._armLeftMotor.set(-self._constants.kResetSpeed)
      ],
      lambda: [
        self._armRightEncoder.setPosition(0),
        self._armLeftEncoder.setPosition(0),
        self._armRightMotor.set(0),
        self._armLeftMotor.set(0),
        utils.enableSoftLimits(self._armRightMotor, True),
        utils.enableSoftLimits(self._armLeftMotor, True),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("LauncherArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._armRightMotor.set(0)
    self._armLeftMotor.set(0)
    self.clearTargetAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/RightArm/Position", self._armRightEncoder.getPosition())
    SmartDashboard.putNumber("Robot/Launcher/LeftArm/Position", self._armLeftEncoder.getPosition())
    SmartDashboard.putBoolean("Robot/Launcher/Arm/IsAlignedToTarget", self._isAlignedToTarget)