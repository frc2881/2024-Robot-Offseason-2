from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(
      self,
      getLauncherTargetDistance: Callable[[], units.millimeters]
    ) -> None:
    super().__init__()
    self._getLauncherTargetDistance = getLauncherTargetDistance

    self._constants = constants.Subsystems.Intake

    self._rollersMotor = CANSparkMax(self._constants.kRollerMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._rollersMotor.restoreFactoryDefaults())
    utils.validateParam(self._rollersMotor.setIdleMode(self._constants.kRollerMotorIdleMode))
    utils.validateParam(self._rollersMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._rollersMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._rollersMotor.burnFlash())

    self._topMotor = CANSparkMax(self._constants.kTopMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topMotor.restoreFactoryDefaults())
    utils.validateParam(self._topMotor.setIdleMode(self._constants.kTopMotorIdleMode))
    utils.validateParam(self._topMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topMotor.burnFlash())

    self._bottomMotor = CANSparkMax(self._constants.kBottomMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._bottomMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomMotor.setIdleMode(self._constants.kBottomMotorIdleMode))
    utils.validateParam(self._bottomMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._bottomMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._bottomMotor.setInverted(True))
    utils.validateParam(self._bottomMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self) -> Command:
    return self.run(
      lambda: self._run(MotorDirection.Reverse, MotorDirection.Reverse, MotorDirection.Reverse, self._constants.kSpeedIntake)
    ).until(
      lambda: False # self._getLauncherTargetDistance() <= self._constants.kIntakeTriggerDistanceRear
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Run")
  
  def ejectCommand(self) -> Command: # TODO: Test which direction eject should go
    return self.run(
      lambda: self._run(MotorDirection.Forward, MotorDirection.Forward, MotorDirection.Forward, self._constants.kSpeedEject)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Eject")
  
  def alignCommand(self) -> Command:
    return self.run(
      lambda: self._run(MotorDirection.Stop, MotorDirection.Reverse, MotorDirection.Reverse, self._constants.kSpeedAlign)
    ).until(
      lambda: self._getLauncherTargetDistance() >= self._constants.kLauncherTriggerDistanceAlign
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Align")

  def launchCommand(self) -> Command:
    return self.run(
      lambda: self._run(MotorDirection.Stop, MotorDirection.Forward, MotorDirection.Forward, self._constants.kSpeedLaunch)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Launch")
  
  def launchAmpCommand(self) -> Command:
    return self.run(
      lambda: self._run(MotorDirection.Stop, MotorDirection.Reverse, MotorDirection.Reverse, self._constants.kSpeedLaunch)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Launch")

  def _run(self, rollers: MotorDirection, top: MotorDirection, bottom: MotorDirection, speed: units.percent = 0) -> None:
    self._rollersMotor.set(self._getSpeed(speed, rollers))
    self._topMotor.set(self._getSpeed(speed, top))
    self._bottomMotor.set(self._getSpeed(speed, bottom))

  def _getSpeed(self, speed: units.percent, motorDirection: MotorDirection) -> units.percent:
    if(motorDirection == MotorDirection.Stop):
      return 0
    else:
      return speed * (self._constants.kMotorMaxReverseOutput if motorDirection == MotorDirection.Reverse else self._constants.kMotorMaxForwardOutput)

  def isLaunchReady(self) -> bool:
    return utils.isValueInRange(self._getLauncherTargetDistance(), self._constants.kLauncherReadyDistanceMin, self._constants.kLauncherReadyDistanceMax)
  
  def reset(self) -> None:
    self._rollersMotor.set(0)
    self._topMotor.set(0)
    self._bottomMotor.set(0)
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Belts/Speed", self._rollersMotor.get())
    SmartDashboard.putNumber("Robot/Intake/IsLaunchReady", self.isLaunchReady())
