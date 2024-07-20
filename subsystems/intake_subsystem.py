from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command, cmd
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(
      self,
      getLauncherHasTarget: Callable[[], bool],
      getLauncherDistance: Callable[[], units.millimeters]
    ) -> None:
    super().__init__()
    self._getLauncherHasTarget = getLauncherHasTarget
    self._getLauncherDistance = getLauncherDistance

    self._constants = constants.Subsystems.Intake

    self._rollersMotor = CANSparkMax(self._constants.kRollerMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._rollersMotor.restoreFactoryDefaults())
    utils.validateParam(self._rollersMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._rollersMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._rollersMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._rollersMotor.burnFlash())

    self._topMotor = CANSparkMax(self._constants.kTopMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topMotor.restoreFactoryDefaults())
    utils.validateParam(self._topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._topMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topMotor.burnFlash())

    self._bottomMotor = CANSparkMax(self._constants.kBottomMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._bottomMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._bottomMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._bottomMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    self._bottomMotor.setInverted(True)
    utils.validateParam(self._bottomMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Reverse, 
        MotorDirection.Reverse, 
        MotorDirection.Reverse, 
        self._constants.kSpeedIntake
      )
    ).until(
      lambda: self._getLauncherHasTarget() and self._getLauncherDistance() <= self._constants.kDistanceIntake
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Run")
  
  def reloadCommand(self) -> Command:
    return self.ejectCommand().withTimeout(
      constants.Subsystems.Intake.kReloadTimeout
    ).andThen(
      self.runCommand()
    ).withName("IntakeSubsystem:Reload")

  def ejectCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        self._constants.kSpeedEject
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Eject")
  
  def alignCommand(self) -> Command:
    return cmd.run(
      lambda: self._run(
        MotorDirection.Stopped, 
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        self._constants.kSpeedAlign
      )
    ).withTimeout(
      self._constants.kAlignTimeout
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Align")

  def launchCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Stopped, 
        MotorDirection.Reverse, 
        MotorDirection.Reverse, 
        self._constants.kSpeedLaunch
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Launch")
  
  def launchAtAmpCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Stopped, 
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        self._constants.kSpeedLaunchAmp
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:LaunchAmp")

  def _run(self, rollers: MotorDirection, top: MotorDirection, bottom: MotorDirection, speed: units.percent = 0) -> None:
    self._rollersMotor.set(self._getSpeed(speed, rollers))
    self._topMotor.set(self._getSpeed(speed, top))
    self._bottomMotor.set(self._getSpeed(speed, bottom))

  def _getSpeed(self, speed: units.percent, motorDirection: MotorDirection) -> units.percent:
    if motorDirection != MotorDirection.Stopped:
      return speed * (self._constants.kMotorMaxReverseOutput if motorDirection == MotorDirection.Reverse else self._constants.kMotorMaxForwardOutput)
    else:
      return 0

  def isLoaded(self) -> bool:
    return self._getLauncherHasTarget() and self._getLauncherDistance() <= self._constants.kDistanceLauncherReadyMax

  def isLaunchReady(self) -> bool:
    return self._getLauncherHasTarget() and utils.isValueInRange(self._getLauncherDistance(), self._constants.kDistanceLauncherReadyMin, self._constants.kDistanceLauncherReadyMax)
  
  def reset(self) -> None:
    self._rollersMotor.set(0)
    self._topMotor.set(0)
    self._bottomMotor.set(0)
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Speed", self._topMotor.get())
    SmartDashboard.putBoolean("Robot/Intake/IsLoaded", self.isLoaded())
    SmartDashboard.putBoolean("Robot/Intake/IsLaunchReady", self.isLaunchReady())
