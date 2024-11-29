from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command, cmd
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax
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

    self._rollersMotor = SparkMax(self._constants.kRollerMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._rollersMotorConfig = SparkBaseConfig()
    (self._rollersMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.setSparkConfig(
      self._rollersMotor.configure(
        self._rollersMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._bottomMotor = SparkMax(self._constants.kBottomMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._bottomMotorConfig = SparkBaseConfig()
    (self._bottomMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._bottomMotor.configure(
        self._bottomMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._topMotor = SparkMax(self._constants.kTopMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._topMotorConfig = SparkBaseConfig()
    (self._topMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kMotorCurrentLimit)
      .inverted(False))
    utils.setSparkConfig(
      self._topMotor.configure(
        self._topMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

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
  
  def scoreAmpCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Stopped, 
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        self._constants.kSpeedScoreAmp
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:ScoreAmp")

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
    self._rollersMotor.stopMotor()
    self._topMotor.stopMotor()
    self._bottomMotor.stopMotor()
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Speed", self._topMotor.get())
    SmartDashboard.putBoolean("Robot/Intake/IsLoaded", self.isLoaded())
    SmartDashboard.putBoolean("Robot/Intake/IsLaunchReady", self.isLaunchReady())
