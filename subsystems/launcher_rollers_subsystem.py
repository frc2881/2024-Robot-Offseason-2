from commands2 import Subsystem, Command
from wpilib import SendableChooser, SmartDashboard
from wpimath import units
from rev import CANSparkBase, CANSparkLowLevel, CANSparkFlex, SparkRelativeEncoder
from lib import utils, logger
from lib.classes import SpeedMode
from classes import LauncherRollersSpeeds
import constants

class LauncherRollersSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Launcher.Rollers

    self._bottomMotor = CANSparkFlex(self._constants.kBottomMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._bottomMotorEncoder = self._bottomMotor.getEncoder()
    self._bottomMotor.setCANMaxRetries(10)
    utils.validateParam(self._bottomMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._bottomMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._bottomMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    self._bottomMotor.setInverted(True)
    utils.validateParam(self._bottomMotor.burnFlash())

    self._topMotor = CANSparkFlex(self._constants.kTopMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._topMotorEncoder = self._topMotor.getEncoder()
    self._topMotor.setCANMaxRetries(10)
    utils.validateParam(self._topMotor.restoreFactoryDefaults())
    utils.validateParam(self._topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._topMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    self._topMotor.setInverted(False)
    utils.validateParam(self._topMotor.burnFlash())

    self._bottomMotorSpeedDelta: units.percent = 0
    self._topMotorSpeedDelta: units.percent = 0

    self._speedMode: bool = False
    speedModeChooser = SendableChooser()
    speedModeChooser.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedModeChooser.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Launcher/Rollers/SpeedMode", speedModeChooser)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, rollersSpeeds: LauncherRollersSpeeds) -> Command:
    return self.run(
      lambda: [
        self._bottomMotor.set((self._constants.kSpeedsDemo.bottom if self._speedMode == SpeedMode.Demo else rollersSpeeds.bottom) * self._constants.kMotorMaxForwardOutput),
        self._topMotor.set((self._constants.kSpeedsDemo.top if self._speedMode == SpeedMode.Demo else rollersSpeeds.top) * self._constants.kMotorMaxForwardOutput),
        self._updateMotorSpeedDeltas()
      ]
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherRollersSubsystem:Run")
  
  def _updateMotorSpeedDeltas(self) -> None:
    self._bottomMotorSpeedDelta = self._getMotorSpeedDelta(self._bottomMotor, self._bottomMotorEncoder)
    self._topMotorSpeedDelta = self._getMotorSpeedDelta(self._topMotor, self._topMotorEncoder)

  def _getMotorSpeedDelta(self, motor: CANSparkBase, encoder: SparkRelativeEncoder) -> units.percent:
    return encoder.getVelocity() / (motor.get() * self._constants.kMotorFreeSpeed) if motor.get() != 0 else 0

  def isLaunchReady(self) -> bool:
    # TODO: validate speed telemetry for launch ready timeout
    return self._bottomMotorSpeedDelta >= self._constants.kLaunchSpeedDeltaMin and self._topMotorSpeedDelta >= self._constants.kLaunchSpeedDeltaMin

  def reset(self) -> None:
    self._bottomMotor.set(0)
    self._topMotor.set(0)
    self._bottomMotorSpeedDelta = 0
    self._topMotorSpeedDelta = 0

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Bottom/Speed/Delta", self._bottomMotorSpeedDelta)
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Top/Speed/Delta", self._topMotorSpeedDelta)
    SmartDashboard.putBoolean("Robot/Launcher/Rollers/IsLaunchReady", self.isLaunchReady())
