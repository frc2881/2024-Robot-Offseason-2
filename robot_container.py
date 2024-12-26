from commands2 import Command, cmd
from wpilib import DriverStation, SendableChooser, SmartDashboard
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from lib import logger, utils
from lib.classes import Alliance, RobotState
from lib.controllers.game_controller import GameController
from lib.controllers.lights_controller import LightsController
from lib.sensors.distance_sensor import DistanceSensor
from lib.sensors.gyro_sensor_navx2 import GyroSensor_NAVX2
from lib.sensors.object_sensor import ObjectSensor
from lib.sensors.pose_sensor import PoseSensor
from commands.auto_commands import AutoCommands
from commands.game_commands import GameCommands
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.launcher_arm_subsystem import LauncherArmSubsystem
from subsystems.launcher_rollers_subsystem import LauncherRollersSubsystem
from subsystems.localization_subsystem import LocalizationSubsystem
from classes import LightsMode
import constants

class RobotContainer:
  def __init__(self) -> None:
    self._setupSensors()
    self._setupSubsystems()
    self._setupControllers()
    self._setupCommands()
    self._setupTriggers()
    self._setupLights()
    utils.addRobotPeriodic(self._updateTelemetry)

  def _setupSensors(self) -> None:
    self.gyroSensor = GyroSensor_NAVX2(constants.Sensors.Gyro.NAVX2.kComType)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.kPoseSensorConfigs)
    SmartDashboard.putString("Robot/Sensor/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))
    self.launcherDistanceSensor = DistanceSensor(
      constants.Sensors.Distance.Launcher.kSensorName,
      constants.Sensors.Distance.Launcher.kMinTargetDistance,
      constants.Sensors.Distance.Launcher.kMaxTargetDistance
    )
    self.intakeObjectSensor = ObjectSensor(constants.Sensors.Object.Intake.kCameraName)
    
  def _setupSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(self.gyroSensor.getHeading)
    self.localizationSubsystem = LocalizationSubsystem(
      self.poseSensors,
      self.gyroSensor.getRotation,
      self.driveSubsystem.getSwerveModulePositions
    )
    AutoBuilder.configure(
      self.localizationSubsystem.getPose, 
      self.localizationSubsystem.resetPose, 
      self.driveSubsystem.getChassisSpeeds, 
      self.driveSubsystem.drive,
      PPHolonomicDriveController(
        constants.Subsystems.Drive.kPathFollowerTranslationPIDConstants,
        constants.Subsystems.Drive.kPathFollowerRotationPIDConstants
      ),
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self.driveSubsystem
    )
    self.intakeSubsystem = IntakeSubsystem(
      self.launcherDistanceSensor.hasTarget,
      self.launcherDistanceSensor.getDistance
    )
    self.launcherArmSubsystem = LauncherArmSubsystem()
    self.launcherRollersSubsystem = LauncherRollersSubsystem()

  def _setupControllers(self) -> None:
    self.driverController = GameController(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operatorController = GameController(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(True)

  def _setupCommands(self) -> None:
    self.gameCommands = GameCommands(self)
    self.autoCommands = AutoCommands(self)
    self._autoCommandChooser = SendableChooser()
    self.autoCommands.addAutoOptions(self._autoCommandChooser)
    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

  def _setupTriggers(self) -> None:
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        self.driverController.getLeftY,
        self.driverController.getLeftX,
        self.driverController.getRightX
    ))
    self.driverController.rightTrigger().and_((
        self.driverController.leftTrigger().or_(
        self.driverController.leftBumper()).or_(
        self.driverController.a()).or_(
        self.driverController.b()).or_(
        self.driverController.y())
      ).negate()
    ).whileTrue(self.gameCommands.runIntakeCommand())
    self.driverController.rightTrigger().and_(self.driverController.leftTrigger()).whileTrue(self.gameCommands.runLauncherCommand())
    self.driverController.rightTrigger().and_(self.driverController.leftBumper()).whileTrue(self.gameCommands.runLauncherCommand())
    self.driverController.rightTrigger().and_(self.driverController.a()).whileTrue(self.gameCommands.scoreAmpCommand())
    self.driverController.rightTrigger().and_(self.driverController.b()).whileTrue(self.gameCommands.runLauncherCommand())
    self.driverController.rightTrigger().and_(self.driverController.y()).whileTrue(self.gameCommands.runLauncherCommand())
    self.driverController.leftTrigger().whileTrue(self.gameCommands.alignLauncherToTargetCommand())
    self.driverController.rightBumper().whileTrue(self.gameCommands.ejectIntakeCommand())
    self.driverController.leftBumper().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionShuttle, constants.Subsystems.Launcher.Rollers.kSpeedsShuttle))
    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand())
    self.driverController.leftStick().whileTrue(self.driveSubsystem.lockCommand())
    # self.driverController.povUp().whileTrue(cmd.none())
    # self.driverController.povDown().whileTrue(cmd.none())
    # self.driverController.povLeft().whileTrue(cmd.none())
    # self.driverController.povRight().whileTrue(cmd.none())
    self.driverController.a().whileTrue(self.gameCommands.alignLauncherToAmpCommand())
    self.driverController.b().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionSubwoofer))
    self.driverController.y().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionPodium))
    self.driverController.x().whileTrue(
      self.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionClimbUp)
    ).onFalse(
      self.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionClimbDown)
    )
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())

    self.operatorController.leftY().whileTrue(
      self.launcherArmSubsystem.runCommand(
        lambda: self.operatorController.getLeftY()
    ))
    self.operatorController.rightTrigger().and_(
      self.operatorController.leftTrigger().or_(
      self.operatorController.povUp()).or_(
      self.operatorController.povDown()).or_(
      self.operatorController.povLeft()).or_(
      self.operatorController.povRight())
    ).whileTrue(self.gameCommands.runLauncherCommand())
    self.operatorController.leftTrigger().whileTrue(self.gameCommands.alignLauncherToTargetCommand())
    self.operatorController.rightBumper().and_(self.operatorController.leftBumper()).whileTrue(self.gameCommands.scoreAmpCommand())
    self.operatorController.leftBumper().whileTrue(self.gameCommands.alignLauncherToAmpCommand())
    # self.operatorController.rightStick().whileTrue(cmd.none())
    # self.operatorController.leftStick().whileTrue(cmd.none())
    self.operatorController.povUp().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionShuttle))
    self.operatorController.povDown().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionSubwoofer))
    self.operatorController.povLeft().whileTrue(self.gameCommands.alignLauncherManualCommand()) # for manual launcher targeting and tuning only
    self.operatorController.povRight().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionPodium))
    # self.operatorController.a().whileTrue(cmd.none())
    # self.operatorController.b().whileTrue(cmd.none())
    # self.operatorController.y().whileTrue(cmd.none())
    # self.operatorController.x().whileTrue(cmd.none())
    # self.operatorController.start().whileTrue(cmd.none())
    self.operatorController.back().whileTrue(self.launcherArmSubsystem.resetToZeroCommand())
    
  def _setupLights(self) -> None:
    self.lightsController = LightsController()
    utils.addRobotPeriodic(self._updateLights)

  def _updateLights(self) -> None:
    lightsMode = LightsMode.Default
    if utils.getRobotState() == RobotState.Enabled and not self._robotHasInitialZeroResets():
      lightsMode = LightsMode.RobotNotReady
    else: 
      if self.launcherDistanceSensor.hasTarget():
        lightsMode = LightsMode.IntakeNotReady
        if self.intakeSubsystem.isLoaded():
          lightsMode = LightsMode.IntakeReady
          if utils.getRobotState() == RobotState.Disabled:
            if not self.localizationSubsystem.hasVisionTargets():
              lightsMode = LightsMode.VisionNotReady
          else:
            if self.driveSubsystem.isAlignedToTarget() and self.launcherArmSubsystem.isAlignedToTarget():
              lightsMode = LightsMode.LaunchReady
    self.lightsController.setLightsMode(lightsMode)

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or self.launcherArmSubsystem.hasInitialZeroReset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())

  def getAutoCommand(self) -> Command:
    return self._autoCommandChooser.getSelected()()

  def autoInit(self) -> None:
    self.resetRobot()

  def autoExit(self) -> None: 
    self.gyroSensor.resetRobotToField(self.localizationSubsystem.getPose())

  def teleopInit(self) -> None:
    self.resetRobot()

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()
    self.intakeSubsystem.reset()
    self.launcherArmSubsystem.reset()
    self.launcherRollersSubsystem.reset()
