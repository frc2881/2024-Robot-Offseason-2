from typing import TYPE_CHECKING
from wpilib import RobotBase
from commands2 import Command, cmd
if TYPE_CHECKING: from robot_container import RobotContainer
from lib import utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
import constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

  def runIntakeCommand(self) -> Command:
    return cmd.sequence(
      cmd.either(
        self._robot.intakeSubsystem.runCommand().deadlineFor(
          self._robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionIntake)
        ),
        self._robot.intakeSubsystem.reloadCommand(),
        lambda: not self._robot.launcherDistanceSensor.hasTarget()
      ),
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:RunIntake")

  def ejectIntakeCommand(self) -> Command:
    return cmd.sequence(
      self._robot.intakeSubsystem.ejectCommand()
    ).withName("GameCommands:EjectIntake")
  
  def alignRobotToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._robot.driveSubsystem.alignToTargetCommand(
          lambda: self._robot.localizationSubsystem.getPose(), 
          lambda: self._robot.localizationSubsystem.getTargetHeading()
        ),
        self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short),
        cmd.sequence(
          cmd.waitUntil(lambda: self._robot.driveSubsystem.isAlignedToTarget()),
          self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
        )
      )
    ).withName("GameCommands:AlignRobotToTarget")

  def alignLauncherToTargetCommand(self) -> Command:
    return cmd.sequence(
      self._robot.intakeSubsystem.alignCommand(),
      cmd.parallel(
        self._robot.launcherArmSubsystem.alignToTargetCommand(lambda: self._robot.localizationSubsystem.getTargetDistance()),
        self._robot.launcherRollersSubsystem.runCommand(constants.Subsystems.Launcher.Rollers.kSpeedsDefault)
      )
    ).withName("GameCommands:AlignLauncherToTarget")

  def alignLauncherToPositionCommand(self, position: float, launcherRollerSpeeds = constants.Subsystems.Launcher.Rollers.kSpeedsDefault) -> Command:
    return cmd.sequence(
      self._robot.intakeSubsystem.alignCommand(),
      cmd.parallel(
        self._robot.launcherArmSubsystem.alignToPositionCommand(position),
        self._robot.launcherRollersSubsystem.runCommand(launcherRollerSpeeds)
      )
    ).withName("GameCommands:AlignLauncherToPosition")

  def alignLauncherToAmpCommand(self) -> Command:
    return cmd.sequence(
      self._robot.intakeSubsystem.alignCommand(),
      self._robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionAmp)
    ).withName("GameCommands:AlignLauncherToAmp")
  
  def alignLauncherManualCommand(self, launcherRollerSpeeds = constants.Subsystems.Launcher.Rollers.kSpeedsDefault) -> Command:
    return cmd.sequence(
      self._robot.intakeSubsystem.alignCommand(),
      cmd.parallel(
        self._robot.launcherRollersSubsystem.runCommand(launcherRollerSpeeds)
      )
    ).withName("GameCommands:AlignLauncherManual")
  
  def runLauncherCommand(self) -> Command:
    return cmd.sequence(
      cmd.waitUntil(lambda: self._robot.launcherRollersSubsystem.isLaunchReady()).withTimeout(constants.Game.Commands.kScoringLaunchTimeout),
      self._robot.intakeSubsystem.launchCommand() 
    ).onlyIf(
      lambda: self._robot.intakeSubsystem.isLaunchReady()
    ).until(
      lambda: not self._robot.intakeSubsystem.isLoaded()
    ).finallyDo(lambda end: [
      self._robot.driveSubsystem.clearTargetAlignment(),
      self._robot.launcherArmSubsystem.clearTargetAlignment()
    ]).withName("GameCommands:RunLauncher")

  def launchToTargetCommand(self) -> Command:
    return cmd.deadline(
      cmd.sequence(
        cmd.waitSeconds(constants.Game.Commands.kScoringAlignmentTimeout),
        self.runLauncherCommand()
      ),
      self.alignLauncherToTargetCommand()
    ).withName("GameCommands:LaunchToTarget")
  
  def launchAtPositionCommand(self, position: float, launcherRollerSpeeds = constants.Subsystems.Launcher.Rollers.kSpeedsDefault) -> Command:
    return cmd.deadline(
      cmd.sequence(
        cmd.waitSeconds(constants.Game.Commands.kScoringAlignmentTimeout),
        self.runLauncherCommand()
      ),
      self.alignLauncherToPositionCommand(position, launcherRollerSpeeds)
    ).withName("GameCommands:LaunchAtPosition")
  
  def scoreAmpCommand(self) -> Command:
    return cmd.sequence(
      self._robot.intakeSubsystem.scoreAmpCommand()
    ).withName("GameCommands:ScoreAmp")
  
  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self._robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self._robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")