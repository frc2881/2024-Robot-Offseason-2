from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance
if TYPE_CHECKING: from robot_container import RobotContainer
import constants

class AutoPath(Enum):
  Pickup1 = auto()
  Pickup12 = auto()
  Pickup2 = auto()
  Pickup21 = auto()
  Pickup23 = auto()
  Pickup3 = auto()
  Pickup32 = auto()
  Pickup4 = auto()
  Pickup41 = auto()
  Pickup5 = auto()
  Pickup61 = auto()
  Pickup62 = auto()
  Pickup63 = auto()
  Pickup72 = auto()
  Pickup73 = auto()
  Pickup8 = auto()
  Score1 = auto()
  Score2 = auto()
  Score3 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

    AutoBuilder.configure(
      self._robot.localizationSubsystem.getPose, 
      self._robot.localizationSubsystem.resetPose, 
      self._robot.driveSubsystem.getChassisSpeeds, 
      self._robot.driveSubsystem.drive, 
      constants.Subsystems.Drive.kPathPlannerController,
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.driveSubsystem
    )

    self._autoCommandChooser = SendableChooser()
    self._autoCommandChooser.setDefaultOption("None", cmd.none)

    self._autoCommandChooser.addOption("[0]", self.auto_0)

    self._autoCommandChooser.addOption("[1] 0_1", self.auto_1_0_1)
    self._autoCommandChooser.addOption("[1] 0_1_2_3", self.auto_1_0_1_2_3)
    self._autoCommandChooser.addOption("[1] 0_1_2_3_83", self.auto_1_0_1_2_3_83)
    self._autoCommandChooser.addOption("[1] 0_1_2_62", self.auto_1_0_1_2_62)
    self._autoCommandChooser.addOption("[1] 0_1_41", self.auto_1_0_1_41)
    self._autoCommandChooser.addOption("[1] 0_1_41_51", self.auto_1_0_1_41_51)
    self._autoCommandChooser.addOption("[1] 0_1_51", self.auto_1_0_1_51)
    self._autoCommandChooser.addOption("[1] 0_1_51_41", self.auto_1_0_1_51_41)
    self._autoCommandChooser.addOption("[1] 0_1_51_61", self.auto_1_0_1_51_61)
    self._autoCommandChooser.addOption("[1] 0_1_51_62", self.auto_1_0_1_51_62)
    self._autoCommandChooser.addOption("[1] 0_51_62_72", self.auto_1_0_51_62_72)

    self._autoCommandChooser.addOption("[2] 0_2", self.auto_2_0_2) 
    self._autoCommandChooser.addOption("[2] 0_2_62", self.auto_2_0_2_62) 
    self._autoCommandChooser.addOption("[2] 0_2_62_51", self.auto_2_0_2_62_51) 
    self._autoCommandChooser.addOption("[2] 0_2_62_72", self.auto_2_0_2_62_72)
    self._autoCommandChooser.addOption("[2] 0_2_72", self.auto_2_0_2_72)
    self._autoCommandChooser.addOption("[2] 0_2_72_62", self.auto_2_0_2_72_62) 
    self._autoCommandChooser.addOption("[2] 0_2_1", self.auto_2_0_2_1) 
    self._autoCommandChooser.addOption("[2] 0_2_3", self.auto_2_0_2_3) 
    self._autoCommandChooser.addOption("[2] 0_2_3_62", self.auto_2_0_2_3_62) 

    self._autoCommandChooser.addOption("[3] 0_3", self.auto_3_0_3)
    self._autoCommandChooser.addOption("[3] 0_3_2_1", self.auto_3_0_3_2_1)
    self._autoCommandChooser.addOption("[3] 0_3_2_1_41", self.auto_3_0_3_2_1_41) 
    self._autoCommandChooser.addOption("[3] 0_3_2_1_51", self.auto_3_0_3_2_1_51) 
    self._autoCommandChooser.addOption("[3] 0_3_2_62", self.auto_3_0_3_2_62) 
    self._autoCommandChooser.addOption("[3] 0_3_62", self.auto_3_0_3_62) 
    self._autoCommandChooser.addOption("[3] 0_3_62_72", self.auto_3_0_3_62_72) 
    self._autoCommandChooser.addOption("[3] 0_3_72", self.auto_3_0_3_72) 
    self._autoCommandChooser.addOption("[3] 0_3_72_62", self.auto_3_0_3_72_62) 
    self._autoCommandChooser.addOption("[3] 0_3_73", self.auto_3_0_3_73)
    self._autoCommandChooser.addOption("[3] 0_3_73_83", self.auto_3_0_3_73_83)
    self._autoCommandChooser.addOption("[3] 0_3_82", self.auto_3_0_3_82) 
    self._autoCommandChooser.addOption("[3] 0_3_83", self.auto_3_0_3_83) 
    self._autoCommandChooser.addOption("[3] 0_3_82_62", self.auto_3_0_3_82_62) 
    self._autoCommandChooser.addOption("[3] 0_3_82_72", self.auto_3_0_3_82_72) 
    self._autoCommandChooser.addOption("[3] 0_3_83_62", self.auto_3_0_3_83_62) 
    self._autoCommandChooser.addOption("[3] 0_3_83_72", self.auto_3_0_3_83_72) 
    self._autoCommandChooser.addOption("[3] 0_3_83_73", self.auto_3_0_3_83_73)
    self._autoCommandChooser.addOption("[3] 0_73", self.auto_3_0_73)
    self._autoCommandChooser.addOption("[3] 0_83", self.auto_3_0_83)
    self._autoCommandChooser.addOption("[3] 0_73_83", self.auto_3_0_73_83)
    self._autoCommandChooser.addOption("[3] 0_83_73", self.auto_3_0_83_73)
    self._autoCommandChooser.addOption("[3] 0_83_72_62", self.auto_3_0_83_72_62)  

    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

  def getSelected(self) -> Command:
    return self._autoCommandChooser.getSelected()()

  def _move(self, path: AutoPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      self._paths.get(path), 
      constants.Subsystems.Drive.kPathFindingConstraints
    ).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )
  
  def _moveToScore(self, path: AutoPath) -> Command:
    return self._move(path).onlyIf(self._robot.intakeSubsystem.isLoaded)
  
  def _pickup(self, path: AutoPath) -> Command:
    return cmd.deadline(
      self._robot.gameCommands.runIntakeCommand(),
      self._move(path)
    ).withTimeout(constants.Game.Commands.kAutoPickupTimeout)
  
  def _score(self) -> Command:
    return cmd.deadline(
      self._robot.gameCommands.launchToTargetCommand(),
      self._robot.gameCommands.alignRobotToTargetCommand().withTimeout(constants.Game.Commands.kScoringAlignmentTimeout)
    )
  
  def _scorePreload(self) -> Command:
    return cmd.sequence(
      self._robot.gameCommands.launchAtPositionCommand(constants.Subsystems.Launcher.Arm.kPositionSubwoofer)
    )

  def auto_0(self) -> Command:
    return cmd.sequence(
      self._scorePreload()
    ).withName("AutoCommands:[0]")
  
  def auto_1_0_1(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score()
    ).withName("AutoCommands:[1]_0_1")
   
  def auto_1_0_1_2_3(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup21),
      self._score(),
      self._pickup(AutoPath.Pickup32),
      self._score()
    ).withName("AutoCommands:[1]_0_1_2_3")
    
  def auto_1_0_1_2_3_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup21),
      self._score(),
      self._pickup(AutoPath.Pickup32),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[1]_0_1_2_3_83")
    
  def auto_1_0_1_2_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup21),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[1]_0_1_2_62")
    
  def auto_1_0_1_41(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup4),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[1]_0_1_41")
   
  def auto_1_0_1_41_51(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup4),
      self._moveToScore(AutoPath.Score1),
      self._score(),
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[1]_0_1_41_51")  
   
  def auto_1_0_1_51(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[1]_0_1_51")
   
  def auto_1_0_1_51_41(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score(),
      self._pickup(AutoPath.Pickup4),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[1]_0_1_51_41")
   
  def auto_1_0_1_51_61(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score(),
      self._pickup(AutoPath.Pickup61),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[1]_0_1_51_61")
   
  def auto_1_0_1_51_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup1),
      self._score(), 
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[1]_0_1_51_62")
  
  def auto_1_0_51_62_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup72),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[1]_0_51_62_72")
   
  def auto_2_0_2(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score()
    ).withName("AutoCommands:[2]_0_2")
      
  def auto_2_0_2_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[2]_0_2_62")
   
  def auto_2_0_2_62_51(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[2]_0_2_62_51")
   
  def auto_2_0_2_62_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup72),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[2]_0_2_62_72")
   
  def auto_2_0_2_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup72),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[2]_0_2_72")
   
  def auto_2_0_2_72_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup72),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[2]_0_2_72_62")
  
  def auto_2_0_2_1(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup12),
      self._score()
    ).withName("AutoCommands:[2]_0_2_1")
   
  def auto_2_0_2_3(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup32),
      self._score()
    ).withName("AutoCommands:[2]_0_2_3")
   
  def auto_2_0_2_3_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup2),
      self._score(), 
      self._pickup(AutoPath.Pickup32),
      self._score(),
      self._pickup(AutoPath.Pickup63),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[2]_0_2_3_62")
   
  def auto_3_0_3(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score()
    ).withName("AutoCommands:[3]_0_3")

  def auto_3_0_73(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_73")
   
  def auto_3_0_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_83")
  
  def auto_3_0_73_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_73_83")
   
  def auto_3_0_83_73(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_83_73")
  
  def auto_3_0_3_2_1(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(), 
      self._pickup(AutoPath.Pickup23),
      self._score(),
      self._pickup(AutoPath.Pickup12),
      self._score()
    ).withName("AutoCommands:[3]_0_3_2_1")
   
  def auto_3_0_3_2_1_41(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(), 
      self._pickup(AutoPath.Pickup23),
      self._score(),
      self._pickup(AutoPath.Pickup12),
      self._score(),
      self._pickup(AutoPath.Pickup4),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[3]_0_3_2_1_41")
   
  def auto_3_0_3_2_1_51(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(), 
      self._pickup(AutoPath.Pickup23),
      self._score(),
      self._pickup(AutoPath.Pickup12),
      self._score(),
      self._pickup(AutoPath.Pickup5),
      self._moveToScore(AutoPath.Score1),
      self._score()
    ).withName("AutoCommands:[3]_0_3_2_1_51")
   
  def auto_3_0_3_2_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(), 
      self._pickup(AutoPath.Pickup23),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[1]_0_1_2_3")

  def auto_3_0_3_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup63),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_62")
  
  def auto_3_0_3_62_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup63),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup72),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_62_72")
  
  def auto_3_0_3_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_72")
  
  def auto_3_0_3_72_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_72_62")
  
  def auto_3_0_3_73(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_3_73")
  
  def auto_3_0_3_73_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_3_73_83")
  
  def auto_3_0_3_82(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_82")
   
  def auto_3_0_3_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_3_83")
   
  def auto_3_0_3_82_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_82_62")
  
  def auto_3_0_3_82_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup72),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_82_72")
  
  def auto_3_0_3_83_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_83_62")
  
  def auto_3_0_3_83_72(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_3_83_72")
  
  def auto_3_0_3_83_73(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup3),
      self._score(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score3),
      self._score()
    ).withName("AutoCommands:[3]_0_3_83_73")
  
  def auto_3_0_83_72_62(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(AutoPath.Pickup8),
      self._moveToScore(AutoPath.Score3),
      self._score(),
      self._pickup(AutoPath.Pickup73),
      self._moveToScore(AutoPath.Score2),
      self._score(),
      self._pickup(AutoPath.Pickup62),
      self._moveToScore(AutoPath.Score2),
      self._score()
    ).withName("AutoCommands:[3]_0_83_72_62")
  