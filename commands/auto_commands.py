from typing import TYPE_CHECKING
from commands2 import Command, cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
if TYPE_CHECKING: from robot_container import RobotContainer
from classes import AutoPath
import constants

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

    self._paths: dict[AutoPath, PathPlannerPath] = {}
    for path in AutoPath:
      self._paths[path] = PathPlannerPath.fromPathFile(path.name)

    self._addAutoOptions()

  def _move(self, path: AutoPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(self._paths.get(path), constants.Subsystems.Drive.kPathFindingConstraints)
  
  def _moveToScore(self, path: AutoPath) -> Command:
    return self._move(path).onlyIf(lambda: self._robot.intakeSubsystem.isLoaded())
  
  def _pickup(self, path: AutoPath) -> Command:
    return cmd.deadline(
      self._robot.gameCommands.runIntakeCommand(),
      self._move(self._paths.get(path))
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

  def _addAutoOptions(self) -> None:
    self._robot.addAutoOption("[0]", self.auto_0)

    self._robot.addAutoOption("[1] 0_1", self.auto_1_0_1)
    self._robot.addAutoOption("[1] 0_1_2_3", self.auto_1_0_1_2_3)
    self._robot.addAutoOption("[1] 0_1_2_3_83", self.auto_1_0_1_2_3_83)
    self._robot.addAutoOption("[1] 0_1_2_62", self.auto_1_0_1_2_62)
    self._robot.addAutoOption("[1] 0_1_41", self.auto_1_0_1_41)
    self._robot.addAutoOption("[1] 0_1_41_51", self.auto_1_0_1_41_51)
    self._robot.addAutoOption("[1] 0_1_51", self.auto_1_0_1_51)
    self._robot.addAutoOption("[1] 0_1_51_41", self.auto_1_0_1_51_41)
    self._robot.addAutoOption("[1] 0_1_51_61", self.auto_1_0_1_51_61)
    self._robot.addAutoOption("[1] 0_1_51_62", self.auto_1_0_1_51_62)
    self._robot.addAutoOption("[1] 0_51_62_72", self.auto_1_0_51_62_72)

    self._robot.addAutoOption("[2] 0_2", self.auto_2_0_2) 
    self._robot.addAutoOption("[2] 0_2_62", self.auto_2_0_2_62) 
    self._robot.addAutoOption("[2] 0_2_62_51", self.auto_2_0_2_62_51) 
    self._robot.addAutoOption("[2] 0_2_62_72", self.auto_2_0_2_62_72)
    self._robot.addAutoOption("[2] 0_2_72", self.auto_2_0_2_72)
    self._robot.addAutoOption("[2] 0_2_72_62", self.auto_2_0_2_72_62) 
    self._robot.addAutoOption("[2] 0_2_1", self.auto_2_0_2_1) 
    self._robot.addAutoOption("[2] 0_2_3", self.auto_2_0_2_3) 
    self._robot.addAutoOption("[2] 0_2_3_62", self.auto_2_0_2_3_62) 

    self._robot.addAutoOption("[3] 0_3", self.auto_3_0_3)
    self._robot.addAutoOption("[3] 0_3_2_1", self.auto_3_0_3_2_1)
    self._robot.addAutoOption("[3] 0_3_2_1_41", self.auto_3_0_3_2_1_41) 
    self._robot.addAutoOption("[3] 0_3_2_1_51", self.auto_3_0_3_2_1_51) 
    self._robot.addAutoOption("[3] 0_3_2_62", self.auto_3_0_3_2_62) 
    self._robot.addAutoOption("[3] 0_3_62", self.auto_3_0_3_62) 
    self._robot.addAutoOption("[3] 0_3_62_72", self.auto_3_0_3_62_72) 
    self._robot.addAutoOption("[3] 0_3_72", self.auto_3_0_3_72) 
    self._robot.addAutoOption("[3] 0_3_72_62", self.auto_3_0_3_72_62) 
    self._robot.addAutoOption("[3] 0_3_73", self.auto_3_0_3_73)
    self._robot.addAutoOption("[3] 0_3_73_83", self.auto_3_0_3_73_83)
    self._robot.addAutoOption("[3] 0_3_82", self.auto_3_0_3_82) 
    self._robot.addAutoOption("[3] 0_3_83", self.auto_3_0_3_83) 
    self._robot.addAutoOption("[3] 0_3_82_62", self.auto_3_0_3_82_62) 
    self._robot.addAutoOption("[3] 0_3_82_72", self.auto_3_0_3_82_72) 
    self._robot.addAutoOption("[3] 0_3_83_62", self.auto_3_0_3_83_62) 
    self._robot.addAutoOption("[3] 0_3_83_72", self.auto_3_0_3_83_72) 
    self._robot.addAutoOption("[3] 0_3_83_73", self.auto_3_0_3_83_73)
    self._robot.addAutoOption("[3] 0_73", self.auto_3_0_73)
    self._robot.addAutoOption("[3] 0_83", self.auto_3_0_83)
    self._robot.addAutoOption("[3] 0_73_83", self.auto_3_0_73_83)
    self._robot.addAutoOption("[3] 0_83_73", self.auto_3_0_83_73)
    self._robot.addAutoOption("[3] 0_83_72_62", self.auto_3_0_83_72_62)   

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
  