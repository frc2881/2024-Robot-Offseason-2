from typing import Callable
import math
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from lib import logger, utils
from lib.components.leadscrew_module import LeadscrewModule
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

    self._leadscrewModuleLeft = LeadscrewModule(self._constants.kLeadScrewModuleConfigLeft)
    self._leadscrewModuleRight = LeadscrewModule(self._constants.kLeadScrewModuleConfigRight)

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:Run")

  def alignToPositionCommand(self, position: float) -> Command:
    return self.run(
      lambda: [
        self._setPosition(position),
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
        self._setPosition(position),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self._alignToIntake()
    ).withName("LauncherArmSubsystem:AlignToTarget")

  def _setSpeed(self, speed: units.percent) -> None:
    self._leadscrewModuleLeft.setSpeed(speed),

  def _setPosition(self, position: float) -> None:
    self._leadscrewModuleLeft.setPosition(position)

  def _getPosition(self) -> float:
    return self._leadscrewModuleLeft.getPosition()

  def _alignToIntake(self) -> Command:
    self._setPosition(self._constants.kPositionIntake)
    self.clearTargetAlignment()

  def _getTargetPosition(self, targetDistance: units.meters) -> float:
    return utils.getInterpolatedValue(targetDistance, self._targetDistances, self._targetPositions)

  def _setIsAlignedToTarget(self, position: float) -> None:
    self._isAlignedToTarget = math.fabs(self._getPosition() - position) <= self._constants.kTargetAlignmentPositionTolerance

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._leadscrewModuleLeft.startZeroReset()
      ],
      lambda: [
        self._leadscrewModuleLeft.endZeroReset(),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("LauncherArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._leadscrewModuleLeft.reset()
    self.clearTargetAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", self._getPosition())
    SmartDashboard.putBoolean("Robot/Launcher/Arm/IsAlignedToTarget", self._isAlignedToTarget)