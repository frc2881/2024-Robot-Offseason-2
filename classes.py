from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units

class LightsMode(Enum):
  Default = auto()
  RobotNotReady = auto()
  VisionNotReady = auto()
  IntakeReady = auto()
  IntakeNotReady = auto()
  LaunchReady = auto()

@dataclass(frozen=True)
class LauncherArmPositionTarget:
  distance: units.meters
  position: float

@dataclass(frozen=True)
class LauncherRollersSpeeds:
  top: units.percent
  bottom: units.percent
