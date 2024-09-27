from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units

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
