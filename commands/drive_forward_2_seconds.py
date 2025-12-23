import wpilib
from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class Drive_Forward_X_Seconds(Command):
    """
    Drives the robot forward in a straight line for X seconds. 
    Robot Centric movement
    """
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def __init__(self, drivetrain : CommandSwerveDrivetrain, seconds : float) -> None:
        self.drivetrain = drivetrain
        self.speed = 0.95
        self.seconds = seconds
        self.addRequirements(drivetrain)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def initialize(self) -> None:
        self.timer = wpilib.Timer()
        self.timer.start()
   
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def execute(self):
        self.drivetrain.driving_forward(self.speed)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def isFinished(self) -> bool:
        if self.timer.get() < self.seconds:
            return False
        else:
            return True
    def end(self, interrupted: bool):
        self.drivetrain.driving_forward(0.0)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
