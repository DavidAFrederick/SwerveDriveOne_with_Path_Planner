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
        self.speed = 0.5
        self.seconds = seconds
        self.addRequirements(drivetrain)

        self.counter_for_periodic_printing = 0


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def initialize(self) -> None:
        self.timer = wpilib.Timer()
        self.timer.start()
   
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    def execute(self):
        self.drivetrain.driving_forward(self.speed)


                    # This code causes the output to be printed twice a second
        self.counter_for_periodic_printing = self.counter_for_periodic_printing + 1
        if (self.counter_for_periodic_printing % 10 == 0): 
            self.counter_for_periodic_printing = 0
            print(f">>>  self.speed: {self.speed:5.2f}   ")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def isFinished(self) -> bool:
        if self.timer.get() < self.seconds:
            return False
        else:
            return True
    def end(self, interrupted: bool):
        self.drivetrain.driving_forward(0.0)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
