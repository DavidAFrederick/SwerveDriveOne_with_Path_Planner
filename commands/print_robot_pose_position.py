import wpilib
from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.geometry import Pose2d


class PrintRobotPose(Command):
    """ 
    This command just prints the  current robot position (Pose) for testing purposes
    This is the field-oriented  position
    """

    def __init__(self, drivetrain : CommandSwerveDrivetrain) -> None:

        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)


    def initialize(self) -> None:
        pass       #  This function is not being used.

    def execute(self) -> None:
        ### Get robot's current pose (Position and heading)

        self.robot_state = self.drivetrain.get_state()  # NOTE:  Can cause watchdog timer trips
        #  The get_state() must take time. If used too often can cause Watchdog Timer trip warnings
        #  Grab state once and use parse out the data

        self.initial_pose = Pose2d(self.robot_state.pose.x,
                                   self.robot_state.pose.y, 
                                   self.robot_state.pose.rotation().radians())
        print(f">>>     Robot Pose: {self.initial_pose}  ",end='')
        print(f"{self.robot_state.pose.rotation().degrees():6.2f} <<<<<<<<<<<<<<<<<<<<<<<<<")

        
    def isFinished(self) -> bool:
       return True             

    def end(self, interrupted: bool) -> None:
       pass       #  This function is not being used.