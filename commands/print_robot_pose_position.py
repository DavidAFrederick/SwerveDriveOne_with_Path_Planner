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

        self.initial_pose = self.drivetrain.get_robot_current_pose()

        print(f">>>     Robot Pose: {self.initial_pose}  ",end='')
        print(f"{self.initial_pose.rotation().degrees():6.2f} <<<<<<<<<<<<<<<<<<<<<<<<<")
        # print(f"{self.robot_state.pose.rotation().degrees():6.2f} <<<<<<<<<<<<<<<<<<<<<<<<<")

        
    def isFinished(self) -> bool:
       return True             

    def end(self, interrupted: bool) -> None:
       pass       #  This function is not being used.