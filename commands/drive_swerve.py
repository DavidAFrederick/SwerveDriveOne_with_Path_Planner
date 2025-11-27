import wpilib
from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve


class DriveSwerveCommand(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        print ("Calling command")

        self.drivetrain.apply_request(
            lambda: (
                self
                ._robot_centric_drive
                .with_velocity_x(2)
                .with_velocity_y(2)
                .with_rotational_rate(0)
            ) 
        ) 

    def isFinished(self) -> bool:
       return False        ## Key command running until button released

    def end(self, interrupted: bool) -> None:
        pass



