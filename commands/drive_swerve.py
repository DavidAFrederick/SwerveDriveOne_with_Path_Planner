import wpilib
from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from phoenix6.swerve.requests import FieldCentric, RobotCentric


class DriveSwerveCommand(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:

        self.swerve_request = (RobotCentric()
                               .with_velocity_x(2)
                               .with_velocity_y(2))
        
        self.drivetrain.set_control(self.swerve_request)

    def isFinished(self) -> bool:
       return False        

    def end(self, interrupted: bool) -> None:
        pass



