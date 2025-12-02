from commands2 import SequentialCommandGroup
from wpimath.geometry import Translation2d

from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.pause_command import PauseCommand
from commands.drive_specific_distance import DriveDistanceSwerveCommand


from subsystems.ledsubsystem import LEDSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vision_subsystem import VisionSystem

class DriveInSquareDemo(SequentialCommandGroup):


    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem , 
                 led : LEDSubsystem,
                 size_of_square : float) -> None:

        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
        self.size_of_square = size_of_square

        #  D <--  C
        #  |      ^
        #  V      |
        #  A -->  B

        self.addCommands(
            #  A to B Leg  
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(self.size_of_square, 0.0)),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),

            #  B to C Leg
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(0.0, self.size_of_square)),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),

            #  C to D Leg
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(-self.size_of_square, 0.0)),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),

            #  D to A Leg
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(0.0, -self.size_of_square)),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),

        )

