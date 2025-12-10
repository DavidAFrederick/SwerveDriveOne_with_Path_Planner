from commands2 import SequentialCommandGroup, PrintCommand
import commands2
import commands2.cmd
from wpimath.geometry import Translation2d

from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.pause_command import PauseCommand
from commands.drive_specific_distance import DriveDistanceSwerveCommand


from subsystems.ledsubsystem import LEDSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class DriveInSquareDemo(SequentialCommandGroup):


    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 led : LEDSubsystem,
                 size_of_square : float) -> None:

        super().__init__()
        self.drivetrain = drivetrain
        self.led = led
        self.size_of_square = size_of_square

        #  D <--  C
        #  |      ^
        #  V      |
        #  A -->  B

        self.addCommands(
            #  A to B Leg  
            PrintCommand("A to B Leg =========================="),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),
 
            #  B to C Leg
            PrintCommand("B to C Leg ========================="),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),

            #  C to D Leg
            PrintCommand("C to D Leg======================"),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),

            PrintCommand("#  D to A Leg"),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, 90),


            PrintCommand("#  A to B Leg  "),
            TurnHeadingSwerveCommand(self.drivetrain, 45),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, -90),
 
            PrintCommand("#  B to C Leg"),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, -90),

            PrintCommand("#  C to D Leg"),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, -90),

            PrintCommand("#  D to A Leg"),
            DriveDistanceSwerveCommand(self.drivetrain, self.size_of_square),
            PauseCommand(1.0),
            TurnHeadingSwerveCommand(self.drivetrain, -90),


        )

