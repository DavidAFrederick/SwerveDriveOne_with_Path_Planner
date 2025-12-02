from commands2 import SequentialCommandGroup
from wpimath.geometry import Translation2d

from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.pause_command import PauseCommand
from commands.drive_specific_distance import DriveDistanceSwerveCommand


from subsystems.ledsubsystem import LEDSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vision_subsystem import VisionSystem

from apriltagalignmentdata import AprilTagAlignmentData



class AprilTagAlignGroupCommand(SequentialCommandGroup):


    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem , 
                 led : LEDSubsystem,
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:

        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
        self.apriltag_alignment_data = apriltag_alignment_data

        # self.heading_change_degrees = 45.0
        # self.movement_translation = Translation2d(2.0, 0.0)

        self.addCommands(
            PauseCommand(3.0),
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(2.0, 0.0)),
            PauseCommand(3.0),

            TurnHeadingSwerveCommand(self.drivetrain, 90),
            PauseCommand(3.0),
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(0.0, 2.0)),
            PauseCommand(3.0),


            TurnHeadingSwerveCommand(self.drivetrain, 90),
            PauseCommand(3.0),
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(-2.0, 0.0)),
            PauseCommand(3.0),

            TurnHeadingSwerveCommand(self.drivetrain, 90),
            PauseCommand(3.0),
            DriveDistanceSwerveCommand(self.drivetrain, Translation2d(0.0, -2.0)),
            PauseCommand(3.0),


        # DriveDistanceSwerveCommand(self.drivetrain, self.movement_translation),
        # PauseCommand(3.0),
        # TurnHeadingSwerveCommand(self.drivetrain, self.heading_change_degrees),
        # PauseCommand(3.0),

        )

