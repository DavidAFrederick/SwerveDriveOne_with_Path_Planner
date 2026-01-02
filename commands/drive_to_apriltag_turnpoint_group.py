from commands2 import SequentialCommandGroup, PrintCommand
import commands2
import commands2.cmd
from wpimath.geometry import Translation2d

from apriltagalignmentdata import  AprilTagAlignmentData
from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.drive_to_specific_point_with_lateral import DriveToSpecificPointXYSwerveCommand
from commands.pause_command import PauseCommand
from commands.vision_alignment_with_offset import AprilTagWithOffsetAligmentCalculation
from commands.ledcommand import LEDCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vision_subsystem import VisionSystem 
from subsystems.ledsubsystem import LEDSubsystem



class DriveToAprilTagTurnPointCmdGroup(SequentialCommandGroup):

    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem, led: LEDSubsystem,
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:

        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
        self.apriltag_alignment_data = apriltag_alignment_data
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        self.addCommands(
        PrintCommand(">>> ---------------------------------------"),

        LEDCommand(self.led,60),

        #  0 =  red
        #  30 = yellow-gold
        #  60 = green
        #  90 = Blue - Aqua
        # 120 = Dark Blue
        # 150 = Purple
        # -------------------
        PrintCommand(">>> Calculate turn point"),
        AprilTagWithOffsetAligmentCalculation(self.drivetrain, self.vision, self.apriltag_alignment_data),
        PrintCommand(">>> Done - Calculate turn point"),
        PrintCommand(">>> ---------------------------------------"),

        PrintCommand(">>> Pausing"),
        LEDCommand(self.led,120),
        PauseCommand(0.5),

        PrintCommand(">>> ---------------------------------------"),
        PrintCommand("Drive to Specific point"),

        LEDCommand(self.led,0),

        DriveToSpecificPointXYSwerveCommand(self.drivetrain, self.apriltag_alignment_data),
        PrintCommand(">>> ---------------------------------------"),

        PrintCommand(">>> Pausing"),
        PauseCommand(0.5),

        PrintCommand(">>> ---------------------------------------"),
        TurnHeadingSwerveCommand(self.drivetrain, 0.0, self.apriltag_alignment_data),

        PrintCommand(">>> Group Command Complete"),
        PrintCommand(">>> ---------------------------------------"),
        LEDCommand(self.led,60),


          )
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        

