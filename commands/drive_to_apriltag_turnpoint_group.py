from commands2 import SequentialCommandGroup, PrintCommand
import commands2
import commands2.cmd
from wpimath.geometry import Translation2d

from apriltagalignmentdata import  AprilTagAlignmentData
from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.drive_to_specific_point import DriveToSpecificPointSwerveCommand
from commands.pause_command import PauseCommand
from commands.vision_alignment_with_offset import AprilTagWithOffsetAligmentCalculation
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vision_subsystem import VisionSystem 


class DriveToAprilTagTurnPointCmdGroup(SequentialCommandGroup):

    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem, 
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:

        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.apriltag_alignment_data = apriltag_alignment_data

        self.addCommands(
        PrintCommand("Calculate turn point"),
        AprilTagWithOffsetAligmentCalculation(self.drivetrain, self.vision, self.apriltag_alignment_data),
        PrintCommand("Done - Calculate turn point"),

        PrintCommand("Pausing"),
        PauseCommand(2.0),

        PrintCommand("Drive to Specific point"),

        DriveToSpecificPointSwerveCommand(self.drivetrain, self.apriltag_alignment_data),


        # DriveToSpecificPointSwerveCommand(self.drivetrain,   # Passing in Translation2d contain X,Y
        #                                   self.apriltag_alignment_data.get_apriltag_turnpoint_position_meters()),

        # DriveToSpecificPointSwerveCommand(self.drivetrain, 
        #                                   self.apriltag_alignment_data.get_apriltag_turnpoint_X_position_meters(), 
        #                                   self.apriltag_alignment_data.get_apriltag_turnpoint_Y_position_meters()),


        PrintCommand("Pausing"),
        PauseCommand(2.0),

        TurnHeadingSwerveCommand(self.drivetrain, 
                                 self.apriltag_alignment_data.get_apriltag_turnpoint_angle_degrees()),

        PrintCommand("Done"),


          )
        

