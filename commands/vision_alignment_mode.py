import wpilib
from commands2 import Command
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.vision_subsystem import VisionSystem 
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain



class AprilTagAligmentMode(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, vision : VisionSystem , led: LEDSubsystem) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
      
        # self.alignment_enabled = False

        # self.apriltag_present = False
        # self.apriltag_yaw = 0
        # self.apriltag_skew = 0
        # self.apriltag_distance = 0
        # self.april_tag_data = [self.apriltag_present, self.apriltag_yaw, self.apriltag_skew, self.apriltag_distance]

        self.addRequirements(drivetrain, vision, led)

    def initialize(self) -> None:
        self.led.red()
        self.drivetrain.drive_robot_centric_swervedrive(1.0, 1.0, 0.5)

    def execute(self) -> None:
        # self.april_tag_data = self.vision.get_tag_data()
        # print (f"Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}  Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")
        self.drivetrain.drive_robot_centric_swervedrive(0.5, 0.0, 0.0)
   
    def isFinished(self) -> bool:
       return False        ## Key command running until button released

    def end(self, interrupted: bool) -> None:
        self.led.green()
        self.drivetrain.drive_robot_centric_swervedrive(0.0, 0.0, 0.0)



