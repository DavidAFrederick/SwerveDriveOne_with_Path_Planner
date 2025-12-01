import wpilib
from commands2 import Command
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.vision_subsystem import VisionSystem 
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from apriltagalignmentdata import AprilTagAlignmentData
from phoenix6 import swerve
from phoenix6.swerve.requests import RobotCentric


class AprilTagAligmentMode(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem , 
                 led: LEDSubsystem,
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
        self.apriltag_alignment_data = apriltag_alignment_data
      
        self.addRequirements(drivetrain, vision, led)
        self.YAW_THRESHOLD = 2

    def initialize(self) -> None:
        self.led.red()


    def execute(self) -> None:

        self.vision.get_tag_data()
        
        current_fpga_time = wpilib.Timer.getFPGATimestamp()
        # print (f"Vision    Current Yaw: {self.apriltag_alignment_data.apriltag_yaw:5.1f}  at {current_fpga_time:6.1f} ")

        self.turn_rate = 0.2 * self.apriltag_alignment_data.apriltag_yaw

        self.swerve_drive_request = (
            RobotCentric()
            .with_velocity_x( 0.0 )
            .with_velocity_y( 0.0 )
            .with_rotational_rate(self.turn_rate)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.drivetrain.set_control(self.swerve_drive_request)

        # TEMP
        self.apriltag_alignment_data.apriltag_yaw  = self.apriltag_alignment_data.apriltag_yaw - 0.01
        
    def isFinished(self) -> bool:
       if (self.apriltag_alignment_data.apriltag_yaw < self.YAW_THRESHOLD):
           return True
       else:
           return False

    def end(self, interrupted: bool) -> None:
        self.led.green()

        self.drivetrain.stop_driving()


        # self.swerve_drive_request = (
        #     RobotCentric()
        #     .with_velocity_x( 0.0 )
        #     .with_velocity_y( 0.0 )
        #     .with_rotational_rate(self.turn_rate)
        #     .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        # )
        # self.drivetrain.set_control(self.swerve_drive_request)
        # # self.drivetrain.drive_robot_centric_swervedrive(0.0, 0.0, 0.0)



