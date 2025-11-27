import wpilib
from commands2 import Command
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.vision_subsystem import VisionSystem 
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve

class AprilTagAligmentMode(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, vision : VisionSystem , led: LEDSubsystem) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
      
        self.addRequirements(drivetrain, vision, led)

    def initialize(self) -> None:
        self.led.red()

    def execute(self) -> None:
        print ("execute")

        drive_request = (
            swerve.requests.RobotCentric()
            .with_velocity_x( 2 )
            .with_velocity_y( 2 )
            .with_rotational_rate( 0 )
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        
        # Apply the drive request to the swerve drivetrain
        self.drivetrain.apply_request(lambda: drive_request)

    def isFinished(self) -> bool:
       return False        ## Key command running until button released

    def end(self, interrupted: bool) -> None:
        self.led.green()
        # self.drivetrain.drive_robot_centric_swervedrive(0.0, 0.0, 0.0)



