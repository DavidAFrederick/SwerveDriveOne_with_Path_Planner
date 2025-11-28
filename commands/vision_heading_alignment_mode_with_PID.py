import wpilib
from commands2 import Command, PIDCommand
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.vision_subsystem import VisionSystem 
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from apriltagalignmentdata import AprilTagAlignmentData
from phoenix6 import swerve
from phoenix6.swerve.requests import RobotCentric
from wpimath.controller import PIDController

"""
PIDCommands integrate the "calculate" function into the command function.
If separate/independent PID controllers are used, additional functions are needed

"""

class AprilTagHeadingAligmentModePID(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem , 
                 led: LEDSubsystem,
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
        self.apriltag_alignment_data = apriltag_alignment_data
      
              # PID Controller for heading
        self.heading_kP = 0.03  # Proportional gain
        self.heading_kI = 0.00  # Integral gain
        self.heading_kD = 0.005 # Derivative gain
        self.heading_PID_controller = PIDController(self.heading_kP, self.heading_kI, self.heading_kD)
        self.heading_PID_controller.setTolerance(2.0)
        # Not sure if we need this capability
        # self.heading_pid_controller.enableContinuousInput(-180.0, 180.0) # Gyro reports degrees -180 to 180

              # PID Controller for Position relative to AprilTag Position
        self.position_kP = 0.03  # Proportional gain
        self.position_kI = 0.00  # Integral gain
        self.position_kD = 0.005 # Derivative gain
        self.position_PID_controller = PIDController(self.position_kP, self.position_kI, self.position_kD)

        self.addRequirements(drivetrain, vision, led)

    def initialize(self) -> None:
        self.led.red()
        self.heading_PID_controller.reset()
        self.position_PID_controller.reset()

    def execute(self) -> None:

        current_fpga_time = wpilib.Timer.getFPGATimestamp()
        print (f"Vision    Current Yaw: {self.apriltag_alignment_data.apriltag_yaw:5.1f}  at {current_fpga_time:6.1f} ")

        self.turn_rate = self.heading_PID_controller.calculate(self.apriltag_alignment_data.apriltag_yaw, 0)
        # self.position_speed = self.heading_PID_controller.calculate(current_heading, self.target_position)

        self.swerve_drive_request = (
            RobotCentric()
            .with_velocity_x( 0.0 )
            .with_velocity_y( 0.0 )
            .with_rotational_rate(self.turn_rate)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.drivetrain.set_control(self.swerve_drive_request)
        
    def isFinished(self) -> bool:
       if (self.heading_PID_controller.atSetpoint()):     ##  Need to figure out how to handle 2 PID controllers
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



