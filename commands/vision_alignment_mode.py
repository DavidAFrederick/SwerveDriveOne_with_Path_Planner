import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from commands2 import Command
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.vision_subsystem import VisionSystem 
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from apriltagalignmentdata import AprilTagAlignmentData
from phoenix6 import swerve, utils
from phoenix6.swerve.requests import RobotCentric

class AprilTagAligmentMode(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem, 
                 led: LEDSubsystem,
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision
        self.led = led
        self.apriltag_alignment_data = apriltag_alignment_data
      
        self.speed = 0.0
        self.heading_change_degrees = 0.0
        self.kP = 0.1
        self.kI = 0.05
        self.kD = 0.0
        self.kF = 0.0  # Feedforward constant (optional, but often useful)
        self.pid_controller = PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.enableContinuousInput(-180.0, 180.0)
        self.pid_controller.setTolerance(3.0)  

        self.addRequirements(drivetrain, vision, led)

    def initialize(self) -> None:
        """
        1) Reset the PID controller
        2) Get the robot's current Heading
        3) Calculate the target robot heading
        """
        print(">>>>>  AprilTagAligmentMode <<<<<<<<<<<")

        self.pid_controller.reset()
        self.target_heading = 0.0
        self.led.red()


    def execute(self) -> None:
        """
        1) Get the current Yaw to the AprilTag from PhotonVision
        2) Perform PID Calculation
        3) Drive robot rotation
        """

        # Get Yaw to Target (Robot-Centric)
        self.vision.get_tag_data()                  #  Basic - Yaw to AprilTag
        
        if (self.apriltag_alignment_data.get_apriltag_alignment_data_Target_present()):
            self.yaw = self.apriltag_alignment_data.get_apriltag_alignment_data_yaw()
            self.turn_speed = self.pid_controller.calculate( self.yaw, 0)
            # print (f" Turning to Target:  {(self.yaw):5.2f}")
            
            ## Clamp Heading Change Speed
            self.turn_clamped_max_speed = 0.8
            if (self.turn_speed >  self.turn_clamped_max_speed): self.turn_speed =  self.turn_clamped_max_speed
            if (self.turn_speed < -self.turn_clamped_max_speed): self.turn_speed = -self.turn_clamped_max_speed

            self.drivetrain.driving_change_heading(self.turn_speed)
        else:
            self.drivetrain.driving_change_heading(0.0)  # Stop turning
            print  ("Target not present  !!!!!!!!!!!!!!!!!!!!")
            self.yaw = 0.0


    def isFinished(self) -> bool:       
        return self.pid_controller.atSetpoint()        

    def end(self, interrupted: bool) -> None:
        self.led.green()

        self.drivetrain.stop_driving()
        print (f"Complete Turn !!!!!!!!!!!!")

