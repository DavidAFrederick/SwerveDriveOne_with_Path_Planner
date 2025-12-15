from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from phoenix6 import utils
import math
from apriltagalignmentdata import AprilTagAlignmentData

class TurnHeadingSwerveCommand(Command):
    """
    Just changes heading of the robot.
    """
    def __init__(self, drivetrain : CommandSwerveDrivetrain, heading_change_degrees : float, apriltag_alignment_data : AprilTagAlignmentData = None) -> None:
        self.drivetrain = drivetrain
        self.heading_change_degrees = heading_change_degrees
        self.apriltag_alignment_data = apriltag_alignment_data

        self.speed = 0.0
        self.turn_clamped_max_speed = 2.0
        self.kP = 10.0
        self.kI = 0.05
        self.kD = 0.0
        self.kF = 0.0  # Feedforward constant (optional, but often useful)
        self.pid_heading_controller = PIDController(self.kP, self.kI, self.kD)
        self.pid_heading_controller.enableContinuousInput(-180.0, 180.0)
        self.tolerance_in_degrees = 2.0
        self.tolerance_in_radians = self.tolerance_in_degrees / (180/math.pi)
        self.pid_heading_controller.setTolerance( self.tolerance_in_radians )  
        self.addRequirements(drivetrain)


        self.tolerance_in_degrees = 2.0
        self.tolerance_in_radians = self.tolerance_in_degrees / (180/math.pi)
        self.pid_heading_controller.setTolerance( self.tolerance_in_radians )  

    def initialize(self) -> None:
        """
        1) Reset the PID controller
        2) Get the robot's current Heading
        3) Calculate the target robot heading
        4) Run the PID loop calculation
        5) Clamp the speed 
        6) Drive the robot's heading

        """
        if (self.apriltag_alignment_data != None):
            self.heading_change_degrees = - self.apriltag_alignment_data.get_apriltag_turnpoint_angle_degrees()



        self.pid_heading_controller.reset()
        self.current_gyro_heading = self.drivetrain.get_robot_heading()
        print (f"Turn Heading Swerve Command:   Requested Turn: {self.heading_change_degrees:5.2f} ", end='')
        print (f"   Gyro Heading {self.current_gyro_heading:5.2f}   (degrees)")

        # Calculate target heading  (Positive is Counter-Clockwise)
        self.target_heading = self.current_gyro_heading + self.heading_change_degrees

        
    def execute(self) -> None:
        """
        1) Get the current robot Heading
        2) Perform PID Calculation
        3) Drive robot rotation
        """

        self.current_gyro_heading = self.drivetrain.get_robot_heading()
        self.turn_speed = self.pid_heading_controller.calculate(self.current_gyro_heading, self.target_heading)

        ## Clamp Heading Change Speed
        if (self.turn_speed >  self.turn_clamped_max_speed): self.turn_speed =  self.turn_clamped_max_speed
        if (self.turn_speed < -self.turn_clamped_max_speed): self.turn_speed = -self.turn_clamped_max_speed

        self.drivetrain.driving_change_heading(self.turn_speed)
        # print (f"  Requested Heading Change  {self.heading_change_degrees:5.2f} Error {(self.current_gyro_heading -self.target_heading):5.2f}")

    def isFinished(self) -> bool:
       return self.pid_heading_controller.atSetpoint()        

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop_driving()
        print(f"Complete Turn !!!!!!!!!  ", end='')
        print(f"self.current_gyro_heading:: {self.current_gyro_heading:5.1f}   ", end='')
        print(f"self.target_heading::  {self.target_heading:5.1f}  (Degrees)")

