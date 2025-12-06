from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from phoenix6 import utils


#  NEED TO UPDATE TO HANDLE FULL ROTATION.  SEEMS TO HAVE AN ERROR WHEN CROSSING +/-180
##  Pose2d heading is relative to the field. Can I make it relative to the front azimuth of the robot

class TurnHeadingSwerveCommand(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, heading_change_degrees : float) -> None:
        self.drivetrain = drivetrain

        self.speed = 0.0
        self.heading_change_degrees = heading_change_degrees
        self.kP = 0.1
        self.kI = 0.05
        self.kD = 0.0
        self.kF = 0.0  # Feedforward constant (optional, but often useful)
        self.pid_controller = PIDController(self.kP, self.kI, self.kD)
        # self.pid_controller.setInputRange(-180.0, 180.0) # Gyro range
        # self.pid_controller.setContinuous(True) # Heading is continuous (wraps around)
        # self.pid_controller.disableContinuousInput()
        self.pid_controller.enableContinuousInput(-180.0, 180.0)
        # self.pid_controller.setInputRange(-180.0, 180.0)
        # self.pid_controller.setOutputRange(-1.0, 1.0)
        #  NEED TO UPDATE TO HANDLE FULL ROTATION.  SEEMS TO HAVE AN ERROR WHEN CROSSING +/-180
        self.pid_controller.setTolerance(1.0)  
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        """
        1) Reset the PID controller
        2) Get the robot's current Heading
        3) Calculate the target robot heading

        """
        self.pid_controller.reset()

        if utils.is_simulation():
            self.current_gyro_heading = self.drivetrain.get_state().pose.rotation().degrees()
        else:
            self.current_gyro_heading = self._gyro.getAngle()

        # Calculate target heading  (Positive is Counter-Clockwise)
        self.target_heading = self.current_gyro_heading + self.heading_change_degrees

        
    def execute(self) -> None:
        """
        1) Get the current robot Heading
        2) Perform PID Calculation
        3) Drive robot rotation
        """

        if utils.is_simulation():
            self.current_gyro_heading = self.drivetrain.get_state().pose.rotation().degrees()
        else:
            self.current_gyro_heading = self._gyro.getAngle()

        self.turn_speed = self.pid_controller.calculate(self.current_gyro_heading, self.target_heading)

        # print(f"self.current_gyro_heading:: {self.current_gyro_heading:5.1f}   ", end='')
        # print(f"self.target_heading::  {self.target_heading:5.1f}")

        self.drivetrain.driving_change_heading(self.turn_speed)

    def isFinished(self) -> bool:
       return self.pid_controller.atSetpoint()        

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop_driving()
        print (f"Complete Turn !!!!!!!!!!!!")

        if utils.is_simulation():
            self.current_gyro_heading = self.drivetrain.get_state().pose.rotation().degrees()
        else:
            self.current_gyro_heading = self._gyro.getAngle()

        print(f"self.current_gyro_heading:: {self.current_gyro_heading:5.1f}")

