import wpilib
import commands2
import commands2.pidcontroller
import rev

class ElevatorSubsystem(commands2.PIDSubsystem):
    """
    A subsystem that controls an elevator using a PID controller.
    """

    # PID Constants (tune these for your specific elevator)
    kP = 0.05
    kI = 0.001
    kD = 0.00
    kF = 0.0  # Feedforward constant (optional, but often useful)

    # Elevator positions (in units relevant to your sensor, e.g., rotations, inches)
    BOTTOM_POSITION = 0.0
    TOP_POSITION = 10.0

    def __init__(self):
        super().__init__(
            commands2.PIDController(self.kP, self.kI, self.kD),
            self.kF  # Optional feedforward
        )

        # Motor Controller (e.g., SparkMax, TalonSRX)
        self.motor = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        # Set output range for PID controller (e.g., -1 to 1 for motor speed)
        self.getController().setSetpoint(self.BOTTOM_POSITION)  # Initial setpoint
        self.getController().setTolerance(0.1)  # Tolerance for setpoint
        self.getController().setOutputRange(-0.8, 0.8) # Limit motor output

        # Configure the motor
        self.motor.setInverted(False) # Adjust as needed

    def getMeasurement(self) -> float:
        """
        Returns the current position of the elevator.
        This method is called by the PIDController to get the process variable.
        """
        return self.encoder.getPosition()

    def useOutput(self, output: float, setpoint: float):
        """
        Uses the output of the PID controller to control the elevator motor.
        This method is called periodically by the PIDSubsystem.
        """
        # Apply feedforward if used
        # output += self.kF * setpoint # Example feedforward

        self.motor.set(output)

    def setElevatorPosition(self, position: float):
        """
        Sets the desired position for the elevator.
        """
        self.getController().setSetpoint(position)

    def atSetpoint(self) -> bool:
        """
        Checks if the elevator is at the desired setpoint within the tolerance.
        """
        return self.getController().atSetpoint()

    def periodic(self):
        """
        This method is called once per scheduler run.
        """
        # You can add additional periodic tasks here if needed
        pass

    def stop(self):
        """
        Stops the elevator motor.
        """
        self.motor.stopMotor()
        self.disable() # Disable the PID controller


#################################


# from commands2 import PIDCommand
# from wpimath.controller import PIDController
# from wpimath.geometry import Rotation2d
# from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain # Assuming you have a DriveTrain subsystem

# class TurnToAngle(PIDCommand):
#     def __init__(self, target_angle_degrees: float, drive: CommandSwerveDrivetrain):
#         # Create a PIDController with appropriate P, I, D gains
#         # These gains will need to be tuned for your specific robot
#         self.pid_controller = PIDController(0.05, 0.001, 0.005) # Example gains
#         self.pid_controller.enableContinuousInput(-180, 180) # For angles, enable continuous input

#         super().__init__(
#             self.pid_controller,
#             # Measurement source: Get the current robot heading from the drivetrain
#             # Assuming getHeading() returns a Rotation2d object
#             lambda: drive.getHeading().degrees(),
#             # Setpoint source: The target angle in degrees
#             target_angle_degrees,
#             # Use output: Apply the calculated output to the drivetrain
#             lambda output: drive.arcadeDrive(0, output), # Apply rotation only
#             drive # Require the drivetrain subsystem
#         )

#         self.addRequirements(drive)
#         self.getController().setTolerance(2.0) # Set tolerance for onTarget()

#     def isFinished(self) -> bool:
#         # The command finishes when the robot is within the tolerance of the target angle
#         return self.getController().atSetpoint()

#     def end(self, interrupted: bool) -> None:
#         # Stop the drivetrain when the command ends
#         self.requirements[0].arcadeDrive(0, 0)


#=============
# from commands2 import PIDCommand
# from wpimath.controller import PIDController

# class MyPIDCommand(PIDCommand):
#     def __init__(self, subsystem, setpoint_source_func, measurement_source_func, kP, kI, kD):
#         # Create a PIDController instance
#         controller = PIDController(kP, kI, kD)

#         # Call the parent constructor
#         super().__init__(
#             controller,
#             measurement_source_func,  # Function to get the current measurement
#             setpoint_source_func,     # Function to get the desired setpoint
#             lambda output: subsystem.set_motor_speed(output),  # Function to apply output to the motor
#             subsystem  # The subsystem this command acts upon
#         )

#         # Optionally set tolerances or other controller parameters
#         self.getController().setTolerance(0.1) # Example: set a tolerance for the error


import wpilib
from wpilib.drive import DifferentialDrive
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Drivetrain motors
        self.left_motor = wpilib.PWMSparkMax(0)
        self.right_motor = wpilib.PWMSparkMax(1)
        self.right_motor.setInverted(True)  # Invert one side if needed

        self.robot_drive = DifferentialDrive(self.left_motor, self.right_motor)

        # Gyroscope for heading feedback
        self.gyro = wpilib.ADXRS450_Gyro(wpilib.SPI.Port.kOnboardCS0)
        self.gyro.calibrate()

        # PID Controller for heading
        self.kP = 0.03  # Proportional gain
        self.kI = 0.00  # Integral gain
        self.kD = 0.005 # Derivative gain
        self.heading_pid_controller = PIDController(self.kP, self.kI, self.kD)
        self.heading_pid_controller.enableContinuousInput(-180.0, 180.0) # Gyro reports degrees -180 to 180

        # Target heading
        self.target_heading = 0.0 # Desired heading in degrees

    def autonomousInit(self):
        self.gyro.reset()
        self.heading_pid_controller.reset()
        self.target_heading = 0.0 # Start facing forward

    def autonomousPeriodic(self):
        current_heading = self.gyro.getAngle() # Get current heading from gyro

        # Calculate PID output for turning
        turn_speed = self.heading_pid_controller.calculate(current_heading, self.target_heading)

        # Drive forward while correcting heading
        forward_speed = -0.5 # Example forward speed
        self.robot_drive.arcadeDrive(forward_speed, turn_speed)

    def teleopInit(self):
        self.heading_pid_controller.reset()

    def teleopPeriodic(self):
        # Example: Control heading with joystick input in teleop
        # For simplicity, this example doesn't use PID for teleop heading control
        # but focuses on autonomous. You could adapt the PID for teleop as well.
        forward = -self.stick.getRawAxis(1) # Left Y-axis
        turn = self.stick.getRawAxis(4)    # Right X-axis
        self.robot_drive.arcadeDrive(forward, turn)

if __name__ == '__main__':
    wpilib.run(MyRobot)