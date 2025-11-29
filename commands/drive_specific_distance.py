import wpilib
from commands2 import Command, PIDCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from phoenix6.swerve.requests import FieldCentric, RobotCentric, Translation2d, Transform2d
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d

class DriveDistanceSwerveCommand(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, distance_feet) -> None:
        self.drivetrain = drivetrain

        self.speed = 1.0
        self.current_distance = 0.0
        self.target_distance_feet = distance_feet
        self.kP = 1.0
        self.kI = 0.5
        self.kD = 0.0
        self.kF = 0.0  # Feedforward constant (optional, but often useful)
        self.pid_controller = PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.setTolerance(0.1) # Set tolerance for onTarget()

        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        self.pid_controller.reset()

    def execute(self) -> None:
        self.current_distance = self.drivetrain.get_state().pose.x_feet
        self.speed = self.pid_controller.calculate(self.current_distance, self.target_distance_feet)

        print (f"Current distance: {self.current_distance:5.2f}   at speed: {self.speed:5.2f}")

        self.drivetrain.driving_forward(self.speed)

    def isFinished(self) -> bool:
       return self.pid_controller.atSetpoint()        

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop_driving()
        print (f"Complete !!!!!!!!!!!!")

    def code_reminders_not_being_used():
        #####(How to move a pose position)#############
        #   Example output:
        # self.drivetrain.get_state().pose Pose2d(Translation2d(x=3.304821, y=3.145566), Rotation2d(0.005561))


        # from wpimath.geometry import Pose2d, Translation2d, Rotation2d

        # Initial pose of the robot (x, y, rotation)
        initial_pose = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0))
        print(f"Initial pose: {initial_pose}")

        # Relative translation to apply
        relative_translation = Translation2d(0.5, 0.1)
        print(f"Relative translation: {relative_translation}")

        # Rotation to apply to the translation component
        # This is often derived from the robot's current orientation
        relative_rotation = Rotation2d.fromDegrees(90)
        print(f"Relative rotation: {relative_rotation}")

        # Create a Transform2d from the translation and rotation
        relative_transform = Transform2d(relative_translation, relative_rotation)

        # Apply the transformation to the pose
        updated_pose = initial_pose.transformBy(relative_transform)
        print(f"Updated pose: {updated_pose}")

        # The updated_pose is a new Pose2d object with the transformation applied.
        # The original initial_pose remains unchanged.
        print(f"Original pose after transformation: {initial_pose}")


###########
