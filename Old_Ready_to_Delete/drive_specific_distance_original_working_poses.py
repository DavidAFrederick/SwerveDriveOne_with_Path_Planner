import wpilib
from commands2 import Command, PIDCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from phoenix6.swerve.requests import FieldCentric, RobotCentric, Translation2d, Transform2d
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath import units


#  Get the current pose and add the desired translation
#
# Vocabulary
# pose
# Transformation
# Transform
# translation
# 

"""
Get the robot's current pose:   
current_pose = self.drivetrain.get_state().pose

Calculate a new position (pose) 
relative_transform = Transform2d(Translation2d(0.5, 0.1), Rotation2d.fromDegrees(90))

        ### ??? What is the difference between a Transform() and Translation()
        # Transform() - A transform is like a pose (x,y,heading)
        # Translation() - Just X and Y  components of a transform

How do we make it 

Issues:
1)  Pressing a second time, robot does not move, thinks it has already reached the end point

"""



class DriveDistanceSwerveCommand(Command):
    def __init__(self, drivetrain : CommandSwerveDrivetrain, target_translation2d : Translation2d) -> None:
        self.drivetrain = drivetrain

        self.speed = 0.0
        # self.current_distance = 0.0
        self.target_translation2d = target_translation2d
        self.kP = 1.0
        self.kI = 0.0
        self.kD = 0.1
        self.kF = 0.0  # Feedforward constant (optional, but often useful)
        self.pid_controller = PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.setTolerance(0.1) # Set tolerance for onTarget()
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        """
        1) Reset the PID controller
        2) Get the robot's current pose
        3) Calculate the target robot pose

        """
        self.pid_controller.reset()

        ### Get robot's current pose (Position and heading)
        self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())

        #### ?????  IS THERE A BETTER WAY OF GETTING THE ROBOT POSE?

        # Get just the X,Y position components of the current robot pose
        self.current_translation = self.current_pose.translation()
        # 
        # self.relative_transform = Transform2d(self.target_translation2d, Rotation2d.fromDegrees(0))
        # Create a pose for the target position by adding the X,Y position parts and setting the rotation to zero
        self.target_pose = Pose2d((self.current_translation  + self.target_translation2d), Rotation2d.fromDegrees(0))

        print(f"self.current_pose:: {self.current_pose}")
        print(f"self.relative_transform {self.relative_transform}")
        print(f"self.target_pose::  {self.target_pose}")

        #====(Notes)=========================================        

        # ??? What is the difference between a Transform() and Translation()
        # pose2d - Position on the field
        # Transform() - A transform is like a pose
        # Translation() - Just X and Y components of a transform

        # Create a Pose2D 
        new_pose = Pose2d( Translation2d(5.0, 2.0), Rotation2d.fromDegrees(90.0))

        # Create a Transform2d (A movement and rotation)
        new_transform = Transform2d(Translation2d(1.5, 2.0), Rotation2d(0))

        # Create a Translation2d()
        new_translation = Translation2d( 1.0, 2.0)


        # Create a transform2D from a translation2D (Just add the heading angle)
        #     self.relative_transform = Transform2d(self.target_translation2d, Rotation2d.fromDegrees(0))

        # Create a translation2D from a transform2D
        #    self.translation2D  = translation2D.toTranslation2d()


        ##   Extract the Translation2d components from each pose
        # translation1 = pose1.translation()
        # translation2 = pose2.translation()

        ## Calculate the distance between the two Translation2d objects
        # distance = translation1.getDistance(translation2)

        #====(Notes)=========================================        

        # self.x_distance_meters = self.target_pose2d.x
        # self.y_distance_meters = self.target_pose2d.y

        # self.relative_transform = Transform2d(
        #     Translation2d(
        #     self.x_distance_meters, 
        #     self.y_distance_meters), 
        #     Rotation2d.fromDegrees(0))
        
    def execute(self) -> None:
        """
        1) Get the current robot pose
        2) Calculate the distance between current pose and target pose
        3) 
        
        """

                ### Get robot's current pose (Position and heading)
        self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())
        
        # Calculate the distance between the robot's current position and the target position
        # Calculate the distance between the two poses
        ## 
        self.current_distance = self.current_pose.translation().distance(self.target_pose.translation())


        self.speed = - self.pid_controller.calculate(self.current_distance, 0)


        print (f"Robot Pos {self.current_pose.x:4.2f}: {self.current_pose.y:4.2f} ",end='')
        print (f"Target Pos {self.target_pose.x:4.2f}: {self.target_pose.y:4.2f}  ",end='')
        print (f"Current distance: {self.current_distance:5.2f}   at speed: {self.speed:5.2f}")

        self.drivetrain.driving_forward(self.speed)

    def isFinished(self) -> bool:
       return self.pid_controller.atSetpoint()        

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop_driving()
        print (f"Complete !!!!!!!!!!!!")
        self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
                            self.drivetrain.get_state().pose.y, 
                            self.drivetrain.get_state().pose.rotation().radians())
        print(f"self.current_pose:: {self.current_pose}")


    def code_reminders_not_being_used(self):
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

        #===================================================

        self.x_distance_feet  = 2
        self.y_distance_feet  = 0

        self.x_distance_meters = units.feetToMeters(self.x_distance_feet)
        self.y_distance_meters = units.feetToMeters(self.y_distance_feet)

        self.current_pose = self.drivetrain.get_state().pose
        #   Calculate a new position (pose) 
        self.relative_transform = Transform2d(Translation2d(
            self.x_distance_meters, self.y_distance_meters), 
            Rotation2d.fromDegrees(90))

        print(f"Original: X: {self.current_pose}    Final:  {self.relative_transform} ")


###########