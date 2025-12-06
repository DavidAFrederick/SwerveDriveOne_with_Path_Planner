import wpilib
from commands2 import Command, PIDCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from phoenix6.swerve.requests import FieldCentric, RobotCentric, Translation2d, Transform2d
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath import units
import math


class DriveDistanceSwerveCommand(Command):
    """
    Drives the robot forward in a straight line.  Robot Centric movement
    """


    def __init__(self, drivetrain : CommandSwerveDrivetrain, forward_movement_meters : float) -> None:
        self.drivetrain = drivetrain

        self.speed = 0.0
        self.forward_movement_meters = forward_movement_meters
        self.kP = 1.0
        self.kI = 0.5
        self.kD = 0.0
        self.kF = 0.0  # Feedforward constant (optional, but often useful)
        self.pid_controller = PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.setTolerance(0.1) # Set tolerance for onTarget()
        self.addRequirements(drivetrain)

        """
               Algorithm:
        __init__
        1) Get current Translation2d (x,y position on field) [Field-centric]
        2) Get the current pose.rotation (Heading in field centric)
        3) Get input parameter. This is the distance to travel [Robot-centric]
        4) Calculate the delta-x and delta-y distance to move [Field-centric]
        5) Calculate target position (x,y on field-centric)

        __exec__
        1) Get current Translation2d (x,y position on field) [Field-centric]
        2) Calculate remaining delta-x  and delta-y
        3) Calculate distance to target
        4) Run PID calculation

        """

    def initialize(self) -> None:

        self.pid_controller.reset()

        ### Get robot's current pose (Position and heading)
        self.initial_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())

        #### ?????  IS THERE A BETTER WAY OF GETTING THE ROBOT POSE?

        # Get just the X,Y position and rotation components of the current robot pose [Field-centric]
        self.initial_translation     = self.initial_pose.translation()
        self.initial_heading_degrees = self.initial_pose.rotation().degrees()
        self.initial_heading_radians = self.initial_pose.rotation().radians()

        self.delta_x_field_movement  = self.forward_movement_meters * math.cos(self.initial_heading_radians)
        self.delta_y_field_movement  = self.forward_movement_meters * math.sin(self.initial_heading_radians)

        self.target_x_field_position = self.initial_translation.x + self.delta_x_field_movement
        self.target_y_field_position = self.initial_translation.y + self.delta_y_field_movement

        print(f"Init:  {self.initial_translation.x:4.1f} {self.initial_translation.y:4.1f}  Heading: {self.initial_heading_degrees:4.1f}  ", end="")
        print(f"Delta: {self.delta_x_field_movement:4.1f} {self.delta_y_field_movement:4.1f} ", end="")
        print(f"Final: {self.target_x_field_position:4.1f} {self.target_y_field_position:4.1f}  ")
        


        # self.initial_heading = self.drivetrain.get_state().pose.rotation().degrees()
        # self.initial_heading_radians = self.drivetrain.get_state().pose.rotation().radians()

        # self.relative_transform = Transform2d(self.target_translation2d, Rotation2d.fromDegrees(0))
        # Create a pose for the target position by adding the X,Y position parts and setting the rotation to zero
        # self.target_pose = Pose2d((self.current_translation  + self.target_translation2d), Rotation2d.fromDegrees(0))
        # self.target_translation = Translation2d((self.current_translation  + self.target_translation2d))

        # print(f"self.current_translation:: {self.current_translation}")
        # # print(f"self.relative_transform {self.relative_transform}")
        # print(f"self.target_transform::  {self.target_translation}")
        
        ### NEED TO MAKE THIS ROBOT-CENTRIC VS FIELD-CENTRIC


    def execute(self) -> None:
        """
        __exec__
        1) Get current Translation2d (x,y position on field) [Field-centric]
        2) Calculate remaining delta-x  and delta-y
        3) Calculate distance to target
        4) Run PID calculation

        """

        ### Get robot's current pose (Position and heading)
        self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())

        self.current_translation     = self.current_pose.translation()
        self.current_heading_degrees = self.current_pose.rotation().degrees()

        self.remaining_delta_x_field_movement = self.target_x_field_position - self.current_translation.x
        self.remaining_delta_y_field_movement = self.target_y_field_position - self.current_translation.y

        self.current_distance = math.sqrt( math.pow (self.remaining_delta_x_field_movement,  2 ) +  
                                          math.pow(self.remaining_delta_x_field_movement, 2) )


        print(f"current:  {self.current_translation.x:4.1f} {self.current_translation.y:4.1f} Heading: {self.current_heading_degrees:4.1f}  ", end='')
        print (f"Remaining {self.remaining_delta_x_field_movement:4.1f}  {self.remaining_delta_x_field_movement:4.1f} Distance {self.current_distance:4.1f} " , end='')
        print(f"Final: {self.target_x_field_position:4.1f} {self.target_y_field_position:4.1f}  ")




                ### Get robot's current pose (Position and heading)
        # self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
        #                            self.drivetrain.get_state().pose.y, 
        #                            self.drivetrain.get_state().pose.rotation().radians())
        
        # self.current_translation = self.current_pose.translation()
        # self.current_translation = Translation2d(self.drivetrain.get_state().pose.x,
        #                            self.drivetrain.get_state().pose.y)
        # self.current_heading = self.drivetrain.get_state().pose.rotation().degrees()
        # self.current_heading_radians = self.drivetrain.get_state().pose.rotation().radians()

        # Calculate the distance to the target position using C = sqrt( sqr(Adjancent side) + sqr (Opposite side))

        # self.adjancent_side = self.forward_movement_meters * math.cos(self.current_heading_radians) 
        # self.opposite_side = self.forward_movement_meters * math.sin(self.current_heading_radians) 
        # self.current_distance = math.sqrt( math.pow (self.adjancent_side,2 ) +  math.pow(self.opposite_side, 2) )
        # print (f" Adj: {self.adjancent_side:4.1f}  Opp: {self.opposite_side:4.1f}  Distance {self.current_distance:4.1f}")

        # print (f"target_translation2d:  X: {self.target_translation2d.x}    Y: {self.target_translation2d.y}")

        # Calculate the distance between the robot's current position and the target position
        # self.current_distance = self.current_pose.translation().distance(self.target_pose.translation())
        # self.current_distance = self.current_translation.distance(self.target_translation)

        self.speed = - self.pid_controller.calculate(self.current_distance, 0)

        ## Clamp Speed
        self.clamped_max_speed = 1.0
        if (self.speed > self.clamped_max_speed): self.speed = self.clamped_max_speed
        if (self.speed < -self.clamped_max_speed): self.speed = -self.clamped_max_speed
          


        # print (f"Robot Pos {self.current_translation.x:4.2f}: {self.current_translation.y:4.2f} ",end='')
        # # print (f"Target Pos {self.target_translation.x:4.2f}: {self.target_translation.y:4.2f}  ",end='')
        print (f"Current distance: {self.current_distance:5.2f}   at speed: {self.speed:5.2f}  Heading: {self.current_heading_degrees:5.1f}")

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

###########