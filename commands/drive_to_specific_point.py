import wpilib
from commands2 import Command, PIDCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from phoenix6.swerve.requests import FieldCentric, RobotCentric, Translation2d, Transform2d
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath import units
import math


class DriveToSpecificPointSwerveCommand(Command):
    """
    Drives the robot forward (robot-centric) to a point defined x meters forward and y meters across.  
    Robot Centric movement using PID loops for both forward movement and heading change movemment.

    """


    def __init__(self, drivetrain : CommandSwerveDrivetrain, forward_movement_meters : float, lateral_position_meters : float) -> None:
        self.drivetrain = drivetrain

        # Create the two PID controllers,  One for forward movement and one for heading change
        self.speed = 0.0
        self.distance_clamped_max_speed = 2.0
        self.forward_movement_meters = forward_movement_meters
        self.distance_kP = 1.0
        self.distance_kI = 0.5
        self.distance_kD = 0.0
        self.distance_kF = 0.0  
        self.pid_distance_controller = PIDController(self.distance_kP, self.distance_kI, self.distance_kD)
        self.pid_distance_controller.setTolerance(0.05) 

        self.turn_speed = 0.0
        self.turn_clamped_max_speed = 2.0
        self.lateral_position_meters = lateral_position_meters
        self.heading_kP = 3.0
        self.heading_kI = 0.5
        self.heading_kD = 0.0
        self.heading_kF = 0.0  
        self.pid_heading_controller = PIDController(self.heading_kP, self.heading_kI, self.heading_kD)
        self.pid_heading_controller.setTolerance(0.5) 

        self.addRequirements(drivetrain)

        """
               Algorithm:
        Initialization (Runs each time the command is triggered)

        1) Get current Translation2d (x,y position on field) [Field-centric]
        2) Get the current pose.rotation (Heading in field centric)
        3) Get input parameters (forward and lateral). This is the distance to travel [Robot-centric]
        4) Calculate the distance the robot needs to travel from the current position to the final position (Robot-Centric)
        4) Calculate the delta-x and delta-y distance of the move [Field-centric]
        5) Calculate target position (x,y on field-centric) by adding the current position to the delta-X and Y


        """

    def initialize(self) -> None:

        self.pid_distance_controller.reset()
        self.pid_heading_controller.reset()

        ### Get robot's current pose (Position and heading) [Field-Centric]
        self.initial_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())

        # Get the X,Y position and rotation components of the current robot pose [Field-centric]
        self.initial_translation     = self.initial_pose.translation()
        self.initial_heading_degrees = self.initial_pose.rotation().degrees()
        self.initial_heading_radians = self.initial_pose.rotation().radians()

        # Calculate the field-centric change in X position and Y position from the current position to the target position
        # for just the forward movement of the robot (Not including the cross motion)

        # Calculate field-centric position of final (distance + cross) target point
        # This is robot-centric 
        # First calculate the distance then calculate the heading [ ArcTan (Opposite over Adjancent)]
        self.distance_to_final_final_target_point = math.sqrt(  math.pow(self.forward_movement_meters, 2 ) +  
                                          math.pow(self.lateral_position_meters, 2) )
        
        self.angle_to_final_final_target_point_radians = math.atan2 (self.lateral_position_meters, self.forward_movement_meters )

        #  Calculate the Field-centric, X component and Y component for the total planned movement to be used for PID calculations
        self.final_final_target_point_x = self.distance_to_final_final_target_point * math.cos(self.angle_to_final_final_target_point_radians)
        self.final_final_target_point_y = self.distance_to_final_final_target_point * math.sin(self.angle_to_final_final_target_point_radians)
        
        ### >>> Does not include the robot start position

        # Calculate  the Field-oriented X and Y position of the target field location (End point)
        self.target_x_field_position = self.initial_translation.x + self.final_final_target_point_x
        self.target_y_field_position = self.initial_translation.y + self.final_final_target_point_y

        # self.target_x_field_position = self.final_final_target_point_x
        # self.target_y_field_position = self.final_final_target_point_y

        print (f"Start Drive ------")
        print(f"Init: {self.initial_translation.x:4.1f} {self.initial_translation.y:4.1f} Heading: {self.initial_heading_degrees:4.1f}  ", end="")
        print(f"Delta: X: {self.final_final_target_point_x:4.1f} Y: {self.final_final_target_point_y:4.1f} ", end="")
        print(f"Final Pos: X: {self.target_x_field_position:4.1f}  Y: {self.target_y_field_position:4.1f}  ")

        print (f"Cross Point positions ------")
        print (f"Distance:  {self.distance_to_final_final_target_point:5.2f}   ", end='')
        print (f"Position:  X: {self.final_final_target_point_x} Y: {self.final_final_target_point_y} ", end='')
        print (f"Angle:  {57.296 *  self.angle_to_final_final_target_point_radians:4.2f} ")
        
        
    def execute(self) -> None:
        """
        Algorithm:
        __exec__
        1) Get current Translation2d (x,y position on field) and heading [Field-centric]
        2) Calculate remaining delta-x  and delta-y  (Target position - current position)
        3) Calculate distance to target position
        4) Run PID calculation for forward movement

        5) Calculate the current heading to the target end point 
        6) Run PID calculation for heading

        """

        ### Get robot's current pose (Position and heading)
        self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())

        self.current_translation     = self.current_pose.translation()
        self.current_heading_degrees = self.current_pose.rotation().degrees()
        self.current_heading_radians = self.current_pose.rotation().radians()

        # Calculate the remaining travel in X and Y components between the current position and the final position (Field-oriented)
        self.remaining_delta_x_field_movement = self.target_x_field_position - self.current_translation.x
        self.remaining_delta_y_field_movement = self.target_y_field_position - self.current_translation.y

        self.current_distance = math.sqrt(math.pow(self.remaining_delta_x_field_movement, 2) +  
                                          math.pow(self.remaining_delta_y_field_movement, 2) )

        # PID Loop calculation
        self.distance_speed = - self.pid_distance_controller.calculate(self.current_distance, 0)

        ## Clamp Forward Motion Speed
        if (self.distance_speed > self.distance_clamped_max_speed): self.distance_speed = self.distance_clamped_max_speed
        if (self.distance_speed < -self.distance_clamped_max_speed): self.distance_speed = -self.distance_clamped_max_speed
          

        #- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        #  Heading Calculations

        # Calculate the actual heading needed to get to the target end point
        self.target_heading_radians = math.atan2( self.remaining_delta_y_field_movement, self.remaining_delta_x_field_movement)

        # PID heading calculation to get to the required heading
        self.turn_speed = self.pid_heading_controller.calculate(self.current_heading_radians, self.target_heading_radians)

        ## Clamp Heading Change Speed
        if (self.turn_speed >  self.turn_clamped_max_speed): self.turn_speed =  self.turn_clamped_max_speed
        if (self.turn_speed < -self.turn_clamped_max_speed): self.turn_speed = -self.turn_clamped_max_speed


        # We can only drive the robot in forward motion or heading change. Do heading first, then distance from target

        if (self.pid_heading_controller.atSetpoint()):
            self.drivetrain.driving_forward(self.distance_speed)
            print (f"^^^ Forward Speed:  {self.distance_speed:5.2f} ", end='')
            print(f"Forward: current:  {self.current_translation.x:5.2f} {self.current_translation.y:5.2f} Heading: {self.current_heading_degrees:5.2f}  ", end='')
            print (f"Remaining {self.remaining_delta_x_field_movement:5.2f}  {self.remaining_delta_x_field_movement:5.2f} Distance {self.current_distance:5.2f} " , end='')
            print(f"Final: {self.target_x_field_position:5.2f} {self.target_y_field_position:5.2f}  ")

        else:
            self.drivetrain.driving_change_heading(self.turn_speed)
            print (f">>>>> Turn Speed {self.turn_speed:5.1f}  ", end='')
        print (f"Current distance: {self.current_distance:5.2f}   Heading: {self.current_heading_degrees:5.1f}  ", end='')
        print (f"Heading Error: {57.296 * (self.target_heading_radians - self.current_heading_radians):5.2f} (Target-current) ")


    def isFinished(self) -> bool:
       return self.pid_distance_controller.atSetpoint()        

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop_driving()
        print (f"Complete Drive   SP !!!!!!!!!!!!")
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