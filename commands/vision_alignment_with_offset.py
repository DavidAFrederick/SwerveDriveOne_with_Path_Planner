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
import math

class AprilTagWithOffsetAligmentCalculation(Command):
    """
    This command only does calculations to determine where to place the robot to
    be directly in front of the Apriltag.  The result is stored in the "AprilTagAlignmentData" data object
    
    """
    def __init__(self, drivetrain : CommandSwerveDrivetrain, 
                 vision : VisionSystem, 
                 apriltag_alignment_data : AprilTagAlignmentData) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision
        self.apriltag_alignment_data = apriltag_alignment_data
        self.addRequirements(drivetrain, vision)

    def initialize(self) -> None:

        """
        The goal of this algorithm is to place the robot directly in front of an AprilTag
        then rotate the robot to be perpendictular to the AprilTag a specific distance from the 
        Apriltag.

        When this command is activated, the initialization route gets the current robots pose 
        (field-oriented X,Y position and rotation) plus the AprilTag position (X,Y) and AprilTag
        rotation (relative to the robot).

        The turn-point should be about 1.4 times the width of the robot.

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -        

        Algorithm to calculate the turnPoint.
        [1] Get the current robot position and heading from the current robot pose (Field-Centric)
        [2] Get the AprilTag Transform3d from the vision subsystem. X and Y position plus AprilTag Rotation (Robot-Centric)
        [3] Calculate the direct distance from the robot to the center of the AprilTag [Pythagorean theorem]

        [4]] Calculate the side lengths and angles within an obtuse triange formed by the following 3 points [Field-Oriented]
           (P) Current position of the robot  (provided by pose) 
           (Q) Center of the AprilTag  
           (G) turn-point for robot to stop movement

        The length of the sides are determined as follows:
           Distance between (P) and (Q) - (side a) - Robot to AprilTag - Provided by PhotonVision [Pythagorean theorem]
           Distance between (Q) and (G) - (side b) - 140% * length of side of robot - 42"  = 1.08 meters
           Distance between (G) and (P) - (side c) - To be Calculated using Law of Cosines as follows:

           Side c = SQRT[ (side a)^2 + (side b)^2 - 2 * (side a) * (side b) *  COS (Angle C) ]
 
        The angles within this obtuse triange are determined as follows:
           Angle A (Opposite side a) - ArcSin((side a)/(side c) * Sin (Angle C))
           Angle B (Opposite side b) - ArcSin((side a)/(side b) * Sin (Angle B))
           Angle C (Opposite side c) - Angle Yaw + Angle Target (rotation) (OR (not sure)) Angle Yaw - Angle Target (rotation) 

        [5] Calculate the turn-point position in robot centric terms (delta-x, delta-y)

        [6] Place the data into the AprilTagAlignmentData data object


        # bestCameraToTarget=
        #   Transform3d(Translation3d(x=0.691508, y=0.219795, z=0.141702), 
        #   Rotation3d(x=0.008310, y=-0.060773, z=-2.588145))

        """
        print(">>>>>  AprilTag Aligment Mode with Offset (AprilTagWithOffsetAligmentCalculation) <<<<<<<<<<<")

        # [1] Get the current robot position and heading from the current robot pose (Field-Centric)

        ### Get robot's current pose (Position and heading) [Field-Centric]
        self.initial_pose = Pose2d(self.drivetrain.get_state().pose.x,
                                   self.drivetrain.get_state().pose.y, 
                                   self.drivetrain.get_state().pose.rotation().radians())

        # Get the X,Y position and rotation components of the current robot pose [Field-centric]
        self.initial_translation     = self.initial_pose.translation()
        self.initial_heading_degrees = self.initial_pose.rotation().degrees()
        self.initial_heading_radians = self.initial_pose.rotation().radians()

        # [2] Get the AprilTag Transform3d from the vision subsystem. X and Y position plus AprilTag Rotation (Robot-Centric)
        # Get Yaw to Target (Robot-Centric)

        ## TODO Need to add a check to see  if data is available

        #### TESTING self.vision.get_tag_data()                  #  Basic - Yaw to AprilTag
        # self.vision.get_tag_data()          # Calls vision subsystem and placed AprilTag target data into "apriltagalignmentdata" data object

        ## Temporary tool to load in test data to calculation verification
        self.apriltag_alignment_data.set_apriltag_bestCameraToTarget_TEST()
        print (f" >>>>>>> TEST MODE FOR DATA <<<<<<<<<<  vision_alignment_with_offset at line 121")
        
        if (self.apriltag_alignment_data.get_apriltag_alignment_data_Target_present()):     # Get the position and yaw of the AprilTag
            self.aprilTag_position_and_pose = self.apriltag_alignment_data.get_apriltag_bestCameraToTarget()

                            # Get the Transform3d from the camera to the target  [Robot-Centric]
            self.distance_to_AprilTag_X_meters = self.aprilTag_position_and_pose.translation().X()
            self.distance_to_AprilTag_Y_meters = self.aprilTag_position_and_pose.translation().Y()
            self.distance_to_AprilTag_Z_meters = self.aprilTag_position_and_pose.translation().Z()

            self.distance_to_AprilTag_X_feet = 3.28 * self.distance_to_AprilTag_X_meters
            self.distance_to_AprilTag_Y_feet = 3.28 * self.distance_to_AprilTag_Y_meters
            self.distance_to_AprilTag_Z_feet = 3.28 * self.distance_to_AprilTag_Z_meters

            self.pose_of_AprilTag_roll_degrees    = self.aprilTag_position_and_pose.rotation().x_degrees
            self.pose_of_AprilTag_pitch_degrees   = self.aprilTag_position_and_pose.rotation().y_degrees
            self.pose_of_AprilTag_yaw_degrees_raw = self.aprilTag_position_and_pose.rotation().z_degrees

            self.pose_of_AprilTag_yaw_degrees = 0.0
            # Change the Yaw to be an offset from being perpendicular to the axis to the robot
            if (self.pose_of_AprilTag_yaw_degrees_raw > 0):
                self.pose_of_AprilTag_yaw_degrees = 180 - self.pose_of_AprilTag_yaw_degrees_raw
            elif (self.pose_of_AprilTag_yaw_degrees_raw < 0): 
                self.pose_of_AprilTag_yaw_degrees = 0  - (self.pose_of_AprilTag_yaw_degrees_raw + 180)

        # [3] Calculate the direct distance from the robot to the center of the AprilTag [Pythagorean theorem]

            self.distance_to_AprilTag_meters = math.sqrt(  math.pow(self.distance_to_AprilTag_X_meters, 2) +  
                                                           math.pow(self.distance_to_AprilTag_Y_meters, 2) )


        # [4]] Calculate the side lengths and angles within an obtuse triange formed by the following 3 points [Field-Oriented]
        #    (P) Current position of the robot  (provided by pose) 
        #    (Q) Center of the AprilTag  
        #    (G) turn-point for robot to stop movement

        # The angles within this obtuse triange are determined as follows:
        #    Angle A (Opposite side a) - ArcSin((side a)/(side c) * Sin (Angle C))
        #    Angle B (Opposite side b) - ArcSin((side a)/(side b) * Sin (Angle B))
        #    Angle C (Opposite side c) - Angle Yaw + Angle Target (rotation) (OR (not sure)) Angle Yaw - Angle Target (rotation) 


        # The length of the sides are determined as follows:
        #    Distance between (P) and (Q) - (side a) - Robot to AprilTag - Provided by PhotonVision [Pythagorean theorem]
        #    Distance between (Q) and (G) - (side b) - 140% * length of side of robot - 42"  = 1.08 meters
        #    Distance between (G) and (P) - (side c) - To be Calculated using Law of Cosines as follows:

        #    Side c = SQRT[ (side a)^2 + (side b)^2 - 2 * (side a) * (side b) *  COS (Angle C) ]

        ##   Acquire and Calculate  the  yaw of the AprilTag relative to the robot (relative to the robot beam )
            # Change the Yaw to be an offset from being perpendicular to the axis to the robot

            # Calculate the sides and angles of the obtuse alignment triangle

            self.alignmentTriangle_Angle_C_degrees = self.apriltag_alignment_data.apriltag_yaw + self.pose_of_AprilTag_yaw_degrees
            self.alignmentTriangle_Angle_C_radians = self.alignmentTriangle_Angle_C_degrees * math.pi  / 180
            # print (f"alignmentTriangle_Angle_C_degrees {self.alignmentTriangle_Angle_C_degrees} ")

            self.alignmentTriangle_side_a_meters = self.distance_to_AprilTag_meters
            self.alignmentTriangle_side_b_meters = 1.08  # Offset distance   # TODO Need to make a Constant
            self.alignmentTriangle_side_c_meters = math.sqrt(  math.pow(self.alignmentTriangle_side_a_meters, 2 ) + 
                                                             math.pow(self.alignmentTriangle_side_b_meters, 2 ) - 
                                                             2 * self.alignmentTriangle_side_a_meters * self.alignmentTriangle_side_b_meters *
                                                             math.cos(self.alignmentTriangle_Angle_C_radians))
            
        #    Angle A (Opposite side a) - ArcSin((side a)/(side c) * Sin (Angle C))
        #    Angle B (Opposite side b) - ArcSin((side a)/(side b) * Sin (Angle B))
        #    Angle A should be greater than 90 so took complement (180 - angle)  Pi in radians

            self.alignmentTriangle_Angle_A_radians = math.pi - (math.asin ((self.alignmentTriangle_side_a_meters/self.alignmentTriangle_side_c_meters) 
                                                      * math.sin(self.alignmentTriangle_Angle_C_radians)))
            self.alignmentTriangle_Angle_B_radians = (math.asin ((self.alignmentTriangle_side_b_meters/self.alignmentTriangle_side_a_meters) 
                                                      * math.sin(self.alignmentTriangle_Angle_A_radians)))

        # [5] Calculate the turn-point position in robot centric terms (delta-x, delta-y)
            self.drive_to_turnpoint_angle_radians = (self.apriltag_alignment_data.apriltag_yaw * math.pi / 180 )+ self.alignmentTriangle_Angle_B_radians
            self.drive_to_turnpoint_X_component_meters = math.cos(self.drive_to_turnpoint_angle_radians)
            self.drive_to_turnpoint_Y_component_meters = math.sin(self.drive_to_turnpoint_angle_radians)

            if (True):
                print(f"Robot initial Pose: X: {self.initial_translation.x:4.1f} Y: {self.initial_translation.y:4.1f} ", end='')
                print(f" heading: {self.initial_heading_degrees} Degrees")
                print(f"Raw AprilTag Pose {self.aprilTag_position_and_pose}")
                print(f"AprilTag Position: X: {self.distance_to_AprilTag_X_meters:4.1f} Y: {self.distance_to_AprilTag_Y_meters:4.1f} ", end='')
                print(f"AprilTag Yaw {self.pose_of_AprilTag_yaw_degrees:4.2f}  Distance: {self.distance_to_AprilTag_meters:4.1f}")
                print(f"Obtuse Triange Angles: A {(57.3 * self.alignmentTriangle_Angle_A_radians):5.1f} ", end='')
                print(f" B {(57.3 * self.alignmentTriangle_Angle_B_radians):5.1f} ", end='')
                print(f" C {(57.3 * self.alignmentTriangle_Angle_C_radians):5.1f} ")
                print(f"Obtuse Triange sides: a: {self.alignmentTriangle_side_a_meters:4.1f}   ", end='')
                print(f" b: {self.alignmentTriangle_side_b_meters:4.1f}   ", end='')
                print(f" c: {self.alignmentTriangle_side_c_meters:4.1f}   ")
                print(f"Turn-Point:  Drive Angle: {( 180 / math.pi * self.drive_to_turnpoint_angle_radians):4.1f} ", end='')
                print(f"  X-distance: {self.drive_to_turnpoint_X_component_meters:4.1f}", end='')
                print(f"  Y-distance: {self.drive_to_turnpoint_Y_component_meters:4.1f}")

        # [6] Place the data into the AprilTagAlignmentData data object

            self.apriltag_alignment_data.set_apriltag_turnpoint_position (self.drive_to_turnpoint_X_component_meters, 
                                                                         self.drive_to_turnpoint_Y_component_meters)
            self.apriltag_alignment_data.print_apriltag_alignment_turn_point_data()
            self.set_apriltag_turnpoint_angle_degrees(self.pose_of_AprilTag_yaw_degrees)


        print("Done: >>>>>  AprilTag Aligment Mode with Offset (AprilTagWithOffsetAligmentCalculation) <<<<<<<<<<<")

    def execute(self) -> None:
        pass
 
    def isFinished(self) -> bool:       
        return True        
 
    def end(self, interrupted: bool) -> None:
        pass
 
