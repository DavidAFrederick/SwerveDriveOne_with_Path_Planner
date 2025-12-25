from wpimath.geometry import Pose2d
from commands2 import Command
from subsystems.vision_subsystem import VisionSystem 
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from apriltagalignmentdata import AprilTagAlignmentData
import math

class AprilTagWithOffsetAligmentCalculation(Command):
    """
    This command only does calculations to determine the location (called: turn-point) which will place the
     robot directly in front of the Apriltag.  The result is stored in the "AprilTagAlignmentData" data object.
    
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

        [4]] Calculate the side lengths and angles within an obtuse triangle formed by the following 3 points [Field-Oriented]
           (P) Current position of the robot  (provided by pose) 
           (Q) Center of the AprilTag  
           (G) turn-point for robot to stop movement  (X,Y Position we are calculating)

        The length of the sides are determined as follows:
           Distance between (P) and (Q) - (side a) - Robot to AprilTag - Provided by PhotonVision [Pythagorean theorem]
           Distance between (Q) and (G) - (side b) - 140% * length of side of robot - 42"  = 1.08 meters
           Distance between (G) and (P) - (side c) - To be Calculated using Law of Cosines as follows:

           Side c = SQRT[ (side a)^2 + (side b)^2 - 2 * (side a) * (side b) *  COS (Angle C) ]
 
        The angles within this obtuse triangle are determined as follows:
           Angle A (Opposite side a) - ArcSin((side a)/(side c) * Sin (Angle C))
           Angle B (Opposite side b) - ArcSin((side a)/(side b) * Sin (Angle B))
           Angle C (Opposite side c) - Angle Yaw + Angle Target (rotation) (OR (not sure)) Angle Yaw - Angle Target (rotation) 

        [5] Calculate the turn-point position in robot centric terms (delta-x, delta-y)

        [6] Place the data into the AprilTagAlignmentData data object

        Here is an example of what the PhotonVision returns when in 3D Mode(Values are Meters and Radians)
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


        ## To Enable testing without the photonVision Camera System:
        #  1) Disable / Comments out:  
        #     # self.vision.get_tag_data()
        #  2) Enable / Add code to create test vision data: 
        #     # self.apriltag_alignment_data.set_apriltag_bestCameraToTarget_TEST()
        #  3) In the RobotContainer, Disable the real vision subsystem and enable the Dummy Vision subsystem


        ## TODO Need to add a check to see  if data is available

        if (self.apriltag_alignment_data.get_test_mode()):   # True means use simulated data and NOT the photonvision
            ## Temporary tool to load in test data to calculation verification
            self.apriltag_alignment_data.set_apriltag_bestCameraToTarget_TEST() ### Set fake test data 

            self.apriltag_alignment_data.print_all_apriltag_alignment_data() # Print the test data
        else:
            self.vision.get_tag_data()          # Calls vision subsystem and placed AprilTag target data into "apriltagalignmentdata" data object

        
        if (self.apriltag_alignment_data.get_apriltag_alignment_data_Target_present()):     # Get the position and yaw of the AprilTag
            print ("++++++++++++++++++  Running calc ++++++++++++++++=")
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

            # PhotonVision provides the Yaw of the AprilTag itself within the Rotation part of the Transform3D.
            # If the Apriltag is perfectly perpendicular to the robot, a value of 180  is returned
            # If the left side of the Apriltag is away from the robot by 5 degrees (CCW from top), a positive value of 175 degrees is returned
            # If the right side of the Apriltag is away from the robot by 5 degrees (CW from top), a negative value of -175 degrees is returned

            # This value is converted to O degrees is perpendicular, left  side back is +5 degrees, right side back is -5 degrees
            # This value is used to align the robot to be perpendicular to the AprilTag at the end of motion

            # Change the Yaw to be an offset from being perpendicular to the axis to the robot
            if (self.pose_of_AprilTag_yaw_degrees_raw > 0):
                # self.pose_of_AprilTag_yaw_degrees =  self.pose_of_AprilTag_yaw_degrees_raw
                self.pose_of_AprilTag_yaw_degrees = 180 - self.pose_of_AprilTag_yaw_degrees_raw
            elif (self.pose_of_AprilTag_yaw_degrees_raw < 0): 
                self.pose_of_AprilTag_yaw_degrees = -(180 + self.pose_of_AprilTag_yaw_degrees_raw)
            else:
                self.pose_of_AprilTag_yaw_degrees = 0

        # [3] Calculate the direct distance from the robot to the center of the AprilTag [Pythagorean theorem]

            self.distance_to_AprilTag_meters = math.sqrt(  math.pow(self.distance_to_AprilTag_X_meters, 2) +  
                                                           math.pow(self.distance_to_AprilTag_Y_meters, 2) )

        # [4]] Calculate the side lengths and angles within an obtuse triangle formed by the following 3 points [Field-Oriented]
        #    (P) Current position of the robot  (provided by pose) 
        #    (Q) Center of the AprilTag  
        #    (G) turn-point for robot to stop movement

        # The angles within this obtuse triangle are determined as follows:
        #    Angle A (Opposite side a) - ArcSin((side a)/(side c) * Sin (Angle C))
        #    Angle B (Opposite side b) - ArcSin((side a)/(side b) * Sin (Angle B))
        #    Angle C (Opposite side c) - Angle Yaw + Angle Target (rotation) (OR (not sure)) Angle Yaw - Angle Target (rotation) 

        # The length of the sides are determined as follows:
        #    Distance between (P) and (Q) - (side a) - Robot to AprilTag - Provided by PhotonVision [Pythagorean theorem]
        #    Distance between (Q) and (G) - (side b) - 140% * length of side of robot - 42"  = 1.08 meters
        #    Distance between (G) and (P) - (side c) - To be Calculated using Law of Cosines as follows:

        #    Side c = SQRT[ (side a)^2 + (side b)^2 - 2 * (side a) * (side b) *  COS (Angle C) ]

        # Calculate the sides and angles of the obtuse alignment triangle

        # Heading Yaw: Robot Heading to AprilTag (Positive means April Tag is left of camera center)

            # Angle C is calculated using triangle rules.
            # Theta J (angle opposite of Robot-Tag Yaw) = 90 - Yaw
            # Theta C (in obtuse triangel) = Yaw + AprilTagYaw
            #       Found by saying 90 = Theta C + Theta J  - AprilTagYaw 

            # This angle is uncertain
            self.alignmentTriangle_Angle_C_degrees = abs(self.apriltag_alignment_data.apriltag_yaw) + self.pose_of_AprilTag_yaw_degrees
            ## ERROR - Does the code need absolute values
        ##  TODO:  Not 100% sure this is correct

            self.alignmentTriangle_Angle_C_radians = self.alignmentTriangle_Angle_C_degrees * math.pi  / 180

            self.alignmentTriangle_side_a_meters = self.distance_to_AprilTag_meters
            self.alignmentTriangle_side_b_meters = 1.08  # Offset distance   # TODO Need to make a Constant
            self.alignmentTriangle_side_c_meters = math.sqrt( math.pow(self.alignmentTriangle_side_a_meters, 2 ) + 
                                                              math.pow(self.alignmentTriangle_side_b_meters, 2 ) - 
                                                              2 * self.alignmentTriangle_side_a_meters * self.alignmentTriangle_side_b_meters *
                                                              math.cos(self.alignmentTriangle_Angle_C_radians))
            
        #    Angle A (Opposite side a) - ArcSin((side a)/(side c) * Sin (Angle C))
        #    Angle B (Opposite side b) - ArcSin((side a)/(side b) * Sin (Angle B))
        #    Angle A should be greater than 90 so took complement (180 - angle)  Pi in radians

            if (self.alignmentTriangle_side_c_meters != 0):
                self.alignmentTriangle_Angle_A_radians = math.pi - (math.asin ((self.alignmentTriangle_side_a_meters/self.alignmentTriangle_side_c_meters) 
                                                        * math.sin(self.alignmentTriangle_Angle_C_radians)))
            else:
                self.alignmentTriangle_Angle_A_radians = 2 * math.pi   #  Not sure if this is correct
                print ("Divide by zero protection (1) !!!!!!!!!!!!!!!!!!!!!!")
            
            if (self.alignmentTriangle_side_a_meters != 0):
                self.alignmentTriangle_Angle_B_radians = (math.asin ((self.alignmentTriangle_side_b_meters/self.alignmentTriangle_side_a_meters) 
                                                        * math.sin(self.alignmentTriangle_Angle_A_radians)))
            else:
                self.alignmentTriangle_Angle_B_radians = 0 # ??????
                print ("Divide by zero protection (2) !!!!!!!!!!!!!!!!!!!!!!")

        # [5] Calculate the turn-point position in robot centric terms (delta-x, delta-y)
                ### TODO - ERROR HERE - Need to figure out to to calculate turnpoint angle
            self.drive_to_turnpoint_angle_radians = (self.apriltag_alignment_data.apriltag_yaw * math.pi / 180 ) + self.alignmentTriangle_Angle_B_radians
            self.drive_to_turnpoint_X_component_meters = self.alignmentTriangle_side_c_meters * math.cos(self.drive_to_turnpoint_angle_radians)
            self.drive_to_turnpoint_Y_component_meters = self.alignmentTriangle_side_c_meters * math.sin(self.drive_to_turnpoint_angle_radians)

            if (True):
                print("Robot:    -    -    -    -    -    -    -    -    -    -    -    -    -    -    -    -    -")
                print(f"Robot initial Pose: X: {self.initial_translation.x:6.3f} Y: {self.initial_translation.y:6.3f} ", end='')
                print(f" heading: {self.initial_heading_degrees:6.3} Degrees")
                print("Robot to AprilTag:  -    -    -    -    -    -    -    -    -    -    -    -    -    -    -")
                print(f"AprilTag Position: X: {self.distance_to_AprilTag_X_meters:6.3f} Y: {self.distance_to_AprilTag_Y_meters:6.3f}   ", end='')
                print(f"Yaw degrees of Apriltag off of robot heading (Positive Tag is left of camera center): ", end='')
                print(f"{self.apriltag_alignment_data.get_apriltag_alignment_data_yaw():6.3f}")
                print("AprilTag (itself): pose (position and rotation)   -    -    -    -    -    -    -    -    -")
                print(f"Raw AprilTag Pose {self.aprilTag_position_and_pose}")
                print(f"AprilTag Position  (Meters): X: {self.distance_to_AprilTag_X_meters:6.3f} Y: {self.distance_to_AprilTag_Y_meters:6.3f}   ")
                print(f"Apriltag Rotation Yaw: (positive means left side of Apriltag is away from robot):  ", end='')
                print(f"Raw Value: {self.pose_of_AprilTag_yaw_degrees_raw:6.3f}  ", end ='')
                print(f"Converted Value: {self.pose_of_AprilTag_yaw_degrees:6.3f}")
                print("Alignment Triangle: -    -    -    -    -    -    -    -    -    -    -    -    -    -    -")
                print(f"Obtuse Triangle Angles (Degrees): A {(57.3 * self.alignmentTriangle_Angle_A_radians):6.3f} ", end='')
                print(f" B {(57.3 * self.alignmentTriangle_Angle_B_radians):6.3f} ", end='')
                print(f" C {(57.3 * self.alignmentTriangle_Angle_C_radians):6.3f} ")
                print(f"Obtuse Triangle sides (Meters): a: {self.alignmentTriangle_side_a_meters:6.3f}   ", end='')
                print(f" b: {self.alignmentTriangle_side_b_meters:6.3f} ", end='')
                print(f" c: {self.alignmentTriangle_side_c_meters:6.3f}   ")
                print(f"Turn-Point: Forward-distance: {self.drive_to_turnpoint_X_component_meters:6.3f}  ", end='')
                print(f"Cross-distance: (Positive to left)  {self.drive_to_turnpoint_Y_component_meters:6.3f}   ", end='')
                print(f"Drive Angle: {( 180 / math.pi * self.drive_to_turnpoint_angle_radians):6.3f} ")
                print("======(Alignment Triangle Verification)===============")
                print(f"Sin(X)/x: {math.sin(self.alignmentTriangle_Angle_A_radians)/self.alignmentTriangle_side_a_meters:6.3f}", end='')
                print(f" {math.sin(self.alignmentTriangle_Angle_B_radians)/self.alignmentTriangle_side_b_meters:6.3f}", end='')
                print(f" {math.sin(self.alignmentTriangle_Angle_C_radians)/self.alignmentTriangle_side_c_meters:6.3f}  << Should be the same")
                print(f"Angles: A: {(180/math.pi * self.alignmentTriangle_Angle_A_radians):6.3f}", end='')
                print(f" B: {(180/math.pi * self.alignmentTriangle_Angle_B_radians):6.3f}", end='')
                print(f" C: {(180/math.pi * self.alignmentTriangle_Angle_C_radians):6.3f}", end='')
                print(f"  Sum of angles: {(180/math.pi * (self.alignmentTriangle_Angle_A_radians + 
                                                        self.alignmentTriangle_Angle_B_radians + 
                                                        self.alignmentTriangle_Angle_C_radians)):6.3f}  << Should total 180 degrees")
                print("===================================================")

                #  Triangle Math Verification
                #  1) Sin(A)/side-a = Sin(B)/side-b = sin(C)/side-c
                #  2) Angle A + Angle B + Angle C = 180

        # [6] Place the data into the AprilTagAlignmentData data object

            self.apriltag_alignment_data.set_apriltag_turnpoint_position (self.drive_to_turnpoint_X_component_meters, 
                                                                         self.drive_to_turnpoint_Y_component_meters)
            # self.apriltag_alignment_data.print_apriltag_alignment_turn_point_data()
            self.apriltag_alignment_data.set_apriltag_turnpoint_angle_degrees(self.pose_of_AprilTag_yaw_degrees)
            self.apriltag_alignment_data.print_apriltag_alignment_turn_point_data()

        else:
            print ("---------- NOT -----  Running calc ++++++++++++++++=")


    def execute(self) -> None:
        pass
 
    def isFinished(self) -> bool:       
        return True        
 
    def end(self, interrupted: bool) -> None:
        print("Done: >>>>>  AprilTag Aligment Mode with Offset (AprilTagWithOffsetAligmentCalculation) <<<<<<<<<<<")
        pass
 
