from wpimath.geometry import Translation2d, Transform2d, Transform3d, Translation3d, Rotation3d
import math


class AprilTagAlignmentData:
    def __init__(self) -> None:
            # Initialize the variables
            #  apriltag_present = Indicates if an AprilTag is visible with data available 
        self.apriltag_present = False
            #  apriltag_yaw = The angle in degrees between the heading of the Robot and the visible ApriTag
            #                 AprilTag Yaw is Positive when the target is left of camera centers as viewed in the image
        self.apriltag_yaw = 0
            #  apriltag_skew = Not used for AprilTag.  Verify and delete
        self.apriltag_skew = 0
            #  apriltag_distance = The distance in meters between the camera and the aprilTag
        self.apriltag_distance = 0
            #  apriltag_bestCameraToTarget = A complex data structure (Transform3D) make up of Translation3d and Rotation3d
            #       Translation3d is the X,Y,Z distance in meters between the Robot and AprilTag
            #       Rotation3d is the pitch, roll and yaw of the Apriltag itself relative to the robot coordinate system in radians
        self.apriltag_bestCameraToTarget : Transform3d = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0, 0, 0))
            #       Calculated results are stored here.
            #       The distance between the robot and the turn-point forward distance, cross distance in meters
            #       Forward distance - Forward is positive,  Cross distance: To the left is positive
        self.apriltag_turnpoint_position : Translation2d = Translation2d(0.0, 0.0)   
            #       Calculated results are stored here.
            #       The angle between the robot and the turn-point in degrees, Positive is CW
        self.apriltag_turnpoint_angle_degrees : float = 0.0   # Rotation of the AprilTag about it's z axis
            #       Turns on test mode:
            #       1) Static data created in this file is loading into data structures
            #       2) PhotonVision / Vision subsystem is not instantiated
        self.simulation_test_mode_enable = True


    def set_all_apriltag_alignment_data(self, all_data_list : list) -> None:
        self.apriltag_present = all_data_list[0]
        self.apriltag_yaw = all_data_list[1]
        self.apriltag_skew = all_data_list[2]
        self.apriltag_distance = all_data_list[3]

    def set_apriltag_alignment_data_not_available(self): 
        self.apriltag_present = False
        self.apriltag_yaw = 0
        self.apriltag_skew = 0
        self.apriltag_distance = 0
        self.apriltag_bestCameraToTarget : Transform3d = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0, 0, 0))
        self.apriltag_turnpoint_position : Translation2d = Translation2d(0.0, 0.0)
        self.apriltag_turnpoint_angle_degrees : float = 0.0


    def set_apriltag_alignment_data_yaw(self, yaw : float):    ## Degrees
        self.apriltag_present = True
        self.apriltag_yaw = yaw

        #  AprilTag Yaw is Positive when the target is left of camera centers as viewed in the image

    def set_apriltag_bestCameraToTarget(self, Transform3dData : Transform3d): 
        self.apriltag_present = True
        self.apriltag_bestCameraToTarget = Transform3dData



    def set_apriltag_bestCameraToTarget_TEST(self):              # Meters  (x,y,z)    Radians (roll, pitch, yaw)  (9 degrees)
        """
        Places test data into the data structure in place of vision subsystem
        """
        self.apriltag_present = True
        # (positive means left side of Apriltag is away from robot)
        # Translations X, Y, Z are distances in meters,  Positive is Forward, left, up
        # Rotation3D is AprilTag Roll, Pitch, Yaw in radians.  Yaw Positive is CW (left side Away from robot)

        # Test Sets:
        # self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.0, -0.6, 0.0), Rotation3d(0, 0, (165 * math.pi/180)))
        # self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.0, 0.6, 0.0), Rotation3d(0, 0, (165 * math.pi/180)))
        # self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.59, -0.96, 0.0), Rotation3d(0, 0, (-170 * math.pi/180)))
        # self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.59, -0.96, 0.0), Rotation3d(0, 0, (32 * math.pi/180)))
        self.apriltag_bestCameraToTarget = Transform3d(Translation3d(3.26, 0.82, 0.03), Rotation3d(0, 0, (-3.1)))
        #### >>>  The simulation uses (3.26, -0.82) to get negative Yaw
        #### >>>  Real robot reports positive values (3.26, 0.82) and creates a negative yaw
        # >>  Rotation angle should be close to 180.  

        #  Calculating robot to AprilTag Yaw here so results are consistent

        forward_distance = self.apriltag_bestCameraToTarget.translation().X()
        cross_distance   = self.apriltag_bestCameraToTarget.translation().Y()

        if (forward_distance != 0): 
            self.apriltag_yaw = 180/math.pi *  math.atan(cross_distance/forward_distance)
        else:
            self.apriltag_yaw = 0.0   ## Degrees
        
        self.apriltag_yaw = - self.apriltag_yaw  #   Not sure why this is needed. (Adjusting for difference between simulation and real robot)

        print(f">>> forward_distance: {forward_distance:6.3f}")
        print(f">>> cross_distance: {cross_distance:6.3f}")
        print(f">>> self.apriltag_yaw: {self.apriltag_yaw:6.3f}")


    def set_apriltag_turnpoint_position (self, forward_position_meters : float, cross_position_meters : float):
        """ Calculation Result:  This method is used to transfer results between the calculation command and the movement commands. """
        self.apriltag_turnpoint_position = Translation2d(forward_position_meters, cross_position_meters) 

    def set_apriltag_turnpoint_angle_degrees (self, apriltag_turnpoint_angle_degrees : float):
        """ Calculation Result:  This method is used to transfer results between the calculation command and the movement commands. """
        self.apriltag_turnpoint_angle_degrees = apriltag_turnpoint_angle_degrees

    def set_test_mode (self, test_mode : bool):
            """
            Enable test mode for vision system (True means use fixed position data and disable vision subsystem)
            """
            self.simulation_test_mode_enable = test_mode

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    def get_apriltag_alignment_data_Target_present(self) -> bool: 
        return self.apriltag_present
        
    def get_apriltag_alignment_data_yaw(self) -> float: 
        return self.apriltag_yaw
        #  AprilTag Yaw is Positive when the target is left of camera centers as viewed in the image

    def get_all_apriltag_alignment_data(self) -> list:
        data_list = [
        self.apriltag_present,
        self.apriltag_yaw,
        self.apriltag_skew,
        self.apriltag_distance,
        self.apriltag_bestCameraToTarget,
        self.apriltag_turnpoint_position,
        self.apriltag_turnpoint_angle_degrees,        
        ]
        return data_list
    
    def get_apriltag_bestCameraToTarget(self) -> Transform3d: 
        return self.apriltag_bestCameraToTarget

    def get_apriltag_turnpoint_position_meters(self) -> Translation2d:
        print (f">>> Getting: self.apriltag_turnpoint_position {self.apriltag_turnpoint_position}")
        return self.apriltag_turnpoint_position

    def get_apriltag_turnpoint_X_position_meters(self) -> float:
        return self.apriltag_turnpoint_position.X()

    def get_apriltag_turnpoint_Y_position_meters(self) -> float:
        return self.apriltag_turnpoint_position.Y()

    def get_apriltag_turnpoint_angle_degrees(self) -> float:
        # print (f">>> Getting AprilTag Z-axis Yaw: {self.apriltag_turnpoint_angle_degrees:6.3f}")
        return self.apriltag_turnpoint_angle_degrees 

    def get_test_mode (self) -> bool:   #  True means in Test Mode ( Don't use the camera and get static data from this file)
        return self.simulation_test_mode_enable

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    def print_all_apriltag_alignment_data(self) -> None:
            print(f">>> Test Data ==========================================================")
            print(f">>> Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}  ", end ='') 
            print(f">>> Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")
            print(f">>> apriltag_bestCameraToTarget Transform3d: {self.apriltag_bestCameraToTarget}")
            print(f">>> Turn-Point: X,Y: {self.apriltag_turnpoint_position}  Angle: (Deg): {self.apriltag_turnpoint_angle_degrees:6.3f}")
            print(f">>> ====================================================================")

    def print_apriltag_alignment_turn_point_data(self) -> None:
            print(f">>> Turn Point (Robot-Centric): X: {self.apriltag_turnpoint_position[0]:5.2f}  ", end='') 
            print(f">>>   Y: {self.apriltag_turnpoint_position[1]:5.2f}  ") 
