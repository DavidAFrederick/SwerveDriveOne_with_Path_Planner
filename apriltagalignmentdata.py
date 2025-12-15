from wpimath.geometry import Translation2d, Transform2d, Transform3d, Translation3d, Rotation3d
import math


class AprilTagAlignmentData:
    def __init__(self) -> None:
            # Initialize the return variables
            # >>>> Distances are in meters
            # >>>> Yaw Angles are in Degrees (as it comes from PhotonVision)
        self.apriltag_present = False
        self.apriltag_yaw = 0
        self.apriltag_skew = 0
        self.apriltag_distance = 0
        self.apriltag_bestCameraToTarget : Transform3d = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0, 0, 0))
                    # This data structure is the offset to the AprilTag and the rotation of the Apriltag relative to the robot
                    # Angles in Rotation3d are provided in Radians

        self.apriltag_turnpoint_position : Translation2d = Translation2d(0.0, 0.0)  #  forward distance, cross distance in meters
        self.apriltag_turnpoint_angle_degrees : float = 0.0   # Rotation of the AprilTag about it's z axis

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

    def set_apriltag_alignment_data_yaw(self, yaw : float):    ## Degrees
        self.apriltag_present = True
        self.apriltag_yaw = yaw

        #  AprilTag Yaw is Positive when the target is left of camera centers as viewed in the image

    def set_apriltag_bestCameraToTarget(self, Transform3dData : Transform3d): 
        self.apriltag_present = True
        self.apriltag_bestCameraToTarget = Transform3dData
        forward_distance = self.apriltag_bestCameraToTarget.translation().X()
        cross_distance   = self.apriltag_bestCameraToTarget.translation().Y()
        if (forward_distance != 0):     ## TODO ERROR - Need to figure out how to set positive or negatve
            self.apriltag_yaw = 180/math.pi *  math.atan(cross_distance/forward_distance)
        else:
            self.apriltag_yaw = 0.0   ## Degrees
        self.apriltag_turnpoint_angle_degrees = self.apriltag_bestCameraToTarget.rotation().z_degrees() # Apriltag's yaw

    def set_apriltag_bestCameraToTarget_TEST(self):              # Meters  (x,y,z)    Radians (roll, pitch, yaw)  (9 degrees)
        self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.59, -0.96, 0.0), Rotation3d(0, 0, (-10 * math.pi/180)))
        # self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.59, -0.96, 0.0), Rotation3d(0, 0, (32 * math.pi/180)))
        ### TODO Determine the coordinate system for the bestCameraToTarget
        self.apriltag_present = True
        forward_distance = self.apriltag_bestCameraToTarget.translation().X()
        cross_distance   = self.apriltag_bestCameraToTarget.translation().Y()
        if (forward_distance != 0):   ## TODO - ERROR - See above
            self.apriltag_yaw = 180/math.pi *  math.atan(cross_distance/forward_distance)
        else:
            self.apriltag_yaw = 0.0   ## Degrees
        self.apriltag_turnpoint_angle_degrees = self.apriltag_bestCameraToTarget.rotation().z_degrees # Apriltag's yaw

    def set_apriltag_turnpoint_position (self, forward_position_meters : float, cross_position_meters : float):
        self.apriltag_turnpoint_position = Translation2d(forward_position_meters, cross_position_meters) 

    def set_apriltag_turnpoint_angle_degrees (self, apriltag_turnpoint_angle_degrees : float):
        self.apriltag_turnpoint_angle_degrees = apriltag_turnpoint_angle_degrees

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
        self.apriltag_distance]
        return data_list
    
    def get_apriltag_bestCameraToTarget(self) -> Transform3d: 
        return self.apriltag_bestCameraToTarget

    def get_apriltag_turnpoint_position_meters(self) -> Translation2d:
        print (f"Getting: self.apriltag_turnpoint_position {self.apriltag_turnpoint_position}")
        return self.apriltag_turnpoint_position


    def get_apriltag_turnpoint_X_position_meters(self) -> float:
        return self.apriltag_turnpoint_position.X()

    def get_apriltag_turnpoint_Y_position_meters(self) -> float:
        return self.apriltag_turnpoint_position.Y()

    def get_apriltag_turnpoint_angle_degrees(self) -> float:
        # print (f"Getting AprilTag Z-axis Yaw: {self.apriltag_turnpoint_angle_degrees:6.3f}")
        return self.apriltag_turnpoint_angle_degrees 



# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    def print_all_apriltag_alignment_data(self) -> None:
            print (f"Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}  ", end ='') 
            print (f"Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")

    def print_apriltag_alignment_turn_point_data(self) -> None:
            print (f"Turn Point (Robot-Centric): X: {self.apriltag_turnpoint_position[0]:5.2f}  ", end='') 
            print (f"  Y: {self.apriltag_turnpoint_position[1]:5.2f}  ") 
