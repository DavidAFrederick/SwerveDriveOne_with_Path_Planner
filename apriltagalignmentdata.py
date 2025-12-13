from wpimath.geometry import Transform3d, Translation3d, Rotation3d
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
        self.apriltag_bestCameraToTarget = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0, 0, 0))
                    # This data structure is the offset to the AprilTag and the rotation of the Apriltag relative to the robot
                    # Angles in Rotation3d are provided in Radians

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

    def set_apriltag_alignment_data_yaw(self, yaw : float): 
        self.apriltag_present = True
        self.apriltag_yaw = yaw

    def set_apriltag_bestCameraToTarget(self, Transform3dData : Transform3d): 
        self.apriltag_bestCameraToTarget = Transform3dData


    def set_apriltag_bestCameraToTarget_TEST(self):              # Meters  (x,y,z)    Radians (roll, pitch, yaw)  (9 degrees)
        self.apriltag_bestCameraToTarget = Transform3d(Translation3d(2.0, 0.6, 0.0), Rotation3d(0, 0, (math.pi - 0.165)))
        self.apriltag_present = True
        self.apriltag_yaw = 5.0          # Degrees

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    def get_apriltag_alignment_data_Target_present(self) -> bool: 
        return self.apriltag_present
        
    def get_apriltag_alignment_data_yaw(self) -> float: 
        return self.apriltag_yaw

    def get_all_apriltag_alignment_data(self) -> list:
        data_list = [
        self.apriltag_present,
        self.apriltag_yaw,
        self.apriltag_skew,
        self.apriltag_distance]
        return data_list
    
    def print_all_apriltag_alignment_data(self) -> None:
            print (f"Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}  ", end ='') 
            print (f"Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")

    def get_apriltag_bestCameraToTarget(self) -> Transform3d: 
        return self.apriltag_bestCameraToTarget



