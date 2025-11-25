class AprilTagAlignmentData:
    def __init__(self) -> None:
            # Initialize the return variables
        self.apriltag_present = False
        self.apriltag_yaw = 0
        self.apriltag_skew = 0
        self.apriltag_distance = 0

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

    def set_apriltag_alignment_data_yaw(self, yaw : FloatingPointError): 
        self.apriltag_present = True
        self.apriltag_yaw = yaw

    def get_all_apriltag_alignment_data(self) -> list:
        data_list = [
        self.apriltag_present,
        self.apriltag_yaw,
        self.apriltag_skew,
        self.apriltag_distance]
        return data_list
    
    def print_all_apriltag_alignment_data(self) -> None:
            print (f"Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}  Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")


