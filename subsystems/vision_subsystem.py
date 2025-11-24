from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Transform3d, Translation3d, Rotation3d

from photonlibpy.photonCamera import (
    PhotonCamera,
    PhotonPipelineResult,
)

class VisionSystem(Subsystem):
    """
    This Vision Subsystem only supports the close alignment of the robot to a near-field April Tag.
    The subsystem returns the following results in a tuple:
    - AprilTag Present
    - Yaw
    - Skew
    - Distance

    """    
    def __init__(self) -> None:
        # Run the constructor for the subsystem class
        super().__init__()

        # Initialize the camera and the results variable
        self._camera: PhotonCamera = PhotonCamera("OV9281")
        self._latest_result: PhotonPipelineResult = PhotonPipelineResult()

        # Get current results from the camera
        self._latest_result = self._camera.getLatestResult()

        # Initialize the return variables
        self.apriltag_present = False
        self.apriltag_yaw = 0
        self.apriltag_skew = 0
        self.apriltag_distance = 0

    def periodic(self) -> None:             ## Runs 50 times per second
        if RobotBase.isSimulation():      ## Disabling to allow hardware in the loop simulation testing
            # Don't do anything in sim
            return

        if self._camera is not None:
            self._latest_result = self._camera.getLatestResult()


    def get_tag_data(self) -> tuple:
        target_list = self._latest_result.getTargets()

        # if len(target_list) == 0:   #  No targets present
        if (self._latest_result.hasTargets()):
            self.apriltag_present = False
            self.apriltag_yaw = 0
            self.apriltag_skew = 0
            self.apriltag_distance = 0

        else:
            print (f"vvvvvvvvvvvvvvvvvvvvvvvvvvv")
            print (f" Target_list {target_list}")
            print (f"{len(target_list)} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ")

            for target in target_list:
                print (f"target.getYaw()  {target.getYaw()}")
                self.apriltag_present = True
                self.apriltag_yaw = target.getYaw()
                self.apriltag_skew = 0
                self.apriltag_distance = 0

        SmartDashboard.putBoolean("See Tag", self.apriltag_present)
        print (f"Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}  Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")

        return self.apriltag_present, self.apriltag_yaw, self.apriltag_skew, self.apriltag_distance


    def get_tag_data_v2(self) -> float:

        if self._latest_result.hasTargets():
            # Accessing 2D information (e.g., yaw, pitch, area of the best target)
            best_target = self._latest_result.getBestTarget()
            self.yaw_2d = best_target.getYaw()
            self.pitch_2d = best_target.getPitch()
            self.area_2d = best_target.getArea()

            # Accessing 3D pose information (if 3D mode is enabled in PhotonVision)
            # The getBestTarget().getBestCameraToTarget() returns a Transform3d
            # representing the camera's pose relative to the target.
            camera_to_target_pose: Transform3d = best_target.getBestCameraToTarget()

            # You can then extract translation and rotation components
            self.translation: Translation3d = camera_to_target_pose.translation()
            self.rotation: Rotation3d = camera_to_target_pose.rotation()

            self.apriltag_present = True
            self.apriltag_yaw = 0
            self.apriltag_skew = 0
            self.apriltag_distance = 0

            print ("====================================")
            print (f"self.translation {self.translation}")
            print ("- - - - - - - - - - - - - - - - - - ")
            print (f"self.rotation {self.rotation}")
            print ("====================================")

            print (f"Tag Present: {self.apriltag_present:6.2f} Yaw: {self.apriltag_yaw:6.2f}   Skew: {self.apriltag_skew:6.2f} Distance:{self.apriltag_distance:6.2f}")




#===(Example results)==========================================
#  self._latest_result.getTargets()   
# [PhotonTrackedTarget( yaw=10.314490364269597, 
#                       pitch=8.781454398117983, 
#                       area=4.884765625, 
#                       skew=0.0, 
#                       fiducialId=1, 
#                       bestCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), 
#                       Rotation3d(x=0.000000, y=0.000000, z=0.000000)), 
#                       altCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), 
#                       Rotation3d(x=0.000000, y=0.000000, z=0.000000)), 
#                       minAreaRectCorners=[
#                           TargetCorner(x=179.99999993625107, y=128.99999827820096), 
#                           TargetCorner(x=180.9999997839388, y=67.00000181259198), 
#                           TargetCorner(x=243.00000006374893, y=68.00000172179904), 
#                           TargetCorner(x=242.0000002160612, y=129.99999818740804)], 
#                       detectedCorners=[
#                           TargetCorner(x=182.8006591796875, y=69.13839721679688), 
#                           TargetCorner(x=180.6760711669922, y=129.71650695800784), 
#                           TargetCorner(x=242.9111328125, y=130.98941040039062), 
#                           TargetCorner(x=243.5400848388672, y=68.06653594970703)], 
#                       poseAmbiguity=-1.0, 
#                       objDetectId=-1, 
#                       objDetectConf=-1.0)]
# ==========================================





