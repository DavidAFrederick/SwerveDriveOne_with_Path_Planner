from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from photonlibpy.photonCamera import (
    PhotonCamera,
    PhotonPipelineResult,
)

from apriltagalignmentdata import AprilTagAlignmentData

#  Notes on Reading AprilTag 3D data
#  from networktables import NetworkTables

# As a client to connect to a robot
# Replace 'roborio-XXX-frc.local' with your robot's actual hostname or IP address
# (e.g., '10.XX.YY.2' where XXYY is your team number)
# NetworkTables.initialize(server='roborio-YOURTEAMNUMBER-frc.local')  
# 
# sd = NetworkTables.getTable('SmartDashboard')
# # Read a number
# someNumber = sd.getNumber('someNumber', 0.0)  # 0.0 is the default value

# # Read a string
# message = sd.getString('robotMessage', 'No message yet')

# # Read a boolean
# isRobotEnabled = sd.getBoolean('robotEnabled', False)

# # Read an array of numbers
# sensorReadings = sd.getNumberArray('sensorData', []) # Empty list as default



class VisionSystem(Subsystem):
    """
    This Vision Subsystem only supports the close alignment of the robot to a near-field April Tag.
    The subsystem returns the following results in a data class:
    - AprilTag Present
    - Yaw
    - Skew
    - Distance

    """    
    def __init__(self, apriltag_alignment_data : AprilTagAlignmentData) -> None:
        # Run the constructor for the subsystem class
        super().__init__()
        self.apriltag_alignment_data = apriltag_alignment_data

        # Initialize the camera and the results variable
        self._camera: PhotonCamera = PhotonCamera("OV9281")
        self._latest_result: PhotonPipelineResult = PhotonPipelineResult()

        # Get current results from the camera
        self._latest_result = self._camera.getLatestResult()

        self.apriltag_alignment_data.set_apriltag_alignment_data_yaw(5.0)  # Temp for test


    def periodic(self) -> None:             ## Runs 50 times per second
        # if RobotBase.isSimulation():      ## Disabling to allow hardware in the loop simulation testing
        #     # Don't do anything in sim

        #     return

        if self._camera is not None:
            # print ("CAMERA PRESENT", self._latest_result)
            self._latest_result = self._camera.getLatestResult()

    def get_tag_data(self) -> list:
        target_list = self._latest_result.getTargets()
        print (f"get_tag_data - Length:  {len(target_list)}")


        for target in target_list:
            self.fiducialId = target.getFiducialId()

            # Get the Transform3d from the camera to the target
            self.bestCameraToTarget: Transform3d = target.getBestCameraToTarget()

            # Print or use the transform data
            print(f"Target ID: {self.fiducialId}")
            print(f"Best Camera to Target Transform: {self.bestCameraToTarget}")


        if len(target_list) > 0:   #  No targets present
        # if (self._latest_result.hasTargets()):
            print (f"vvvvvvvvvvvvvvvvvvvvvvvvvvv")
            print (f" Target_list {target_list}")
            print (f"{len(target_list)} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ")

            for target in target_list:
                print (f"target.getYaw()  {target.getYaw()}")
                self.apriltag_alignment_data.set_apriltag_alignment_data_yaw(target.getYaw())
        else:
            self.apriltag_alignment_data.set_apriltag_alignment_data_not_available()

        SmartDashboard.putBoolean("See Tag", self._latest_result.hasTargets())

        self.apriltag_alignment_data.print_all_apriltag_alignment_data()

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


            print ("====================================")
            print (f"self.translation {self.translation}")
            print ("- - - - - - - - - - - - - - - - - - ")
            print (f"self.rotation {self.rotation}")
            print ("====================================")





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

# CAMERA PRESENT PhotonPipelineResult(ntReceiveTimestampMicros=22788945, targets=[PhotonTrackedTarget(yaw=-17.435458193921075, pitch=10.952675124429012, area=5.981770833333333, skew=0.0, fiducialId=1, bestCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), Rotation3d(x=0.000000, y=0.000000, z=0.000000)), altCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), Rotation3d(x=0.000000, y=0.000000, z=0.000000)), minAreaRectCorners=[TargetCorner(x=22.000000267373686, y=121.00000140655237), TargetCorner(x=26.00000025808989, y=50.00000122088267), TargetCorner(x=93.89717075313413, y=53.82519390594763), TargetCorner(x=89.89717076241791, y=124.82519409161733)], detectedCorners=[TargetCorner(x=22.304798126220696, y=121.80617523193364), TargetCorner(x=90.86666107177734, y=123.26227569580077), TargetCorner(x=90.95269012451173, y=55.14942550659177), TargetCorner(x=26.32320404052734, y=50.29891586303711)], poseAmbiguity=-1.0, objDetectId=-1, objDetectConf=-1.0)], metadata=PhotonPipelineMetadata(captureTimestampMicros=22744805, publishTimestampMicros=22775687, sequenceID=131382, timeSinceLastPong=185966), multitagResult=None)
# CAMERA PRESENT


# CAMERA PRESENT PhotonPipelineResult(ntReceiveTimestampMicros=15380157, targets=[PhotonTrackedTarget(yaw=-17.435556427175996, pitch=10.947993721042003, area=5.981770833333333, skew=0.0, fiducialId=1, bestCameraToTarget=Transform3d(Translation3d(x=0.692110, y=0.219953, z=0.141846), Rotation3d(x=0.008514, y=-0.056553, z=-2.586896)), altCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), Rotation3d(x=0.000000, y=0.000000, z=-3.141593)), minAreaRectCorners=[TargetCorner(x=22.000000267373686, y=121.00000140655237), TargetCorner(x=26.00000025808989, y=50.00000122088267), TargetCorner(x=93.89717075313413, y=53.82519390594763), TargetCorner(x=89.89717076241791, y=124.82519409161733)], detectedCorners=[TargetCorner(x=22.32546806335451, y=121.8176956176758), TargetCorner(x=90.84500885009767, y=123.26327514648436), TargetCorner(x=90.99295043945311, y=55.12897872924804), TargetCorner(x=26.279123306274403, y=50.28087234497071)], poseAmbiguity=0.0, objDetectId=-1, objDetectConf=-1.0)], metadata=PhotonPipelineMetadata(captureTimestampMicros=15339709, publishTimestampMicros=15371714, sequenceID=137318, timeSinceLastPong=406215), multitagResult=None)
# CAMERA PRESENT

# ==================

#  2D Capture
# CAMERA PRESENT 
# PhotonPipelineResult(ntReceiveTimestampMicros=22788945, 
# targets=[PhotonTrackedTarget(
#   yaw=-17.435458193921075, 
#   pitch=10.952675124429012, 
#   area=5.981770833333333, 
#   skew=0.0, fiducialId=1, 
# bestCameraToTarget=Transform3d(
#   Translation3d(x=0.000000, y=0.000000, z=0.000000), 
#   Rotation3d(x=0.000000, y=0.000000, z=0.000000)), 
# altCameraToTarget=Transform3d(
#   Translation3d(x=0.000000, y=0.000000, z=0.000000), 
#   Rotation3d(x=0.000000, y=0.000000, z=0.000000)), 
# minAreaRectCorners=[
#   TargetCorner(x=22.000000267373686, y=121.00000140655237), 
#   TargetCorner(x=26.00000025808989, y=50.00000122088267), 
#   TargetCorner(x=93.89717075313413, y=53.82519390594763), 
#   TargetCorner(x=89.89717076241791, y=124.82519409161733)], 
# detectedCorners=[
#   TargetCorner(x=22.304798126220696, y=121.80617523193364), 
#   TargetCorner(x=90.86666107177734, y=123.26227569580077), 
#   TargetCorner(x=90.95269012451173, y=55.14942550659177), 
#   TargetCorner(x=26.32320404052734, y=50.29891586303711)], 
#   poseAmbiguity=-1.0, objDetectId=-1, objDetectConf=-1.0)], 
# metadata=PhotonPipelineMetadata(captureTimestampMicros=22744805, 
# publishTimestampMicros=22775687, sequenceID=131382, timeSinceLastPong=185966), multitagResult=None)


#  3D Capture
# CAMERA PRESENT 
# PhotonPipelineResult(ntReceiveTimestampMicros=15380157, 
# targets=[PhotonTrackedTarget(
#   yaw=-17.435556427175996, 
#   pitch=10.947993721042003, 
#   area=5.981770833333333, 
#   skew=0.0, 
#   fiducialId=1, 
# bestCameraToTarget=Transform3d(
#   Translation3d(x=0.692110, y=0.219953, z=0.141846), 
#   Rotation3d(x=0.008514, y=-0.056553, z=-2.586896)), 
# altCameraToTarget=Transform3d(
#   Translation3d(x=0.000000, y=0.000000, z=0.000000), 
#   Rotation3d(x=0.000000, y=0.000000, z=-3.141593)), 
# minAreaRectCorners=[
#   TargetCorner(x=22.000000267373686, y=121.00000140655237), 
#   TargetCorner(x=26.00000025808989, y=50.00000122088267), 
#   TargetCorner(x=93.89717075313413, y=53.82519390594763), 
#   TargetCorner(x=89.89717076241791, y=124.82519409161733)], 
# detectedCorners=[
#   TargetCorner(x=22.32546806335451, y=121.8176956176758), 
#   TargetCorner(x=90.84500885009767, y=123.26327514648436), 
#   TargetCorner(x=90.99295043945311, y=55.12897872924804), 
#   TargetCorner(x=26.279123306274403, y=50.28087234497071)], 
#   poseAmbiguity=0.0, objDetectId=-1, objDetectConf=-1.0)], 
# metadata=PhotonPipelineMetadata(captureTimestampMicros=15339709, 
# publishTimestampMicros=15371714, sequenceID=137318, timeSinceLastPong=406215), multitagResult=None)



# get_tag_data - Length:  1
# Target ID: 1
# Best Camera to Target Transform: Transform3d(Translation3d(x=0.691508, y=0.219795, z=0.141702), Rotation3d(x=0.008310, y=-0.060773, z=-2.588145))
# vvvvvvvvvvvvvvvvvvvvvvvvvvv
#  Target_list [PhotonTrackedTarget(yaw=-17.444449496362772, pitch=10.951484931252457, area=5.981770833333333, skew=0.0, fiducialId=1, bestCameraToTarget=Transform3d(Translation3d(x=0.691508, y=0.219795, z=0.141702), Rotation3d(x=0.008310, y=-0.060773, z=-2.588145)), altCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), Rotation3d(x=0.000000, y=0.000000, z=-3.141593)), minAreaRectCorners=[TargetCorner(x=22.000000267373686, y=121.00000140655237), TargetCorner(x=26.00000025808989, y=50.00000122088267), TargetCorner(x=93.89717075313413, y=53.82519390594763), TargetCorner(x=89.89717076241791, y=124.82519409161733)], detectedCorners=[TargetCorner(x=22.283584594726562, y=121.80010986328126), TargetCorner(x=90.9406509399414, y=123.34922027587889), TargetCorner(x=90.9338607788086, y=55.113624572753906), TargetCorner(x=26.247613906860348, y=50.33272171020508)], poseAmbiguity=0.0, objDetectId=-1, objDetectConf=-1.0)]
# 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# target.getYaw()  -17.444449496362772
# Tag Present:   1.00 Yaw: -17.44  Skew:   0.00 Distance:  0.00
# get_tag_data - Length:  1
# Target ID: 1
# Best Camera to Target Transform: Transform3d(Translation3d(x=0.691751, y=0.219873, z=0.141734), Rotation3d(x=0.008171, y=-0.059707, z=-2.587668))
# vvvvvvvvvvvvvvvvvvvvvvvvvvv
#  Target_list [PhotonTrackedTarget(yaw=-17.43960675782512, pitch=10.950058409408681, area=5.981770833333333, skew=0.0, fiducialId=1, bestCameraToTarget=Transform3d(Translation3d(x=0.691751, y=0.219873, z=0.141734), Rotation3d(x=0.008171, y=-0.059707, z=-2.587668)), altCameraToTarget=Transform3d(Translation3d(x=0.000000, y=0.000000, z=0.000000), Rotation3d(x=0.000000, y=0.000000, z=-3.141593)), minAreaRectCorners=[TargetCorner(x=22.000000267373686, y=121.00000140655237), TargetCorner(x=26.00000025808989, y=50.00000122088267), TargetCorner(x=93.89717075313413, y=53.82519390594763), TargetCorner(x=89.89717076241791, y=124.82519409161733)], detectedCorners=[TargetCorner(x=22.264122009277347, y=121.83138275146484), TargetCorner(x=90.9001693725586, y=123.30415344238281), TargetCorner(x=90.95113372802733, y=55.148963928222635), TargetCorner(x=26.2767448425293, y=50.32369232177735)], poseAmbiguity=0.0, objDetectId=-1, objDetectConf=-1.0)]
# 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# .89717076241791, y=124.82519409161733)], detectedCorners=[TargetCorner(x=22.264122009277347, y=121.83138275146484), TargetCorner(x=90.9001693725586, y=123.30415344238281), TargetCorner(x=90.95113372802733, y=55.148963928222635), TargetCorner(x=26.2767448425293, y=50.32369232177735)], poseAmbiguity=0.0, objDetectId=-1, objDetectConf=-1.0)]
# 1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



