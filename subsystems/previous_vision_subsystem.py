from commands2 import Subsystem, Command, cmd
from wpimath.geometry import Transform3d, Transform2d, Pose2d, Pose3d
from wpimath.units import feetToMeters
from photonlibpy.photonCamera import (
    PhotonCamera,
    setVersionCheckEnabled,
    PhotonPipelineResult,
)

# https://docs.photonvision.org/en/v2026.0.0-alpha-2/docs/programming/photonlib/getting-target-data.html
# Used in JAVA

# from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import RobotBase, DriverStation, SmartDashboard
from typing import List, Dict, Optional

"""
DF: Pulled from Crescendo and removed Note Detection code

Setup Notes:
Laptop should be set at 10.18.95.2 so the PhotonVision can publish data to the Network Tables
The PhotonVision CoProcessor should be set at 10.18.95.11.   Default value for photonvision

We can see the Photon Vision published data on the Network Tables display of the Simulation


"""


import os

# RED_SPEAKER_TAG = 4
# RED_AMP_TAG = 5
# BLUE_AMP_TAG = 6
# BLUE_SPEAKER_TAG = 7
TAG_NONE = 100


class CameraPoseEstimate:
    def __init__(self, obs_time: float, pose_estimate: Pose2d) -> None:
        self._timestamp: float = obs_time
        self._estimate: Pose2d = Pose2d


class TagMetaData:
    def __init__(self, id: int, yaw: float):
        self._id = id
        self._yaw = yaw


class VisionSystem(Subsystem):
    """
    Class to manage the various cameras and their targeting on the Robot
    """

    # def __init__(self, init_april_tag: bool, init_note_detection: bool) -> None:
    def __init__(self) -> None:
        super().__init__()

        # Value to track whether or not we've gotten good information from the camera
        self._timeout_in_seconds = 1   ## DF:  Existing Code Review Comment:    Not used anywhere - REMOVE

        self._tag_camera = SimpleTagDetectionPhotonCamera("OV9281")
        # # Intialize cameras to None and instantiate them if they should be used    ## DF: Might use later
        # if RobotBase.isSimulation():
        #     self._tag_camera = self._note_camera = None
        # else:
        #     self._tag_camera = SimpleTagDetectionPhotonCamera("OV9281")

        # Keep track of a given tag
        self.__tag_id = TAG_NONE      ## DF:  Existing Code Review Comment:   NOT USED ANYMORE - REMOVE

    def periodic(self) -> None:             ## Runs 50 times per second
        # if RobotBase.isSimulation():      ## Disabling to allow hardware in the loop simulation testing
        #     # Don't do anything in sim
        #     return

        # Update the camera results
        if self._tag_camera is not None:
            self._tag_camera.update_camera_results()

        SmartDashboard.putBoolean("See Tag", self.has_desired_tag_in_sight())

    def get_tag_yaw(self) -> float:
        return 1000 if self._tag_camera is None else self._tag_camera.get_tag_yaw()

    # def get_pose_estimates(self) -> List[CameraPoseEstimate]:
    #     if self._tag_camera is not None:
    #         return self._tag_camera.get_pose_estimates()

    # def set_target_tag(self, position: ShooterPosition) -> None:
    def set_target_tag(self) -> None:

        # if (
        #     position == ShooterPosition.SUBWOOFER_1
        #     or position == ShooterPosition.SUBWOOFER_2
        #     or position == ShooterPosition.SUBWOOFER_3
        # ):
        #     # Set according to blue or red
        #     if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #         self.__tag_id = BLUE_SPEAKER_TAG
        #     elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        #         self.__tag_id = RED_SPEAKER_TAG

        # if position == ShooterPosition.AMP:
        #     # Set according to blue or red
        #     if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #         self.__tag_id = BLUE_AMP_TAG
        #     elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        #         self.__tag_id = RED_SPEAKER_TAG

        # self._tag_camera.set_desired_target_id(self.__tag_id)
        self._tag_camera.set_desired_target_id(1)     ##  DF Temp setting to Apriltag #1

    def __get_tag_metadata(self, tag_id: int) -> Optional[TagMetaData]:
        if self._tag_camera is not None:
            return self._tag_camera._tag_metadatas[tag_id]

    def has_desired_tag_in_sight(self) -> bool:
        if self._tag_camera is not None:
            return self._tag_camera.get_tag_yaw() != 1000
        else:
            return False


class AprilTagPhotonCamera:
    def __init__(self, name: str, center_offset: Transform3d) -> None:
        setVersionCheckEnabled(False)

        self._camera: PhotonCamera = PhotonCamera(name)
        self._center_offset: Transform3d = center_offset

        self._pose_estimates: List = []

        self._tag_metadatas: Dict = {}

        tagPath = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "deploy", "2024-crescendo.json")
        )
        self._fieldTagLayout: AprilTagFieldLayout = AprilTagFieldLayout(tagPath)

    def update_camera_results(self, previousPoseEstimate: Pose2d) -> None:
        # Grab the latest results from the camera. A result suggests that a camera frame
        # was processed, not that it found something.  We need to filter on the results
        # to get the best pose estimate
        result: PhotonPipelineResult = self._camera.getLatestResult()
        result_time: float = result.getTimestamp()

        # Update the dashboard to let drivers know we're functional

        # Clear the old values
        self._pose_estimates.clear()
        self._tag_metadatas.clear()

        # Process each PhotonTrackedTarget
        # Use this as a basis for target selection:
        # https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html#d-to-3d-ambiguity
        # Use your odometry history from all sensors to pick the pose closest to where you expect the robot to be.
        # Reject poses which are very unlikely (ex: outside the field perimeter, or up in the air)
        # Ignore pose estimates which are very close together (and hard to differentiate)
        # Use multiple cameras to look at the same target, such that at least one camera can generate a good pose estimate
        # Look at many targets at once, using each to generate multiple pose estimates. Discard the outlying estimates, use the ones which are tightly clustered together.
        
        for target in result.getTargets():
            target_ID = target.getFiducialId()

            # Make sure that we've gotten an actual april tag, should be numbered
            if target_ID >= 0:
                fieldPose: Pose3d = self._fieldTagLayout.getTagPose(target_ID)
                if fieldPose is not None:
                    candidates = []
                    # Add the primary result to the list first, then add alternates
                    candidates.append(
                        self.__target_to_field_Pose(
                            fieldPose, target.getBestCameraToTarget()
                        )
                    )
                    candidates.append(
                        self.__target_to_field_Pose(
                            fieldPose, target.getAlternateCameraToTarget()
                        )
                    )

                    self._tag_metadatas[target_ID] = TagMetaData(
                        target_ID, target.getYaw
                    )

                    # Now with a candidate list, filter for the best results
                    filtered_candidates: List[Pose2d] = []
                    for candidate in candidates:
                        if self.__pose_on_field(candidate):
                            filtered_candidates.append(candidate)

                        # Add any other filtering here in the future

                    # Start with nothing, and replace if we find something
                    best_candidate = None
                    best_candidate_distance = 9999999
                    for candidate in filtered_candidates:
                        diff = (candidate - previousPoseEstimate).translation().norm()

                        if diff < best_candidate_distance:
                            best_candidate_distance = diff
                            best_candidate = candidate

                    # If we found something worth storing, store it
                    if best_candidate is not None:
                        self._pose_estimates.append(
                            CameraPoseEstimate(result_time, best_candidate)
                        )

    def get_pose_estimates(self) -> List:
        return self._pose_estimates

    def __target_to_field_Pose(self, targetPose: Pose3d, offset: Transform3d) -> Pose2d:
        cameraPose = targetPose.transformBy(self._center_offset.inverse())
        return cameraPose.transformBy(self._center_offset.inverse()).toPose2d()

    def __pose_on_field(self, pose: Pose2d) -> bool:
        translation: Transform2d = pose.translation()

        # Get the x,y values from the translation
        x = translation.X()
        y = translation.Y()

        # Translate field dimensions to meters, to match the Translation values
        # Then compare to make sure the location is within the field boundaries
        x_in_field: bool = 0.0 <= x <= feetToMeters(54)
        y_in_field: bool = 0.0 <= y <= feetToMeters(27)

        return x_in_field and y_in_field


class SimpleTagDetectionPhotonCamera:
    def __init__(self, name: str) -> None:
        setVersionCheckEnabled(False)

        self._camera: PhotonCamera = PhotonCamera(name)

        self._latest_result: PhotonPipelineResult = PhotonPipelineResult()

        # Initialize to a nonsense value, we will update this during auto init and
        # when the shooter is set to a specific site like amp or speaker
        self._target_fididial_id = 1

    def update_camera_results(self) -> None:
        self._latest_result = self._camera.getLatestResult()

    def get_tag_yaw(self) -> float:
        """
        Return the yaw of the April Tag, or 1000 if the correct speaker tag isn't
        detected in the pipeline
        """

           ## DF:  Existing Code Review Comment: Robotpy and libraries don't recognize "PhotonTrackedTarget", Researched and replaced
        # target_list: List[PhotonTrackedTarget] = self._latest_result.getTargets()
        target_list = self._latest_result.getTargets()

        # print (f" Target_list {target_list}")
        # print (f"{len(target_list)} >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  ")

        # If there are no current results, return 1000 to signify no target
        if len(target_list) == 0:
            # we have no targets
            return 1000


#             ==========================================
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

        # Iterate through the target list and filter on the April tag ID.
        for target in target_list:
            if target.getFiducialId() == self._target_fididial_id:   ## NEED TO SET THIS
                print (f"target.getYaw()  {target.getYaw()}")
                return target.getYaw()

        # We had some targets, but none matched so return the
        # no Yaw value of 1000
        return 1000

    def set_desired_target_id(self, id: float) -> None:
        self._target_fididial_id = id
