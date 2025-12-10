from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from photonlibpy.photonCamera import (
    PhotonCamera,
    PhotonPipelineResult,
)

from apriltagalignmentdata import AprilTagAlignmentData

class VisionSystemDUMMY(Subsystem):
    """
    This Vision Subsystem only supports the close alignment of the robot to a near-field April Tag.
    The subsystem returns the following results in a data class:
    - AprilTag Present
    - Yaw
    - Skew
    - Distance

    """    
    def __init__(self) -> None:
        # Run the constructor for the subsystem class
        super().__init__()


    def periodic(self) -> None:    
        pass
    
