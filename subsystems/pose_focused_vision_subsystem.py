import wpilib
from commands2 import Subsystem, Command, cmd

from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy

from robotpy_apriltag import AprilTagFieldLayout, AprilTag, AprilTagFields
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.units import meters

#  DF:  Camera Name:  OV9281
# https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-welded.json



class VisionSubsystem(Subsystem):
    def __init__(self):
        self.camera = PhotonCamera("OV9281") # Replace with your camera name
        self.pose_estimator = PhotonPoseEstimator(
            # wpilib.apriltag.AprilTagFieldLayout(wpilib.apriltag.loadAprilTagFieldLayout("2024Game.json")), # Load field layout
            AprilTagFieldLayout(wpilib.apriltag.loadAprilTagFieldLayout("2025Game.json")), # Load field layout
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            self.camera,
            wpilib.Transform3d(
                wpilib.Translation3d(0.0, 0.0, 0.0), # Camera position relative to robot center
                wpilib.Rotation3d(0, 0, 0) # Camera rotation relative to robot center
            )
        )
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")
        print (f"Vision System Instantiated ========================================")

    def get_latest_result(self):
        return self.camera.getLatestResult()

    def get_estimated_pose(self, current_robot_pose):
        self.pose_estimator.setReferencePose(current_robot_pose)
        return self.pose_estimator.update()
    

"""
    Swerve Drivetrain Control: Your swerve drive code will need to accept desired vx, vy (field-centric or robot-centric), and omega (angular velocity).

Alignment Logic:
- When an AprilTag is detected, obtain its pose relative to the camera.
Calculate the desired robot pose relative to the AprilTag (e.g., a specific distance and angle from the tag).
- Use a PID controller (or similar control loop) to generate vx, vy, and omega commands for your swerve drive to move the robot towards the desired alignment.
- For example, if aligning to face the tag squarely, you might use the tag's yaw to control the robot's angular velocity and its range to control forward/backward movement.

Command Implementation:
- Create a command (e.g., AlignToAprilTagCommand) that utilizes the VisionSubsystem and the SwerveDrivetrain.
- This command will periodically read vision data, calculate alignment errors, and send appropriate commands to the swerve drive.

Example Alignment Strategy:
- Aiming: Use the tx (yaw) from the vision system to control the robot's turning (omega) to center the tag in the camera's view.
- Ranging: Use the ty (pitch) or calculated distance to control the robot's forward/backward movement (vx).
- Strafing: If aiming for a precise lateral position relative to the tag, use the tx to control strafing (vy) or adjust the robot's field-centric position based on the tag's field-relative pose.

Key Considerations:
- Coordinate Systems: Be mindful of the different coordinate systems (camera-centric, robot-centric, field-centric) and perform necessary transformations.
- PID Tuning: Carefully tune your PID controllers for smooth and accurate alignment.
- Error Handling: Implement error handling for cases where no AprilTag is detected.
- Field Layout: Ensure your robot code accurately reflects the AprilTag field layout for proper pose estimation.

https://www.google.com/search?q=ctre+swerve+drive+python+how+to+add+april+tag+alignment+using+vision&sca_esv=9ce27de1500c60ce&sxsrf=AE3TifOVQhqzLwwnQPbBsKUG2V8U7zry_Q%3A1763839379257&ei=kw0iabKyD4Dl5NoP09iv6Ak&ved=0ahUKEwjyx5DdvYaRAxWAMlkFHVPsC50Q4dUDCBE&uact=5&oq=ctre+swerve+drive+python+how+to+add+april+tag+alignment+using+vision&gs_lp=Egxnd3Mtd2l6LXNlcnAiRGN0cmUgc3dlcnZlIGRyaXZlIHB5dGhvbiBob3cgdG8gYWRkIGFwcmlsIHRhZyBhbGlnbm1lbnQgdXNpbmcgdmlzaW9uSM_ZAVDuCFjmwgFwCHgBkAEBmAGrAaAByyaqAQUzNy4xNbgBA8gBAPgBAZgCM6ACiCLCAgoQABiwAxjWBBhHwgIFECEYoAHCAgUQIRirAsICBxAhGKABGAqYAwCIBgGQBgiSBwUzNi4xNaAH8a8CsgcFMjguMTW4B-YhwgcGMC40OS4yyAdw&sclient=gws-wiz-serp

"""

# ========================

"""
DRIVING THE ROBOT ROBOT_CENTRIC

ctre swerve drive python how to drive the robot robot-centric




https://www.google.com/search?q=ctre+swerve+drive+python+how+to+drive+the+robot+robot-centric&oq=ctre+swerve+drive+python+how+to+drive+the+robot+robot-centric&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIHCAEQIRiPAjIHCAIQIRiPAtIBCjE0NTM1ajBqMTWoAgiwAgHxBSlPXyJM_RSM8QUpT18iTP0UjA&sourceid=chrome&ie=UTF-8



"""

