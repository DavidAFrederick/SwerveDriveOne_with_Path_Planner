import wpilib
from photonlibpy import photonvision
from wpimath.geometry import Transform3d, Pose3d, Rotation3d, Rotation2d, Pose2d
from wpilib import DriverStation


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Initialize PhotonCamera
        self.camera = photonvision.PhotonCamera("photonvision") # Replace "photonvision" with your camera name

        # Define the camera-to-robot transform (e.g., if camera is mounted on the robot)
        # This is a placeholder; you'll need to accurately measure your robot's setup.
        # Example: Camera is 0.2 meters forward, 0.1 meters left, and 0.3 meters up from robot center.
        # It's also rotated -90 degrees around the Z-axis (pointing right relative to robot front).
        self.camera_to_robot = Transform3d(0.2, 0.1, 0.3, Rotation3d(0, 0, -90)) 

        # Load the AprilTag field layout for your competition year
        self.tag_layout = wpilib.AprilTagFieldLayout(wpilib.AprilTagField.k2024Crescendo) # Or your specific year

        # Initialize PhotonPoseEstimator
        self.pose_estimator = photonvision.PhotonPoseEstimator(
            self.tag_layout,
            photonvision.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, # Or another suitable strategy
            self.camera,
            self.camera_to_robot
        )

        # Initialize a drivetrain pose estimator (e.g., SwerveDrivePoseEstimator or DifferentialDrivePoseEstimator)
        # This is a simplified placeholder; you'd integrate this with your actual drivetrain.
        self.drivetrain_pose_estimator = wpilib.estimator.SwerveDrivePoseEstimator(
            wpilib.kinematics.SwerveDriveKinematics(
                # Define your swerve module locations here
                wpilib.geometry.Translation2d(0.5, 0.5),
                wpilib.geometry.Translation2d(0.5, -0.5),
                wpilib.geometry.Translation2d(-0.5, 0.5),
                wpilib.geometry.Translation2d(-0.5, -0.5)
            ),
            Rotation2d(0), # Initial gyro heading
            (0,0,0,0), # Initial module positions
            Pose2d(0,0,Rotation2d(0)) # Initial robot pose
        )


    def robotPeriodic(self):
        # Get the latest vision results
        vision_results = self.camera.getLatestResult()

        # Update the PhotonPoseEstimator with the latest results
        # This returns an Optional<EstimatedRobotPose>
        estimated_robot_pose_optional = self.pose_estimator.update(vision_results)

        # If a pose was estimated, use it to update the drivetrain pose estimator
        if estimated_robot_pose_optional.isPresent():
            estimated_robot_pose = estimated_robot_pose_optional.get()
            
            # The EstimatedRobotPose contains the estimated Pose3d and its timestamp
            self.drivetrain_pose_estimator.addVisionMeasurement(
                estimated_robot_pose.estimatedPose.toPose2d(),
                estimated_robot_pose.timestampSeconds
            )

            # You can also access other information from the estimated_robot_pose if needed
            # For example: estimated_robot_pose.strategy, estimated_robot_pose.ambiguity

        # In a real robot, you would also be updating your drivetrain pose estimator
        # with odometry data (wheel encoders, gyro) in every robotPeriodic loop.
        # Example:
        # self.drivetrain_pose_estimator.update(
        #     self.gyro.getRotation2d(),
        #     self.front_left_module.getPosition(),
        #     self.front_right_module.getPosition(),
        #     self.back_left_module.getPosition(),
        #     self.back_right_module.getPosition()
        # )

        # Get the current estimated robot pose from your drivetrain pose estimator
        current_robot_pose = self.drivetrain_pose_estimator.getEstimatedPosition()
        # print(f"Current Robot Pose: {current_robot_pose}")

if __name__ == "__main__":
    wpilib.run(MyRobot)