
#==(command)================================
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from commands2 import Command
from pathplannerlib.commands import PathPlannerFollowCommand
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.config import ReplanningConfig


from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.controller import PPHolonomicDriveController

# https://pathplanner.dev/pplib-follow-a-single-path.html#z6jotm8_24


class DriveToPoseCommand(Command):
    # def __init__(self, drivetrain, target_pose: Pose2d, initial_pose: Pose2d):
    def __init__(self, drivetrain, target_pose: Pose2d):
        super().__init__()
        self.target_pose = target_pose
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)


        ### Get robot's current pose (Position and heading)
        self.current_pose = self.drivetrain.get_robot_current_pose()

        # self.current_pose = Pose2d(self.drivetrain.get_state().pose.x,
        #                            self.drivetrain.get_state().pose.y, 
        #                            self.drivetrain.get_state().pose.rotation().radians())
        

        # Generate a simple path to the target pose
        # In a real robot, you'd likely use a pre-generated path or a more sophisticated path generation method
        path = PathPlannerPath.singlePath(
            self.current_pose,
            self.target_pose,
            ReplanningConfig()
        )

        self.follow_command = PathPlannerFollowCommand(
            path,
            self.drivetrain.getPose, # Function to get current robot pose
            self.drivetrain.getChassisSpeeds, # Function to get current chassis speeds
            self.drivetrain.applySwerveModuleStates, # Function to apply module states to the drivetrain
            self.drivetrain # The drivetrain subsystem
        )

        #----------------
        # Assuming this is a method in your drive subsystem
        def followPathCommand(pathName: str):
            path = PathPlannerPath.fromPathFile(pathName)

            return FollowPathCommand(
                path,
                self.getPose, # Robot pose supplier
                self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                self.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                    PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                    PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
                ),
                Constants.robotConfig, # The robot configuration
                self.shouldFlipPath, # Supplier to control path flipping based on alliance color
                self # Reference to this subsystem to set requirements
            )

        def shouldFlipPath():
            # Boolean supplier that controls when the path will be mirrored for the red alliance
            # This will flip the path being followed to the red side of the field.
            # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            return DriverStation.getAlliance() == DriverStation.Alliance.kRed

        #-----------

    def initialize(self):
        self.follow_command.initialize()
        print (" DRIVING TO POSE ")

    def execute(self):
        self.follow_command.execute()

    def isFinished(self) -> bool:
        return self.follow_command.isFinished()

    def end(self, interrupted: bool):
        self.follow_command.end(interrupted)


#==(Trigger the command)================================


# from wpimath.geometry import Pose2d, Translation2d, Rotation2d

# # Define the target pose (x, y, and heading)
# target_pose = Pose2d(Translation2d(1.0, 2.0), Rotation2d.fromDegrees(90.0))



# Assuming 'robot' is your RobotContainer instance and 'drivetrain' is your SwerveDrivetrain subsystem
# And 'initial_robot_pose' is the current estimated pose of your robot

# Example: Schedule the command to drive to the target pose
# robot.drivetrain.setDefaultCommand(DriveToPoseCommand(robot.drivetrain, target_pose, initial_robot_pose))

#==(Source)================================

# https://www.google.com/search?q=CTRE+swerve+drive+example+python+code+to+drive+to+a+new+pose+using+a+CTRE+swerve+drive+request&sca_esv=857f1b2178e74db9&sxsrf=AE3TifMLrfQ3XrpIueXXnixYLxnG4Jw3ww%3A1764899316908&ei=9DkyaYuUN-if5NoPlu_ssA4&ved=0ahUKEwiL3uSmqqWRAxXoD1kFHZY3G-YQ4dUDCBE&uact=5&oq=CTRE+swerve+drive+example+python+code+to+drive+to+a+new+pose+using+a+CTRE+swerve+drive+request&gs_lp=Egxnd3Mtd2l6LXNlcnAiXkNUUkUgc3dlcnZlIGRyaXZlIGV4YW1wbGUgcHl0aG9uIGNvZGUgdG8gZHJpdmUgdG8gYSBuZXcgcG9zZSB1c2luZyBhIENUUkUgc3dlcnZlIGRyaXZlIHJlcXVlc3RI3FlQAFjEP3ACeAGQAQCYAU2gAeEFqgECMTK4AQPIAQD4AQGYAgCgAgCYAwCSBwCgB8wKsgcAuAcAwgcAyAcA&sclient=gws-wiz-serp