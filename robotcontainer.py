#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from commands2 import SequentialCommandGroup, PrintCommand

import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.units import rotationsToRadians

from subsystems.ledsubsystem import LEDSubsystem
from commands.ledcommand import LEDCommand
from subsystems.vision_subsystem import VisionSystem 

from apriltagalignmentdata import AprilTagAlignmentData

from commands.vision_alignment_mode import AprilTagAligmentMode
from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.drive_specific_distance import DriveDistanceSwerveCommand
from commands.drive_in_square_command_group import DriveInSquareDemo
from commands.drive_to_specific_point import DriveToSpecificPointSwerveCommand


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.85
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_rotational_deadband(
                self._max_angular_rate * 0.075
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        ###   DF:   Added to support robot-centric driving for AprilTag Alignment
        self._robot_centric_drive = (
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))

        self._logger = Telemetry(self._max_speed)
        self._joystick = CommandXboxController(0)
        self.drivetrain = TunerConstants.create_drivetrain()

        self._apriltag_alignment_data = AprilTagAlignmentData()
        self._ledsubsystem = LEDSubsystem()
        self._visionsubsystem = VisionSystem(self._apriltag_alignment_data)

        self._ledsubsystem.setDefaultCommand(LEDCommand( self._ledsubsystem, 100))


        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        move_speed_reduction = 0.4    #### Added to reduce speed while learning about swerve
        rotate_speed_reduction = 1.0  ###  NOTE THAT updating _max_speed did not seem to affect speed
        dead_zone = 0.055
        exp_scaling = 2.0

        self.drivetrain.setDefaultCommand(
            # >>>>>> DRIVE FIELD-CENTRIC <<<<<<<<<<<<<<<<
            self.drivetrain.apply_request(
                lambda: (
                    self
                    ._drive
                    .with_velocity_x(
                        # -self._joystick.getLeftY() * self._max_speed  * move_speed_reduction
                        -self.apply_deadzone_and_curve( self._joystick.getLeftY(), dead_zone, exp_scaling ) * self._max_speed  * move_speed_reduction
                        #### DF:  Updated:  Negated
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        # -self._joystick.getLeftX() * self._max_speed * move_speed_reduction
                        -self.apply_deadzone_and_curve( self._joystick.getLeftX(), dead_zone, exp_scaling ) * self._max_speed  * move_speed_reduction
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate    #### DF:  Original
                        # -self._joystick.getRightX() * self._max_angular_rate * rotate_speed_reduction
                            #### DF:  Updated:  Negated
                    )  # Drive counterclockwise with negative X (left)
                ) # End of Lambda
            ) # End of Apply_request
        )  # End of Default Command


        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # self._joystick.b().whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: (
        #             self
        #             ._robot_centric_drive
        #             .with_velocity_x(2.0)
        #             .with_velocity_y(2.0)
        #             .with_rotational_rate(0)
        #         ) 
        #     ) 
        # )

        # self._joystick.x().onTrue(AprilTagHeadingAligmentModePID(self.drivetrain,
        #                                                   self._visionsubsystem,
        #                                                   self._ledsubsystem,
        #                                                   self._apriltag_alignment_data))

        self._joystick.y().onTrue(AprilTagAligmentMode(self.drivetrain,
                                                          self._visionsubsystem,
                                                          self._ledsubsystem,
                                                          self._apriltag_alignment_data))

        self._joystick.a().onTrue(TurnHeadingSwerveCommand(self.drivetrain, 10))
        self._joystick.b().onTrue(TurnHeadingSwerveCommand(self.drivetrain, -10))

        # self._joystick.a().onTrue(DriveToSpecificPointSwerveCommand(self.drivetrain, 2.0, 0.0))  # forward, cross position Robot-centric in meters
        # self._joystick.b().onTrue(DriveToSpecificPointSwerveCommand(self.drivetrain, 2.0, -2.0))  # forward, cross position Robot-centric in meters
        # self._joystick.x().onTrue(DriveToSpecificPointSwerveCommand(self.drivetrain, 2.0, 2.0))  # forward, cross position Robot-centric in meters
        # self._joystick.y().onTrue(DriveToSpecificPointSwerveCommand(self.drivetrain, 3.0, 4.0))  # forward, cross position Robot-centric in meters


        # self._joystick.x().onTrue(DriveInSquareDemo(self.drivetrain,
        #                                             self._visionsubsystem,
        #                                             self._ledsubsystem,
        #                                             2.0))  # size in meters

        # self.movement_translation = Translation2d(2.0, 1.0)
        # self._joystick.y().onTrue(DriveDistanceSwerveCommand(self.drivetrain, self.movement_translation))
        # Pass in the robot and the desired movement in the  form of a translation (x,y object)

        # ERROR HERE:    Buttons not mapping correctly.

        # self._joystick.a().onTrue(PrintCommand("AAAAAAAAAAAAAAAAAA"))  # Triggered with A button
        # self._joystick.b().onTrue(PrintCommand("BBBBBBBBBBBBBBBBBB"))  # Triggered with X button
        # self._joystick.x().onTrue(PrintCommand("XXXXXXXXXXXXXXXXXX"))  # Triggered with B button
        # self._joystick.y().onTrue(PrintCommand("YYYYYYYYYYYYYYYYYY"))  # Triggered with Y button

        # self.heading_change_degrees = 30   # Degrees with positive is Counter-Clockwise
        # self._joystick.a().onTrue(TurnHeadingSwerveCommand(self.drivetrain, -self.heading_change_degrees))
        # self._joystick.b().onTrue(TurnHeadingSwerveCommand(self.drivetrain, self.heading_change_degrees))


        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def apply_deadzone_and_curve(self, axis_value: float, deadzone: float = 0.1, exponent: float = 2.0) -> float:
        if abs(axis_value) < deadzone:
            return 0.0
        # Normalize to 0-1 range after deadzone
        normalized = (abs(axis_value) - deadzone) / (1.0 - deadzone)
        # Apply curve (e.g., square for smoother ramp)
        curved = normalized ** exponent
        # Reapply sign
        final = curved * (1 if axis_value > 0 else -1)
        print(f"Axis value: {axis_value}, Normalized: {normalized}, Curved: {curved}, final: {final}")
        return final
