from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from phoenix6 import SignalLogger, swerve, units, utils
from typing import Callable, overload
import wpilib
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6.swerve.requests import RobotCentric 
from phoenix6 import hardware, configs
import navx # Or your navX interface


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type,
            drivetrain_constants, arg0, arg1, arg2, arg3
        )

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        # Swerve request to apply during path following
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        # Instantiate the Gyro on the NAVX board on the ROBOTRIO
        self._gyro: navx.AHRS = navx.AHRS.create_spi()

        ## TODO   Initialize the Pigeon
        self.pigeon = hardware.Pigeon2(0, "canivore1")
    
        # Optional: Configure the Pigeon2 (e.g., Mount Pose)
        cfg = configs.Pigeon2Configuration()
        # Example: Pigeon is mounted flat (Yaw=0, Pitch=0, Roll=0)
        cfg.mount_pose.mount_pose_yaw = 0
        cfg.mount_pose.mount_pose_pitch = 0
        cfg.mount_pose.mount_pose_roll = 0
        self.pigeon.configurator.apply(cfg)

        # Retrieve StatusSignal objects for Yaw, Pitch, and Roll
        self.yaw_signal = self.pigeon.get_yaw()
        self.pitch_signal = self.pigeon.get_pitch()
        self.roll_signal = self.pigeon.get_roll()

        if utils.is_simulation():
            self._start_sim_thread()

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
#  This is used by the Path Planner Tool

        self._configure_auto_builder()

    def _configure_auto_builder(self):
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,   # Supplier of current robot pose
            self.reset_pose,                 # Consumer for seeding pose against auto
            lambda: self.get_state().speeds, # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                # PID constants for translation
                PIDConstants(10.0, 0.0, 0.0),
                # PID constants for rotation
                PIDConstants(7.0, 0.0, 0.0)
            ),
            config,
            # Assume the path needs to be flipped for Red vs Blue, this is normally the case
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
            self # Subsystem for requirements
        )

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def periodic(self):
        
        self.get_robot_heading()
        # print(f"Heading {self.get_state().pose.rotation().degrees()}")

        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def driving_change_heading(self, drive_turn_speed : float):
        self.swerve_drive_request = (
            RobotCentric()
            .with_velocity_x( 0.0 )
            .with_velocity_y( 0.0 )
            .with_rotational_rate(drive_turn_speed)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.set_control(self.swerve_drive_request)


    def driving_forward(self, drive_forward_speed : float):
        self.swerve_drive_request = (
            RobotCentric()
            .with_velocity_x( drive_forward_speed )
            .with_velocity_y( 0.0 )
            .with_rotational_rate(0.0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.set_control(self.swerve_drive_request)


    def driving_forward_and_update_heading(self, drive_forward_speed : float, drive_turn_speed : float):
        self.swerve_drive_request = (
            RobotCentric()
            .with_velocity_x( drive_forward_speed )
            .with_velocity_y( 0.0 )
            .with_rotational_rate(drive_turn_speed)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.set_control(self.swerve_drive_request)

    def stop_driving(self):
        self.swerve_drive_request = (
            RobotCentric()
            .with_velocity_x( 0.0 )
            .with_velocity_y( 0.0 )
            .with_rotational_rate(0.0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.set_control(self.swerve_drive_request)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    def get_robot_heading(self) -> float:



        if utils.is_simulation():
            self.current_gyro_heading = self.get_state().pose.rotation().degrees()  # Simulation
        else:
            self.yaw_signal.refresh()
            self.pigeon_yaw = self.yaw_signal.value
            print (f"Pigeon Yaw: {self.pigeon_yaw:5.2f}")

            self.current_gyro_heading = -self._gyro.getAngle()
            # print (f"Current Gyro value: {self.current_gyro_heading:5.1f}")
        return self.current_gyro_heading    
    

