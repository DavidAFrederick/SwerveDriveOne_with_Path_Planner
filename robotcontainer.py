#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from commands2 import SequentialCommandGroup, PrintCommand
import wpilib
from wpilib import LiveWindow
import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerAuto
from phoenix6 import swerve, utils
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.units import rotationsToRadians
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.vision_subsystem import VisionSystem 
from subsystems.vision_subsystem_dummy import VisionSystemDUMMY
from apriltagalignmentdata import AprilTagAlignmentData
from commands.vision_alignment_mode import AprilTagAligmentMode
from commands.vision_alignment_with_offset import AprilTagWithOffsetAligmentCalculation
from commands.turn_specific_heading import TurnHeadingSwerveCommand
from commands.drive_specific_distance import DriveDistanceSwerveCommand
from commands.drive_in_square_command_group import DriveInSquareDemo
from commands.drive_to_specific_point_with_lateral import DriveToSpecificPointXYSwerveCommand
from commands.drive_to_apriltag_turnpoint_group import  DriveToAprilTagTurnPointCmdGroup
from commands.drive_forward_2_seconds import  Drive_Forward_X_Seconds
from commands.print_robot_pose_position import PrintRobotPose
from commands.reset_robot_pose_to_zero import ResetRobotPose
from commands.ledcommand import LEDCommand

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        ## DF: TRYING TO SILENCE A NUMBER OF ERRORS IN THE CONSOLE LOG

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # The function FRC LiveWindow.disableAllTelemetry() is a static method 
        # in the FRC (FIRST Robotics Competition) WPILib library that disables t
        # he sending of data for all sensors and actuators to the SmartDashboard 
        # or Shuffleboard LiveWindow display.  

        LiveWindow.disableAllTelemetry()
        
        # https://robotpy.readthedocs.io/projects/robotpy/en/latest/wpilib/LiveWindow.html        

        #    We are getting a number of Watch Dog errors due to excessive time being taken up by RobotPeriodic().
        #    Near line 320, is a command "register_telemetry" which appears to cause the motors to create and log
        #    telemetry  data.  By removing  this line, the errors have disappeared.   This is needed for simulation.
        #    NOTE: It appears some CTRE settings require power cycling to update the configuration.
        #

        # ### THIS IS TEMPORARY TO SILENCE LOGS
        # self.my_watchdog = wpilib.Watchdog(0.1, self.watchdog_expired_callback)
        # self.my_watchdog.disable()
        # print("Watchdog disabled during disabled mode  <<<<<<<<<<<<<<<<<<<<")
        # #  Did not appear to work........


        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.85
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()      ### DF: We could change this to  swerve.requests.RobotCentric()
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
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))    ## Likely can drop this decorator

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))

        self._logger = Telemetry(self._max_speed)
        self._joystick = CommandXboxController(0)
        self.drivetrain = TunerConstants.create_drivetrain()
        self._apriltag_alignment_data = AprilTagAlignmentData()     ###  DF: Data only class to share data between Vision and Drivetrain
        self._ledsubsystem = LEDSubsystem()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - j- - - 
        # Instantiate the Vision Subsystem if not in Test Mode (Using simulator)

        self._apriltag_alignment_data.set_test_mode(False)  # Change this parameter to enable use of fake photonvision data

        if (self._apriltag_alignment_data.get_test_mode()):   # True means use simulated data and NOT the photonvision
            self._visionsubsystem = VisionSystemDUMMY()   #  TEST
            print (">>> ============== Vision Test Mode Enabled ====================")
        else:
            self._visionsubsystem = VisionSystem(self._apriltag_alignment_data)   

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Setup For Path Planner

        # NamedCommmand must be before creating the auto_chooser
        NamedCommands.registerCommand('RedLEDs', LEDCommand(self._ledsubsystem,0)) 
        NamedCommands.registerCommand('GreenLEDs', LEDCommand(self._ledsubsystem,60)) 
        NamedCommands.registerCommand('BlueLEDs', LEDCommand(self._ledsubsystem,120)) 

        # Path follower - Build Pick List for SmartDashboard
        self._auto_chooser = AutoBuilder.buildAutoChooser("P3-Path-Delta-Default")
        self._auto_chooser.addOption("P1-Path-Alpha", PathPlannerAuto("P1-Path-Alpha"))
        self._auto_chooser.addOption("P2-Path-Beta",  PathPlannerAuto("P2-Path-Beta"))
        SmartDashboard.putData("Select Autonomous Mode Path", self._auto_chooser)

        # Process to add Autonomous Paths:
        # https://docs.google.com/document/d/1EGXVbdAE8ooO11D77TVcRQAntiApK8l3d3jXP6t5k4w


        # Expanding Path Planner:
        #
        # [step 1] Titles of Paths to be displayed on Smart Dashboard:
        #          Examples:
        #             P1-Path-Alpha
        #             P2-Path-Beta
        #             P3-Path-Delta-Default
        # [step 2] Draw paths and autos using names in step 1.   Figure out the goals and exact positions/headings needed
        # [step 3] Identify Robot Commands to be performed using auto.   LED commands, Elevator, Intake commands ....
        #          LEDCommand( self._ledsubsystem, 0)    # LED Red
        #          LEDCommand( self._ledsubsystem, 60)   # LED Green
        #          LEDCommand( self._ledsubsystem, 120)  # LED Blue
        # [step 4] Create the Names of the Paths to be created in PathPlanner App (later)
        #          P1-Path-Alpha
        #          P2-Path-Beta
        #          P3-Path-Delta-Default
        # [step 5] Create the "AutoChooser" within the RobotContainer File
        # [step 6] Create "NamedCommands" within the RobotContainer file
        # [step 7] Create the actual paths and plans in Path Planner.  !!! Be sure to check the box "reset Odometry"
        # [step 8] Test within the Simulation.  Be aware of which aliance (red/blue) in use.
        #

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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
                lambda: (            ## TODO Clean this up back to the original approach
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

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        #   Current Commands Implemented and status:  (as of 12/30/2015)
        # - - - - - - - - - - - - - - - - - - - - - - - - 
        # drive_in_square_command_group.py      - DriveInSquareDemo                     - Working - (This is a group of commands)
        # drive_specific_distance.py            - DriveDistanceSwerveCommand            - Working
        # drive_to_specific_point.py            - DriveToSpecificPointSwerveCommand     - Working
        # drive_to_specific_point_with_lateral	- DriveToSpecificPointXYSwerveCommand   - To be tested
        # ledcommand.py                         - LEDCommand                            - Working
        # pause_command.py                      - PauseCommand                          - Working
        # turn_specific_heading.py              - TurnHeadingSwerveCommand              - Working
        # vision_alignment_mode.py              - AprilTagAligmentMode                  - Updated - Ready to be tested
        # vision_alignment_with_offset	        - vision_alignment_with_offset          - To be Tested
        # drive_to_apriltag_turnpoint_group	    - DriveToAprilTagTurnPointCmdGroup      - To be Tested
        # print_robot_pose_position	            - PrintRobotPose                        - Used for checking driving commands
        # reset_robot_pose_to_zero	            - ResetRobotPose                        - Resets pose to 0,0,0 without code deployment
        # drive_forward_2_seconds			    - Drive_Forward_X_Seconds               - Very simple example for teaching about commands


        # - - - - - - - - - - - - - - - - - - - - - - - - 

        # Most of the button bindings in this section can be removed.
        # They were placed here to test various commands and functions


        self._joystick.leftBumper().whileTrue(
        # >>>>>> DRIVE ROBOT-CENTRIC  When Left Bumper is held  <<<<<<<<<<<<<<<<  getPOV() in Simulation
                self.drivetrain.apply_request(
                    lambda: (
                        self
                        ._robot_centric_drive
                        .with_velocity_x(-self._joystick.getLeftY() *  self._max_speed * 0.5)
                        .with_velocity_y(-self._joystick.getLeftX() *  self._max_speed *  0.5)
                        .with_rotational_rate(-self._joystick.getRightX() * self._max_angular_rate)
                    ) # End of Lambda
                )
        )

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


        # self._joystick.a().onTrue(TurnHeadingSwerveCommand(self.drivetrain,  30))
        # self._joystick.b().onTrue(TurnHeadingSwerveCommand(self.drivetrain, -30))
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        # self._joystick.a().onTrue(Drive_Forward_X_Seconds(self.drivetrain, 2))

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        self._joystick.a().onTrue(DriveToAprilTagTurnPointCmdGroup(self.drivetrain,
                                                          self._visionsubsystem,
                                                          self._apriltag_alignment_data))

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        self._joystick.x().onTrue(PrintRobotPose(self.drivetrain))
        self._joystick.y().onTrue(ResetRobotPose(self.drivetrain))

        

        # self._joystick.x().onTrue(DriveInSquareDemo(self.drivetrain, self._ledsubsystem, 1.0))

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        
        
        # self._joystick.a().onTrue(DriveToSpecificPointSwerveCommand(self.drivetrain, self._apriltag_alignment_data))  
        # self._joystick.b().onTrue(DriveToSpecificPointXYSwerveCommand(self.drivetrain, self._apriltag_alignment_data))  
        # forward, cross position Robot-centric in meters
        ## TODO  - Update notes to describe how to use this command
        
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # self._joystick.x().onTrue(DriveInSquareDemo(self.drivetrain,
        #                                             self._visionsubsystem,
        #                                             self._ledsubsystem,
        #                                             2.0))  # size in meters

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # self.movement_translation = Translation2d(2.0, 1.0)
        # self._joystick.y().onTrue(DriveDistanceSwerveCommand(self.drivetrain, self.movement_translation))
        # Pass in the robot and the desired movement in the  form of a translation (x,y object)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # ERROR HERE:    Buttons not mapping correctly.

        # self._joystick.leftTrigger().onTrue(PrintCommand("LT - LT - LT - LT - LT - LT - LT - LT - "))  # Triggered with A button
        # self._joystick.a().onTrue(PrintCommand("AAAAAAAAAAAAAAAAAA"))  # Triggered with A button
        # self._joystick.b().onTrue(PrintCommand("BBBBBBBBBBBBBBBBBB"))  # Triggered with X button
        # self._joystick.x().onTrue(PrintCommand("XXXXXXXXXXXXXXXXXX"))  # Triggered with B button
        # self._joystick.y().onTrue(PrintCommand("YYYYYYYYYYYYYYYYYY"))  # Triggered with Y button


        ## COMMENTS ON THE BUTTON MISMAPPING 
        # Plugging Cheap Gamepad (www.izdtech.com ZD-V108 into laptop)
        # Starting Real DriverStation Application
        # 
                # Pressing the A button lights the top button on the DriverStation USB Test page
        # Pressing the B Button lights the Second from top
        # Pressing the X Button lights the Third from top
        # Pressing the Y Button lights the Fourth from top
        #
        # This looks correct.
        #
        # Running the simulator with the real gamepad.  Identifies it as a XBox Controller
        # Pressing button corectly identifies the buttons A = A, ...
        #
        # We should use X Input mode according to FRC
        #
        # To switch your ZD-V108 gamepad between XInput (Xbox) and 
        # DirectInput (older standard), usually hold the "V" button 
        # (or sometimes the Home/Mode button) for about 5 seconds, which changes 
        # the light color (blue for XInput, red for DirectInput)

        # You can use the D-pad or Left-stick to play the game according to your 
        # heart using this amazing function. By pressing the BACK + LSB (push the Left Stick vertically) 
        # key combinations, you can switch the function between the D-pad and Left Stick in PC Windows and Xinput mode.

        #
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        ## DF:  These command seem to be working.  No excess printing to console
        # self.heading_change_degrees = 10   # Degrees with positive is Counter-Clockwise
        # self._joystick.a().onTrue(TurnHeadingSwerveCommand(self.drivetrain, -self.heading_change_degrees))
        # self._joystick.b().onTrue(TurnHeadingSwerveCommand(self.drivetrain, self.heading_change_degrees))


        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        ## TODO:    See if this command is causing the WatchDog timer to trip on real Robot

        if utils.is_simulation():
            self.drivetrain.register_telemetry(
                lambda state: self._logger.telemeterize(state)
            )

        #  Looks like this  resolved the Frame Overrun issues.
        #  NOTE:  NEEDED TO CYCLE POWER ON ROBOT FOR THIS SETTING  TO TAKE EFFECT.

    def apply_deadzone_and_curve(self, axis_value: float, deadzone: float = 0.1, exponent: float = 2.0) -> float:
        if abs(axis_value) < deadzone:
            return 0.0
        # Normalize to 0-1 range after deadzone
        normalized = (abs(axis_value) - deadzone) / (1.0 - deadzone)
        # Apply curve (e.g., square for smoother ramp)
        curved = normalized ** exponent
        # Reapply sign
        final = curved * (1 if axis_value > 0 else -1)
        # print(f">>> Axis value: {axis_value}, Normalized: {normalized}, Curved: {curved}, final: {final}")
        return final

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()

    # def watchdog_expired_callback(self):
    #         print("Watchdog expired!  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")