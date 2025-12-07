# physics.py
from pyfrc.physics.core import PhysicsEngine
# from pyfrc.simulation import SimDevice # For mocking SimDevices
# from pyfrc.physics.

# from wpilib.simulation import DifferentialDrivetrainSim, SimDeviceSim, ElevatorSim, DCMotorSim
from wpilib.simulation import SimDeviceSim  #, NavXSim

# from navx import AHRS
# from wpilib import DriverStation
from wpilib.simulation import NavXSim

import math

class PhysicsEngine(PhysicsEngine):
    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        # self.navx_sim = SimDeviceSim("navX-Sensor[4]") # Mock the specific navX device
        self.navx_sim = SimDeviceSim("navX-Sensor") # Mock the specific navX device
        # self.navx_sim2 = NavXSim(self.navx)

        # # Get the simulated values (or set initial ones)
        # self.sim_yaw = self.navx_sim.getFloat("yaw", 0.0)
        # self.sim_pitch = self.navx_sim.getFloat("pitch", 0.0)
        # self.sim_roll = self.navx_sim.getFloat("roll", 0.0)
        # self.sim_altitude = self.navx_sim.getFloat("altitude", 0.0)

        # Get the simulated values (or set initial ones)
        # self.sim_yaw = self.navx_sim.getDouble("yaw", 0.0)
        # self.sim_pitch = self.navx_sim.getDouble("pitch", 0.0)
        # self.sim_roll = self.navx_sim.getDouble("roll", 0.0)
        # self.sim_altitude = self.navx_sim.getDouble("altitude", 0.0)

        # Get the simulated values (or set initial ones)
        self.sim_yaw = self.navx_sim.getDouble("yaw")
        self.sim_pitch = self.navx_sim.getDouble("pitch")
        self.sim_roll = self.navx_sim.getDouble("roll")
        self.sim_altitude = self.navx_sim.getDouble("altitude")

        self.yaw_rate  =  0.0
        self.pitch_rate = 0.0
        self.roll_rate = 0.0
        self.altitude_change = 0.0

    def update_sim(self, state, dt):


        # self.navx_sim2.setAngle(45.0) # Set the simulated yaw angle to 45 degrees

        self.yaw_temp1 = self.navx_sim.getDouble("yaw").get()
        print (f"Yaw {self.yaw_temp1}")
        value : float = 40.0
        self.sim_yaw.set(value)

        self.navx_sim.set("yaw", self.sim_yaw)

        self.yaw_temp1 = self.navx_sim.getDouble("yaw").get()
        print (f"Yaw after {self.yaw_temp1}")
        

        


        # --- Update simulated sensor data based on game state or time ---
        # Example: Make yaw drift slightly over time
        # self.sim_yaw = float(self.sim_yaw.get()) + self.yaw_rate * dt
        # self.sim_yaw %= 360 # Keep yaw within 0-360 degrees

        # Example: Simulate a slight pitch change if robot is tilted in sim
        # You'd get actual robot tilt from physics_controller.chassis_speed
        # For simplicity, let's just add some random noise or movement
        #### self.sim_altitude = self.sim_altitude + self.altitude_change * dt

        # --- Set the values back into the SimDevice ---
        # self.navx_sim = SimDeviceSim("navX-Sensor[4]")
        # gyro_sim.setAngle(sim.toSimDouble(angle_in_degrees))
        # self.navx_sim.setFloat("yaw", self.sim_yaw)
        # self.navx_sim.setFloat("pitch", self.sim_pitch)
        # self.navx_sim.setFloat("roll", self.sim_roll)
        # self.navx_sim.setFloat("altitude", self.sim_altitude)

        # --- Set the values back into the SimDevice ---
        # self.navx_sim.setDouble("yaw", self.sim_yaw)
        # self.navx_sim.setDouble("pitch", self.sim_pitch)
        # self.navx_sim.setDouble("roll", self.sim_roll)
        # self.navx_sim.setDouble("altitude", self.sim_altitude)

        # Add other physics updates here (drivetrain, etc.)
        # Example: state.chassis_speed is available if using standard drivetrain
        # self.yaw_rate = ... # Calculate based on chassis_speed
