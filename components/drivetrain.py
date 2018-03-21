import math

from ctre.wpi_talonsrx import WPI_TalonSRX
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from wpilib import drive

from motioncontrol.utils import (RobotCharacteristics, RobotState, tank_drive_odometry,
                                 tank_drive_wheel_velocities)
from utils import NTStreamer


class Drivetrain:
    robot_drive = drive.DifferentialDrive

    left_drive_motor = WPI_TalonSRX
    right_drive_motor = WPI_TalonSRX
    navx = AHRS

    forward = 0
    curvature = 0
    rotation = 0

    robot_characteristics = RobotCharacteristics(
        acceleration_time=0.3,
        deceleration_time=1.8,
        max_speed=3.7,
        wheel_base=0.6096,
        encoder_ticks=1024 * 4,
        revolutions_to_distance=6 * math.pi * 0.02540,
        speed_scaling=3.7)

    wheel_distances = (0.0, 0.0)

    robot_state = RobotState()

    def setup(self):
        self.odometry_streamer = NTStreamer(self.robot_state, "drivetrain", round_digits=2)

    def forward_at(self, speed):
        self.forward = speed

    def curve(self, curvature):
        self.curvature = curvature

    def turn(self, speed):
        self.rotation = speed

    def get_orientation(self) -> float:
        return math.radians(-self.navx.getAngle())

    def _update_odometry(self):
        encoder_scaling = (self.robot_characteristics.encoder_ticks /
                           self.robot_characteristics.revolutions_to_distance)

        current_wheel_distances = (-self.left_drive_motor.getQuadraturePosition() / encoder_scaling,
                                   self.right_drive_motor.getQuadraturePosition() / encoder_scaling)

        enc_velocity = (self.right_drive_motor.getQuadratureVelocity() -
                        self.left_drive_motor.getQuadratureVelocity()) / 2

        velocity = 10 * enc_velocity / encoder_scaling

        self.robot_state = tank_drive_odometry(current_wheel_distances, self.wheel_distances,
                                               self.get_orientation(), self.robot_state.rotation,
                                               self.robot_state.position, velocity)

        self.wheel_distances = current_wheel_distances

    def _scale_speeds(self, vl: float, vr: float) -> (float, float):
        """Scales left and right motor speeds to a max of ±1.0 if either is
        greater
        """
        if abs(vl) >= abs(vr) and abs(vl) > 1.0:
            return math.copysign(1.0, vl), math.copysign(vr / vl, vr)
        if abs(vr) >= abs(vl) and abs(vr) > 1.0:
            return math.copysign(vl / vr, vl), math.copysign(1.0, vr)
        return vl, vr

    def execute(self):
        self._update_odometry()

        self.odometry_streamer.send(self.robot_state)

        if self.rotation:
            left = -self.rotation
            right = self.rotation
        else:
            left, right = tank_drive_wheel_velocities(self.robot_characteristics.wheel_base,
                                                      self.forward, self.curvature)

        left, right = self._scale_speeds(left, right)
        self.robot_drive.tankDrive(left, right, squaredInputs=False)

        self.forward = 0
        self.curvature = 0
        self.rotation = 0
