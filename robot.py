#!/usr/bin/env python3

import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX
from magicbot import MagicRobot
from robotpy_ext.common_drivers.navx.ahrs import AHRS

from components.drivetrain import Drivetrain

from vision_interface import VisionTracking


class Robot(MagicRobot):

    drivetrain = Drivetrain

    def createObjects(self):
        self.left_drive_motor = WPI_TalonSRX(0)
        self.right_drive_motor = WPI_TalonSRX(2)

        self.right_drive_joystick = wpilib.Joystick(0)
        self.left_drive_joystick = wpilib.Joystick(1)

        WPI_TalonSRX(1).set(WPI_TalonSRX.ControlMode.Follower, self.left_drive_motor.getDeviceID())
        WPI_TalonSRX(3).set(WPI_TalonSRX.ControlMode.Follower, self.right_drive_motor.getDeviceID())

        self.robot_drive = wpilib.drive.DifferentialDrive(self.left_drive_motor,
                                                          self.right_drive_motor)

        self.navx = AHRS.create_spi()
        self.vision_interface = VisionTracking()


if __name__ == '__main__':
    wpilib.run(Robot)
