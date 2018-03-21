from magicbot.state_machine import AutonomousStateMachine, state
from vision_interface import VisionTracking
from components.drivetrain import Drivetrain


class FollowAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Follow'
    DEFAULT = True

    drivetrain = Drivetrain
    vision_interface = VisionTracking

    last_direction = None

    @state(first=True)
    def find(self):
        vision_data = self.vision_interface.cube_position()

        if vision_data is not None:
            self.next_state('follow')
            return

        if self.last_direction == "left":
            self.drivetrain.turn(0.15)
        else:
            self.drivetrain.turn(-0.15)

    @state
    def follow(self):
        vision_data = self.vision_interface.cube_position()

        if vision_data is None:
            self.next_state('find')
            return

        offset, self.last_direction = vision_data

        self.drivetrain.curve(offset / 320)
        self.drivetrain.forward_at(0.2)
