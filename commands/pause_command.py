import wpilib
from commands2 import Command

class PauseCommand(Command):
    def __init__(self, pause_time_seconds : float) -> None:
        super().__init__()
        self.target_count = 50 * pause_time_seconds
        self.counter = 0

    def initialize(self) -> None:
        self.counter = 0
        print (f">>> Timer start")


    def execute(self) -> None:
        self.counter = self.counter + 1
        
    def isFinished(self) -> bool:
        if (self.counter > self.target_count):
            return True
        else:
            return False

    def end(self, interrupted: bool) -> None:
       print (f">>> Timer Complete")
       pass      


