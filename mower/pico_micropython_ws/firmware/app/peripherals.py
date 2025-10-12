"""
Deprecated: this module has been split into dedicated files.

Use the following modules instead:
 - lift_sensors.py     → class LiftSensors
 - bumper_sensors.py   → class BumperSensors
 - perimeter_sensors.py→ class PerimeterSensors
 - aux_outputs.py      → class AuxOutputs

This file remains as a thin compatibility shim that re-exports those classes
and provides the same Peripherals aggregator.
"""

from lift_sensors import LiftSensors  # type: ignore
from bumper_sensors import BumperSensors  # type: ignore
from perimeter_sensors import PerimeterSensors  # type: ignore
from aux_outputs import AuxOutputs  # type: ignore


class Peripherals:
    def __init__(self):
        self.lift = LiftSensors()
        self.bumpers = BumperSensors()
        self.perimeter = PerimeterSensors()
        self.aux = AuxOutputs()

    def snapshot(self):
        return {
            'lift': self.lift.read(),
            'bumpers': self.bumpers.read(),
            'perimeter': {
                'left_raw': self.perimeter.raw_left(),
                'right_raw': self.perimeter.raw_right(),
                'left_active': self.perimeter.left_active(),
                'right_active': self.perimeter.right_active(),
            },
        }
