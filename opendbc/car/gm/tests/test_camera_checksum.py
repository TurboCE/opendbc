import unittest

from opendbc.can.packer import CANPacker
from opendbc.car.gm import gmcan


class TestCameraGasChecksum(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("gm_global_a_powertrain_generated")

  def test_captured_camera_boundaries(self):
    # Raw 0x2CB frames from Trailblazer camera captures: carry and, where
    # representable by the DBC, borrow across the last checksum byte.
    cases = (
      (154.0, bytes.fromhex("0142c50000bd3b00")),
      (1114.0, bytes.fromhex("0142e30000bd1d00")),
      (697.75, bytes.fromhex("c142d5fe00bd29ff")),
    )
    for throttle, expected in cases:
      counter = (expected[0] >> 6) & 3
      with self.subTest(throttle=throttle, counter=counter):
        actual = gmcan.create_gas_regen_command(self.packer, 0, throttle, counter, True, False,
                                               use_camera_checksum=True)[1]
        self.assertEqual(actual, expected)

  def test_preserves_non_checksum_bits(self):
    for enabled in (False, True):
      for full_stop in (False, True):
        for counter in range(4):
          with self.subTest(enabled=enabled, full_stop=full_stop, counter=counter):
            legacy = gmcan.create_gas_regen_command(self.packer, 0, 154.0, counter, enabled, full_stop)[1]
            camera = gmcan.create_gas_regen_command(self.packer, 0, 154.0, counter, enabled, full_stop,
                                                   use_camera_checksum=True)[1]
            self.assertEqual(camera[:5], legacy[:5])
            self.assertEqual(camera[4] & 1, int(not enabled))
            self.assertEqual((camera[0] >> 6) & 3, counter)

  def test_default_keeps_legacy_checksum(self):
    actual = gmcan.create_gas_regen_command(self.packer, 0, 154.0, 0, True, False)[1]
    self.assertEqual(actual, bytes.fromhex("0142c50000bd3a00"))


if __name__ == "__main__":
  unittest.main()
