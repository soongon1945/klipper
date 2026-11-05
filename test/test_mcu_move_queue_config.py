#!/usr/bin/env python3
import pathlib
import sys
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib" / "kconfiglib"))

import kconfiglib


class MoveQueueConfigTests(unittest.TestCase):
    def _load_config(self, machine):
        kconf = kconfiglib.Kconfig(str(ROOT / "src" / "Kconfig"))
        kconf.syms["MACH_STM32"].set_value(2)
        kconf.syms[machine].set_value(2)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".config") as config:
            kconf.write_config(config.name)
        return kconf

    def test_stm32f429_uses_expanded_move_queue(self):
        kconf = self._load_config("MACH_STM32F429")
        self.assertEqual(kconf.syms["MCU_MOVE_QUEUE_SIZE"].str_value, "2048")

    def test_stm32f103_keeps_existing_move_queue(self):
        kconf = self._load_config("MACH_STM32F103")
        self.assertEqual(kconf.syms["MCU_MOVE_QUEUE_SIZE"].str_value, "1024")


if __name__ == "__main__":
    unittest.main()
