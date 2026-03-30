from __future__ import annotations

import unittest

import main


class MainCliTests(unittest.TestCase):
    def test_fixed_route_canvases_flag_is_parsed(self) -> None:
        args = main.parse_args(["--fixed-route-canvases"])

        self.assertTrue(args.fixed_route_canvases)

    def test_fixed_route_canvases_defaults_to_disabled(self) -> None:
        args = main.parse_args([])

        self.assertFalse(args.fixed_route_canvases)


if __name__ == "__main__":
    unittest.main()
