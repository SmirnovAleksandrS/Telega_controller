"""
Entry point.

GUI: Tkinter (stdlib) for zero-deps cross-platform UI.
UART: pyserial.
"""

import argparse

from app.gui_app import VirtualControllerApp

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fixed-route-canvases",
        action="store_true",
        help="disable dynamic coordinate canvases and use legacy fixed-size route view",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    app = VirtualControllerApp(fixed_route_canvases=args.fixed_route_canvases)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()

if __name__ == "__main__":
    main()
