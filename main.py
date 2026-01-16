"""
Entry point.

GUI: Tkinter (stdlib) for zero-deps cross-platform UI.
UART: pyserial.
"""

from app.gui_app import VirtualControllerApp

def main() -> None:
    app = VirtualControllerApp()
    app.run()

if __name__ == "__main__":
    main()
