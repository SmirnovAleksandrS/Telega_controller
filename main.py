"""
Entry point.

GUI: Tkinter (stdlib) for zero-deps cross-platform UI.
UART: pyserial.
"""

from app.gui_app import VirtualControllerApp

def main() -> None:
    app = VirtualControllerApp()
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()

if __name__ == "__main__":
    main()
