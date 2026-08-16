"""Entry point for Module 1 - GUI Application (SRS §3)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6 import QtLocation, QtPositioning  # noqa: F401  (registers QML plugins)
from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow



def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Drone Swarm Simulator - GUI")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
