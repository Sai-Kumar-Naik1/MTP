import sys
from PySide6.QtWidgets import QApplication, QLabel

app = QApplication(sys.argv)

label = QLabel("Drone Swarm Simulator")
label.resize(300, 100)
label.show()

app.exec()
