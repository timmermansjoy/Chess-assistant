from PyQt5.QtWidgets import QLabel, QMainWindow, QApplication, QWidget, QVBoxLayout
from PyQt5.QtGui import QPixmap
import sys


class Menu(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Title")

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        lay = QVBoxLayout(self.central_widget)

        label = QLabel(self)
        pixmap = QPixmap('src/Bishop.png')
        label.setPixmap(pixmap)
        self.resize(300, 300)

        lay.addWidget(label)
        self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Menu()
    sys.exit(app.exec_())
