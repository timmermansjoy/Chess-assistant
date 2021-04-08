from board import *
from testboards import Testboards as TB
from extra import Coordinate


def main():
    board = Board()
    cord = Coordinate(3, 4)
    row, col = cord

    print(cord)
    print(row, col)
    print(cord.row, cord.column)


if __name__ == "__main__":
    main()
