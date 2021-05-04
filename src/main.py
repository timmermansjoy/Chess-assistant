from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai
import timeit


def main():
    board = Board()
    board.move(6,7,4,7)
    board.move(1,1,3,1)
    board.move(4,7,3,7)
    board.move(1,6,3,6)
    print(board)
    board.move(3,7,2,6)
    print(board)
    board.undo()
    print(board)


if __name__ == "__main__":
    main()
