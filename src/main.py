from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai


def main():
    board = Board()

    for i in range(5):
        if i % 2 == 1:
            print("My Turn:")
            s = input()
            board.notationMove(s)

        else:
            print("Computers Turn:")
            beginCoord, endCoord = ai.calculateMove(3, board, False, i)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)

        print(board)


if __name__ == "__main__":
    main()
