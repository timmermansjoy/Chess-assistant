from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai
import timeit


def main():
    board = Board()
    for i in range(4):
        start = timeit.default_timer()
        if i % 2 == 0:
            print("My Turn:")
            beginCoord, endCoord = ai.calculateMove(3, board, True)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)

        else:
            print("Computers Turn:")
            beginCoord, endCoord = ai.calculateMove(3, board, False)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)

        print(board)
        # Your statements here

        stop = timeit.default_timer()

        print('Time: ', stop - start)
        print(board.isCheckmate)
        if board.isCheckmate:
            print("checkmate")
            break


if __name__ == "__main__":
    main()
