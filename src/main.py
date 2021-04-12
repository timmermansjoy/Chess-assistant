from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai


def main():
    board = Board()

    for i in range(10):
        moves = board.getAllValidMoves()
        startpos, endpos = ai.PlayRandomMove(moves)
        board.move(startpos.row, startpos.column, endpos.row, endpos.column)
        print(board)
        print(i)


if __name__ == "__main__":
    main()
