from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai


def main():
    board = Board()
    moves = board.getAllValidMoves()
    startpos, endpos = ai.PlayRandomMove(moves)
    startposrow, startposcol = startpos
    endPosRow, endPosCol = endpos
    board.move(startposrow, startposcol, endPosRow, endPosCol)

    print(board)


if __name__ == "__main__":
    main()
