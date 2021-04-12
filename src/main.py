from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai


def main():
    board = Board()
<<<<<<< HEAD
    board.board = TB.Castle
    board.move(4,3,3,3)
    print(board.getPossibleMoves(7,7,board.board))

    
=======
    moves = board.getAllValidMoves()
    startpos, endpos = ai.PlayRandomMove(moves)
    startposrow, startposcol = startpos
    endPosRow, endPosCol = endpos
    board.move(startposrow, startposcol, endPosRow, endPosCol)

>>>>>>> refs/remotes/origin/main
    print(board)


if __name__ == "__main__":
    main()
