from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.kingNoMovesLeft
    print(board)
    print(board.getPossibleMoves(3, 0))
    



if __name__ == "__main__":
    main()
