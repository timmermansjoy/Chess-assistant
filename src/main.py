from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.kingMoves
    # print(board)
    print(board.getKnightMoves.__doc__)
    print(board.notationToCords("a2a4"))


if __name__ == "__main__":
    main()
