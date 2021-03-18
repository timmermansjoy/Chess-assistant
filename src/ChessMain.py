from Board import *
from Testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.queenMoves
    print(board)
    print(board.getPossibleMoves(3, 3))
    # print(board.notationToCords("b1c3"))
    # board.move(7, 1, 5, 2)
    # print(board)


if __name__ == "__main__":
    main()
