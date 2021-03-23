from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.kingMoves
    print(board)
    for i in board.getAllAttackedFields(Coordinate(3,4)):
        print(i)


if __name__ == "__main__":
    main()
