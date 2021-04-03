from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    board.move(6, 7, 5, 7)
    board.move(1, 2, 2, 2)
    board.move(6, 4, 5, 4)
    board.move(0, 3, 3, 0)
    print(board)
    board.move(6, 3, 5, 3)

    print(board.moveLog)
    print(board.GetChessNotation())
    # print(board.fieldsUnderWhiteThreat)
    # print(board.getAllAttackedFields(True))
    print(board)


if __name__ == "__main__":
    main()
