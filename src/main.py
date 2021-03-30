from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    board.move(6, 4, 4, 4)
    board.move(1, 4, 3, 4)
    board.move(7, 5, 5, 3)
    board.move(0, 6, 2, 5)
    board.move(7, 6, 5, 5)
    board.move(0, 5, 5, 0)
    board.board[2][3] = "B"
    board.castling(True, False)
    print(board.moveLog)
    print(board.GetChessNotation())
    # print(board.fieldsUnderWhiteThreat)
    # print(board.getAllAttackedFields(True))
    print(board)


if __name__ == "__main__":
    main()
