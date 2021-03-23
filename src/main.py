from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.kingMoves
    print(board)
<<<<<<< HEAD
    print(board.getPossibleMoves(3, 4))
    board.board = TB.kingMoves
    print(board.getAllAttackedFields(False))
    print(board.getAllAttackedFields(True))
=======
    print(board.getPossibleMoves(3, 0))
    board.board = TB.kingMoves
    print(board.getAllAttackedFields(False))
    print(board.getAllAttackedFields(True))

>>>>>>> 33380ece0c9f5e15920a0eb4d3a2c3516e6d03da

if __name__ == "__main__":
    main()
