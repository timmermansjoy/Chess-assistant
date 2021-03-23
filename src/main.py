from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.kingMoves
    print(board)
<<<<<<< HEAD
    for i in board.getAllAttackedFields(Coordinate(3,4)):
        print(i)

=======
    print(board.getPossibleMoves(3, 0))
    board.board = TB.kingMoves
    print(board.getAllAttackedFields(False))
    print(board.getAllAttackedFields(True))
>>>>>>> 1d8b839d4b1c88a20d78e42451b4e2863c1fa764

if __name__ == "__main__":
    main()
