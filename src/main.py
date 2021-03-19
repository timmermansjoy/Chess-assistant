from board import *
from testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    board.board = TB.queenMoves
    print(board)
    print(board.getPossibleMoves(3, 3))
    board.placePieceOnNotation('K', 'b5')
    print(board.getPossibleMoves(3, 3))
    print(board)


if __name__ == "__main__":
    main()
