from Board import Board
from Board import Coordinate
from Testboards import Testboards as TB


def main():
    board = Board()
    # board.board = TB.BlackOneTakeOneMoveOption
    print(board)
    print(board.getPossibleMoves(7, 1))
    print(board.notationToCords("b1c3"))
    board.move(7, 1, 5, 2)
    print(board)


if __name__ == "__main__":
      main()
