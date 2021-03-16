from Board import Board
from Board import Coordinate
from Testboards import Testboards as TB


def main():
    board = Board()
    board.board=TB.knightmoverTest
    print(board)
    print(board.getPossibleMoves(4,3))


if __name__ == "__main__":
    main()
