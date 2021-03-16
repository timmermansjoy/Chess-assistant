from Board import Board
from Board import Coordinate
from Testboards import Testboards as TB


def main():
    board = Board()
    board.board=TB.knightmoverTest
    print(board)
    print(board.getPossibleMoves(4,3))
    #move from x,y to
    board.move(4,3,6,2)
    board.move(4,3,6,3)
    print(board)



if __name__ == "__main__":
    main()
