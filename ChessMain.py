from Board import Board
from Board import Coordinate
from Testboards import Testboards as TB


def main():
    board = Board()
    #board.board=TB.bishopAccessTest
    print(board)
    print(board.getPossibleMoves(6,4))
    #move from x,y to
    board.move(6,4,4,4)
    print(board)



if __name__ == "__main__":
    main()
