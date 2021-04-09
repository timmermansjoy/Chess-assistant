from board import *
from testboards import Testboards as TB
from extra import Coordinate


def main():
    board = Board()
    board.board = TB.Castle
    board.move(4,3,3,3)
    board.castling(False,False,board.board)
    
    print(board)


if __name__ == "__main__":
    main()
