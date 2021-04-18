from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai


def main():
    board = Board()

    for i in range(11):
        if i%2 == 0:
            print("My Turn:")
            beginCoord, endCoord = ai.minimaxRoot(3,board,True)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
            
        else:
            print("Computers Turn:")
            beginCoord, endCoord = ai.minimaxRoot(3,board,False)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
            
        print(board)


if __name__ == "__main__":
    main()
