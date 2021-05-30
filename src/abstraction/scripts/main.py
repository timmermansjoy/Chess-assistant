from board import *
from testboards import Testboards as TB
from extra import Coordinate
import ai
import timeit


def test():
    board = Board()
    board.move(6, 7, 4, 7)
    board.move(1, 1, 3, 1)
    board.move(4, 7, 3, 7)
    board.move(1, 6, 3, 6)
    board.move(3, 7, 2, 6)
    board.move(3, 1, 4, 1)
    board.move(2, 6, 1, 7)
    board.move(4, 1, 5, 1)
    i = 1
    print("undo \n --------------------------------------- \n ------------------------------------- \n ------------------------")
    while i < 8:
        i += 1
        board.undo()
    print(board)


def main():
    board = Board()
    fullgameStart = timeit.default_timer()
    for i in range(6):
        start = timeit.default_timer()
        if i % 2 == 0:
            print("My Turn:")
            beginCoord, endCoord = ai.calculateMove(3, board, True, i)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)

        else:
            print("Computers Turn:")
            beginCoord, endCoord = ai.calculateMove(3, board, False, i)
            board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)

        # Your statements here
        print(board)
        print("current game state:", ai.evaluation(board))
        stop = timeit.default_timer()

        print('Time: ', stop - start)
        print(board.isCheckmate)
        if board.isCheckmate:
            print("checkmate")
            break
    fullgameEnd = timeit.default_timer()
    print('Time: ', fullgameEnd - fullgameStart)


def checkmateTest():
    board = Board()
    board.board = TB.checkSetup
    board.move(7, 7, 7, 6)
    board.board[2][5] = "r"
    board.board[2][7] = "r"
    board.board[3][0] = "R"
    board.move(0, 7, 0, 6)
    print("okay")
    print(board.isInCheckmate(True))
    print(board.board)


def getAllValidMovesTest():
    board = Board()
    board.clearBoard()
    board.board[1][0] = "P"
    board.board[6][0] = "p"
    board.board[1][7] = "k"
    board.board[6][7] = "K"
    board.move(1, 0, 0, 0)
    board.move(6, 0, 7, 0)

    print(board)


if __name__ == "__main__":
    main()
    # test()
    #checkmateTest()
    #getAllValidMovesTest()

