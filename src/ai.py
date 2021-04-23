from board import *
from extra import *
import random


PieceValues = {'P': 100, 'N': 320, 'B': 330, 'R': 500, 'Q': 900, 'K': 10000}

opening1 = ['6444','1434', '7655', '0122', '7242']
# some evaluation conditions
# A queen versus two rooks

#     In the middlegame, they are equal
#     In the endgame, the two rooks are somewhat more powerful. With no other pieces on the board, two rooks are equal to a queen and a pawn

# A rook versus two minor pieces

#     In the opening and middlegame, a rook and two pawns are weaker than two bishops; equal to or slightly weaker than a bishop and knight; and equal to two knights
#     In the endgame, a rook and one pawn are equal to two knights; and equal to or slightly weaker than a bishop and knight. A rook and two pawns are equal to two bishops

# Bishops are often more powerful than rooks in the opening. Rooks are usually more powerful than bishops in the middlegame, and rooks dominate the minor pieces in the endgame

def calculateMove(depth, board, white, moveNumber):
    if moveNumber <= 5:
        opening = opening1[moveNumber]
        return Coordinate(int(opening[0]),int(opening[1])), Coordinate(int(opening[2]), int(opening[3]))
    else:
        minimaxRoot(depth,board, white)


def minimaxRoot(depth, board, white):
    validMoves = board.getAllValidMoves()
    bestMoveValue = -20000
    for piece in validMoves:
        for move in piece[1]:
            # First move of the tree
            move1 = board.move(piece[0].row, piece[0].column, move.row, move.column)
            moveBeginCoord = Coordinate(piece[0].row, piece[0].column)
            moveEndCoord = Coordinate(move.row, move.column)
            # this calls the minimax function and checks if the value returned by minimax is higher than bestMoveValue
            value = max(bestMoveValue, minimax(depth - 1, board, not white))
            board.undo()
            board.isWhitePlayerTurn = not board.isWhitePlayerTurn
            if(value > bestMoveValue):
                bestMoveValue = value
                bestMoveBeginCoord = moveBeginCoord
                bestMoveEndCoord = moveEndCoord
                print("Evaluation score: ", bestMoveValue)
                print("Begin Coordination", bestMoveBeginCoord)
                print("End Coordination", bestMoveEndCoord)
    return bestMoveBeginCoord, bestMoveEndCoord


def minimax(depth, board, white):
    if(depth == 0):
        return evaluation(board)

    possibleMoves = board.getAllValidMoves()
    if(white):
        bestMove = -20000
        for piece in possibleMoves:
            # if you want to see every board in the tree uncomment the print statements
            for move in piece[1]:
                # print(piece[0].row, piece[0].column, move.row, move.column)
                board.move(piece[0].row, piece[0].column, move.row, move.column)
                bestMove = max(bestMove, minimax(depth - 1, board, not white))
                print(bestMove)
                # print(board)
                board.undo()
                board.isWhitePlayerTurn = not board.isWhitePlayerTurn

        return bestMove
    else:
        bestMove = 20000
        for piece in possibleMoves:
            for move in piece[1]:
                # print(piece[0].row, piece[0].column, move.row, move.column)
                board.move(piece[0].row, piece[0].column, move.row, move.column)
                bestMove = min(bestMove, minimax(depth - 1, board, not white))
                print(bestMove)
                # print(board)
                board.undo()
                board.isWhitePlayerTurn = not board.isWhitePlayerTurn
        return bestMove


def evaluation(board):
    evaluationBlack = 0
    evaluationWhite = 0
    for row in range(8):
        for col in range(8):
            if board.board[row][col] != '.':
                moveAmount = len(board.getPossibleMoves(row, col, board.board))
                if board.board[row][col].islower():
                    evaluationBlack += moveAmount * 5
                    evaluationBlack += PieceValues.get((board.board[row][col]).upper())
                else:
                    evaluationWhite += moveAmount * 5
                    evaluationWhite += PieceValues.get((board.board[row][col]))

    return evaluationWhite - evaluationBlack

def isEndGame(board):
    heavyPieces = 0
    for row in range(8):
        for col in range(8):
            if board[row][col].upper() != '.' and board[row][col].upper() != 'P':
                heavyPieces += 1
    if heavyPieces < 6:
        return True
    else:
        return False


def pawnEvaluation(board, row, col):
    pass


def PlayRandomMove(validMoves):
    move = random.randint(0, len(validMoves)-1)
    piece = validMoves[move][0]
    nextMove = random.randint(0, len(validMoves[move][1])-1)
    nextLocation = validMoves[move][1][nextMove]
    return piece, nextLocation
