from board import *
from extra import *
import random


PieceValues = {'P': 100, 'N': 320, 'B': 330, 'R': 500, 'Q': 900, 'K': 10000}


def minimaxRoot(depth, board, white):
    validMoves = board.getAllValidMoves()
    bestMoveValue = -20000
    for piece in validMoves:
        for move in piece[1]:
            #First move of the tree
            move1 = board.move(piece[0].row, piece[0].column, move.row, move.column)
            moveBeginCoord = Coordinate(piece[0].row, piece[0].column)
            moveEndCoord = Coordinate(move.row, move.column)
            #this calls the minimax function and checks if the value returned by minimax is higher than bestMoveValue
            value = max(bestMoveValue, minimax(depth - 1, board, not white))
            board.undo()
            board.isWhitePlayerTurn = not board.isWhitePlayerTurn
            if( value > bestMoveValue):
                bestMoveValue = value
                bestMoveBeginCoord = moveBeginCoord
                bestMoveEndCoord = moveEndCoord
                print("Evaluation score: " ,bestMoveValue)
                print("Begin Coordination", bestMoveBeginCoord)
                print("End Coordination", bestMoveEndCoord)
    return bestMoveBeginCoord, bestMoveEndCoord

def minimax(depth, board, white):
    if(depth == 0):
        return evaluation(board, True)
        
    possibleMoves = board.getAllValidMoves()
    if(white):
        bestMove = -20000
        for piece in possibleMoves:
            #if you want to see every board in the tree uncomment the print statements
            for move in piece[1]:
                # print(piece[0].row, piece[0].column, move.row, move.column)
                board.move(piece[0].row, piece[0].column, move.row, move.column)
                bestMove = max(bestMove,minimax(depth - 1, board, not white))
                # print(bestMove)
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
                bestMove = min(bestMove,minimax(depth - 1, board, not white))
                # print(bestMove)
                # print(board)
                board.undo()
                board.isWhitePlayerTurn = not board.isWhitePlayerTurn
        return bestMove


def evaluation(board, white):
    evaluationBlack = 0
    evaluationWhite = 0
    for row in range(8):
        for col in range(8):
            if board.board[row][col] != '.':
                if board.board[row][col].islower():
                    evaluationBlack += PieceValues.get((board.board[row][col]).upper())
                else:
                    evaluationWhite += PieceValues.get((board.board[row][col]).upper())

    if white:
        return evaluationWhite - evaluationBlack
    else:
        return evaluationBlack - evaluationWhite


def PlayRandomMove(validMoves):
    move = random.randint(0, len(validMoves)-1)
    piece = validMoves[move][0]
    nextMove = random.randint(0, len(validMoves[move][1])-1)
    nextLocation = validMoves[move][1][nextMove]
    return piece, nextLocation
