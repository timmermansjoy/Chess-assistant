from board import *
from extra import *
import random


PieceValues = {'P': 100, 'N': 320, 'B': 330, 'R': 500, 'Q': 900, 'K': 10000}


def CalculateBestMove(depth, board):
    board.getPossibleMoves()


def evaluation(board, white):
    evaluationBlack = 0
    evaluationWhite = 0
    for row in range(8):
        for col in range(8):
            if board[row][col] != '.':
                if board[row][col].islower():
                    evaluationBlack += PieceValues.get((board[row][col]).upper())
                else:
                    evaluationWhite += PieceValues.get((board[row][col]).upper())

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
