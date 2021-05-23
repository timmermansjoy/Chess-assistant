from board import *
from extra import *
import random


PieceValues = {'P': 100, 'N': 320, 'B': 330, 'R': 500, 'Q': 900, 'K': 10000}

opening1 = ['6444', '1434', '7655', '0122', '7531']
pawnBoardEvalOpening = np.array([
    [0.90, 0.95, 1.05, 1.10, 1.10, 1.05, 0.95, 0.90],
    [0.90, 0.95, 1.05, 1.15, 1.15, 1.05, 0.95, 0.90],
    [0.90, 0.95, 1.10, 1.20, 1.20, 1.10, 0.95, 0.90],
    [0.97, 1.03, 1.17, 1.27, 1.27, 1.17, 1.03, 0.97],
    [1.06, 1.12, 1.25, 1.40, 1.40, 1.25, 1.12, 1.06]
])
pawnBoardEvalEndGame = np.array([
    [1.20, 1.05, 0.95, 0.90, 0.90, 0.95, 1.05, 1.20],
    [1.20, 1.05, 0.95, 0.90, 0.90, 0.95, 1.05, 1.20],
    [1.25, 1.10, 1.00, 0.95, 0.95, 1.00, 1.10, 1.25],
    [1.33, 1.17, 1.07, 1.00, 1.00, 1.07, 1.17, 1.33],
    [1.45, 1.29, 1.16, 1.05, 1.05, 1.16, 1.29, 1.45]
])
pawnMultipliers = np.array([
    [1.05, 1.15, 1.30],
    [1.30, 1.35, 1.55],
    [2.1, 1.75, 2]
])

PAWN_TABLE = np.array([
    [ 0,  0,  0,  0,  0,  0,  0,  0],
    [ 5, 10, 10,-20,-20, 10, 10,  5],
    [ 5, -5,-10,  0,  0,-10, -5,  5],
    [ 0,  0,  0, 20, 20,  0,  0,  0],
    [ 5,  5, 10, 25, 25, 10,  5,  5],
    [10, 10, 20, 30, 30, 20, 10, 10],
    [50, 50, 50, 50, 50, 50, 50, 50],
    [ 0,  0,  0,  0,  0,  0,  0,  0]
])

KNIGHT_TABLE = np.array([
    [-50, -40, -30, -30, -30, -30, -40, -50],
    [-40, -20,   0,   5,   5,   0, -20, -40],
    [-30,   5,  10,  15,  15,  10,   5, -30],
    [-30,   0,  15,  20,  20,  15,   0, -30],
    [-30,   5,  15,  20,  20,  15,   0, -30],
    [-30,   0,  10,  15,  15,  10,   0, -30],
    [-40, -20,   0,   0,   0,   0, -20, -40],
    [-50, -40, -30, -30, -30, -30, -40, -50]
])

BISHOP_TABLE = np.array([
    [-20, -10, -10, -10, -10, -10, -10, -20],
    [-10,   5,   0,   0,   0,   0,   5, -10],
    [-10,  10,  10,  10,  10,  10,  10, -10],
    [-10,   0,  10,  10,  10,  10,   0, -10],
    [-10,   5,   5,  10,  10,   5,   5, -10],
    [-10,   0,   5,  10,  10,   5,   0, -10],
    [-10,   0,   0,   0,   0,   0,   0, -10],
    [-20, -10, -10, -10, -10, -10, -10, -20]
])

ROOK_TABLE = np.array([
    [ 0,  0,  0,  5,  5,  0,  0,  0],
    [-5,  0,  0,  0,  0,  0,  0, -5],
    [-5,  0,  0,  0,  0,  0,  0, -5],
    [-5,  0,  0,  0,  0,  0,  0, -5],
    [-5,  0,  0,  0,  0,  0,  0, -5],
    [-5,  0,  0,  0,  0,  0,  0, -5],
    [ 5, 10, 10, 10, 10, 10, 10,  5],
    [ 0,  0,  0,  0,  0,  0,  0,  0]
])

QUEEN_TABLE = np.array([
    [-20, -10, -10, -5, -5, -10, -10, -20],
    [-10,   0,   5,  0,  0,   0,   0, -10],
    [-10,   5,   5,  5,  5,   5,   0, -10],
    [  0,   0,   5,  5,  5,   5,   0,  -5],
    [ -5,   0,   5,  5,  5,   5,   0,  -5],
    [-10,   0,   5,  5,  5,   5,   0, -10],
    [-10,   0,   0,  0,  0,   0,   0, -10],
    [-20, -10, -10, -5, -5, -10, -10, -20]
])

# some evaluation conditions
# A queen versus two rooks

#     In the middlegame, they are equal
#     In the endgame, the two rooks are somewhat more powerful. With no other pieces on the board, two rooks are equal to a queen and a pawn

# A rook versus two minor pieces

#     In the opening and middlegame, a rook and two pawns are weaker than two bishops; equal to or slightly weaker than a bishop and knight; and equal to two knights
#     In the endgame, a rook and one pawn are equal to two knights; and equal to or slightly weaker than a bishop and knight. A rook and two pawns are equal to two bishops

# Bishops are often more powerful than rooks in the opening. Rooks are usually more powerful than bishops in the middlegame, and rooks dominate the minor pieces in the endgame


def calculateMove(depth, board, white, moveNumber=5):
    if moveNumber < 5:
        opening = opening1[moveNumber]
        return Coordinate(int(opening[0]), int(opening[1])), Coordinate(int(opening[2]), int(opening[3]))
    else:
        return minimaxRoot(depth, board, white)


def minimaxRoot(depth, board, white):
    validMoves = board.getAllValidMoves()
    if white:
        bestMoveValue = -20000
    else:
        bestMoveValue = 20000
    bestMoveBeginCoord = Coordinate(0, 0)
    bestMoveEndCoord = Coordinate(0, 0)
    for piece in validMoves:
        for move in piece[1]:
            # First move of the tree
            try:
                board.move(piece[0].row, piece[0].column, move.row, move.column)
                moveBeginCoord = Coordinate(piece[0].row, piece[0].column)
                moveEndCoord = Coordinate(move.row, move.column)
                # this calls the minimax function and checks if the value returned by minimax is higher than bestMoveValue
                if white:
                    value = max(bestMoveValue, minimax(depth - 1, board, -20000, 20000, not white))
                    if(value > bestMoveValue):
                        bestMoveValue = value
                        bestMoveBeginCoord = moveBeginCoord
                        bestMoveEndCoord = moveEndCoord
                else:
                    value = min(bestMoveValue, minimax(depth - 1, board, -20000, 20000, not white))
                    if(value < bestMoveValue):
                        bestMoveValue = value
                        bestMoveBeginCoord = moveBeginCoord
                        bestMoveEndCoord = moveEndCoord
                
                # print(board)
                # print(value)
                board.undo()
                board.isWhitePlayerTurn = not board.isWhitePlayerTurn
    
            except Exception as e:
                pass
                # print("non valid move in root:", piece[0].row, piece[0].column, move.row, move.column, e)
                # print(board)
    print("Best move Evaluation score: ", bestMoveValue)
    print("Begin Coordination", bestMoveBeginCoord)
    print("End Coordination", bestMoveEndCoord)
    return bestMoveBeginCoord, bestMoveEndCoord


def minimax(depth, board, alpha, beta, white):
    if(depth == 0 or board.isCheckmate):
        # print(board)
        # print(ev)
        return evaluation(board.board)
    possibleMoves = board.getAllValidMoves()
    if(white):
        bestMove = -20000
        for piece in possibleMoves:
            # if you want to see every board in the tree uncomment the print statements
            for move in piece[1]:
                try:
                    # print(piece[0].row, piece[0].column, move.row, move.column)
                    board.move(piece[0].row, piece[0].column, move.row, move.column)
                    value = minimax(depth - 1, board, alpha, beta, not white)
                    bestMove = max(bestMove, value)
                    # print(bestMove)
                    # print(board)
                    board.undo()
                    board.isWhitePlayerTurn = not board.isWhitePlayerTurn
                    alpha = max(alpha, bestMove)
                    if beta <= alpha:
                        # print("alphabeta")
                        return bestMove
                except Exception as e:
                    # print("non valid move in minimax if:", piece[0].row, piece[0].column, move.row, move.column , e )
                    # print(board)
                    pass

        return bestMove
    else:
        bestMove = 20000
        for piece in possibleMoves:
            for move in piece[1]:
                try:
                    # print(piece[0].row, piece[0].column, move.row, move.column)
                    board.move(piece[0].row, piece[0].column, move.row, move.column)
                    value = minimax(depth - 1, board, alpha, beta, not white)
                    bestMove = min(bestMove, value)
                    # print(bestMove)
                    # print(board)
                    board.undo()
                    board.isWhitePlayerTurn = not board.isWhitePlayerTurn
                    beta = min(beta, bestMove)
                    if beta <= alpha:
                        # print("alphabeta")
                        return bestMove
                except Exception as e:
                    # print("non valid move in minimax else:", piece[0].row, piece[0].column, move.row, move.column, e)
                    # print(board)
                    pass
        return bestMove


def evaluation(board):
    score = 0
    for row in range(8):
        for col in range(8):
            if board[row][col] != '.':
                piece = board[row][col]
                if piece.upper() == 'P':
                    positionTable = PAWN_TABLE
                elif piece.upper() == 'N':
                    positionTable = KNIGHT_TABLE
                elif piece.upper() == 'B':
                    positionTable = BISHOP_TABLE
                elif piece.upper() == 'R':
                    positionTable = ROOK_TABLE
                elif piece.upper() == 'Q':
                    positionTable = QUEEN_TABLE
                if piece.isupper():
                    positionTable = np.flip(positionTable)
                    score += positionTable[row][col]
                    score += PieceValues.get(piece.upper())
                else:
                    score -= positionTable[row][col]
                    score -= PieceValues.get(piece.upper())

    return score


def isEndGame(board):
    heavyPieces = board.getHeavyPieces()
    if heavyPieces < 6:
        return True
    else:
        return False


def pawnEvaluation(board, row, col):
    value = 100
    target = 'p'
    oposition = 'P'
    if board.board[row][col].isupper():
        target = 'P'
        oposition = 'p'
        row = row - 5
    isolated = False
    connected = False
    passed = True
    for i in range(8):
        try:
            if board.board[i][col - 1] == target:
                connected = True
        except:
            pass
        try:
            if board.board[i][col + 1] == target:
                connected = True
        except:
            pass
        if board.board[i][col] == oposition:
            passed = False
    if connected == False:
        isolated = True
    if 0 < row < 6:
        if isEndGame(board):
            value *= pawnBoardEvalEndGame[row - 1][col]
        else:
            value *= pawnBoardEvalOpening[row - 1][col]
    if 2 < row < 6:
        if passed:
            value *= pawnMultipliers[row - 3][2]
        if connected:
            value *= pawnMultipliers[row - 3][1]
        else:
            value *= pawnMultipliers[row - 3][0]
    return value


def PlayRandomMove(validMoves):
    move = random.randint(0, len(validMoves)-1)
    piece = validMoves[move][0]
    nextMove = random.randint(0, len(validMoves[move][1])-1)
    nextLocation = validMoves[move][1][nextMove]
    return piece, nextLocation
