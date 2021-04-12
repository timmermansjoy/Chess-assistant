import random
from extra import *
import random


def PlayRandomMove(validMoves):
    move = random.randint(0, len(validMoves)-1)
    piece = validMoves[move][0]
    nextMove = random.randint(0, len(validMoves[move][1])-1)
    nextLocation = validMoves[move][1][nextMove]
    return piece, nextLocation
