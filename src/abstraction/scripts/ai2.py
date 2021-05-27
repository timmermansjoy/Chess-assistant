import math
from enum import Enum
from node import Node


def sss(root, depth=4):
    OPEN = []
    OPEN.append((root, Status.LIVE, math.inf))
    while True:
        (N, status, h) = OPEN.pop()

        if N == root and status == Status.SOLVED:
            return h

        if status == Status.LIVE:
            N.get_children() = N.board.getAllValidMoves()
            if len(N.children == 0) or depth == 0:
                val = min(h, eval(N))
                N.value = val
                OPEN.append((N, Status.SOLVED, val))
            elif N.max:
                for C in N.children:
                    OPEN.append(C, Status.LIVE, h)
                    depth -= 1
            elif not N.max:
                OPEN.append(N.children[0], Status.LIVE, h)
                depth -= 1

        if status == Status.SOLVED:
            P = N.parent
            if N.max and P.children[-1] == N:
                P.value = h
                P.move = N.move
                OPEN.append(P, Status.SOLVED, h)
            elif N.max:
                OPEN.append((P.children[P.children.index(N) + 1], Status.LIVE, h))
            elif not N.max:
                P.value = h
                P.move = N.move
                OPEN.add(P, Status.SOLVED, h)
                for i in range(len(OPEN)):
                    N, status, h = OPEN[i]
                    if N in P.children:
                        OPEN.remove(OPEN[i])


def move(board, white):
    root = Node(white, board)
    best = sss(root)
    return best.move


class Status(Enum):
    LIVE = 0
    SOLVED = 1
