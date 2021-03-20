import numpy as np


class Testboards:
    noMoveForPawn = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "P"],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    oneMoveForPawn = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "B"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", ".", "N", "R"]
    ])

    captureForPawn = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", ".", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "p", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    noCaptureForPawn = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    rookAccessTest = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", "R", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    knightAccessTest = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", "n", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    bishopAccessTest = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", "b", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    TwoPawnTakeOptions = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", ".", "p", ".", ".", "."],
        [".", ".", ".", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    OneTakeOneMoveOption = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "P", ".", "p", ".", ".", "."],
        [".", ".", ".", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    BlackTwoPawnTakeOptions = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", "p", ".", ".", ".", "."],
        [".", ".", "P", ".", "P", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])

    BlackOneTakeOneMoveOption = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "P", ".", "p", ".", ".", "."],
        [".", ".", ".", "p", ".", ".", ".", "."],
        [".", ".", "p", ".", "P", ".", "P", "."],
        ["P", "P", "P", "P", "P", "P", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"]

    ])

    queenMoves = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", "Q", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", ".", "K", "B", "N", "R"],
    ])

    queenNoMovesLeft = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["Q", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    queenNoMovesUpOrLeft = np.array([
        ["Q", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    queenNoMovesDownOrRight = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "Q"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    queenNoMoves = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])