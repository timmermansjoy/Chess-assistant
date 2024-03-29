import numpy as np


class Testboards:
    # pawns
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

    enPassentStartBoard1 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", "P", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", ".", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    enPassentStartBoard2 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", ".", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    noEnPassentStartBoard1 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        [".", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["p", "P", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", ".", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    noEnPassentStartBoard2 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", ".", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", "P", "p", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", ".", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    noEnPassentStartBoard3 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", "P", "p", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", ".", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    noEnPassentStartBoard4 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", ".", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

# rooks
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
# knights
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
# bishop
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
# queen
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
# King
    kingMoves = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", "K", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", ".", "B", "N", "R"],
    ])

    kingNoMovesLeft = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["K", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", ".", "B", "N", "R"],
    ])

    kingNoMovesUpOrLeft = np.array([
        ["Q", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", "P", "P", "."],
        ["P", "P", "P", "P", "P", ".", "B", "P"],
        ["R", "N", "B", "Q", "K", ".", "N", "R"],
    ])

    kingNoMovesRight = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "Q"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "K"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", ".", "B", "N", "R"],
    ])

    kingNoMoves = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", "P", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    Castle = np.array([
        ["r", ".", ".", ".", "k", ".", ".", "r"],
        ["p", "p", "Q", "p", "p", "p", ".", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", ".", "P", "P", "P", "P"],
        ["R", ".", ".", ".", "K", ".", ".", "R"],
    ])

    checkSetup = np.array([
        ["r", ".", ".", ".", "k", ".", ".", "r"],
        ["p", "p", "Q", "p", "p", "p", ".", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "K", "."],
        ["P", "P", "P", ".", "P", "P", "P", "P"],
        ["R", ".", ".", ".", ".", ".", ".", "R"],
    ])
    checkSetup2 = np.array([
        ["r", ".", ".", ".", "k", ".", ".", "r"],
        ["p", "p", "Q", "p", "p", "p", ".", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "K", "."],
        ["P", "P", "P", ".", "P", "P", "P", "P"],
        ["R", ".", ".", ".", ".", ".", ".", "R"],
    ])
    checkSetup3 = np.array([
        ["r", ".", ".", ".", "k", ".", ".", "r"],
        ["p", "p", "Q", "p", "p", "p", ".", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "p", "P", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "K", "."],
        ["P", "P", "P", ".", "P", "P", "P", "P"],
        ["R", ".", ".", ".", ".", ".", ".", "R"],
    ])
# AI boards
    AIFreeMate = np.array([
        [".", ".", ".", "K", ".", ".", ".", "."],
        ["q", ".", ".", ".", ".", ".", ".", "r"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "k"],
    ])
    AIFreePiece = np.array([
        [".", ".", ".", ".", ".", ".", "N", "K"],
        ["q", ".", ".", ".", ".", ".", "P", "P"],
        ["B", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "R", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "k", "."],
    ])
    AIFreeWhitePiece = np.array([
        [".", ".", ".", ".", ".", ".", "n", "k"],
        ["Q", ".", ".", ".", ".", ".", "p", "p"],
        ["b", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "r", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "K", "."],
    ])
    AINotWasteMaterial = np.array([
        [".", ".", ".", ".", ".", ".", "n", "k"],
        ["Q", ".", ".", ".", ".", ".", "p", "p"],
        [".", "N", ".", ".", ".", ".", ".", "."],
        [".", ".", "b", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", "K", ".", ".", ".", ".", ".", "."],
    ])
    AIblackbisshopMate = np.array([
        [".", ".", ".", ".", ".", ".", "n", "k"],
        [".", ".", ".", ".", ".", ".", ".", "p"],
        [".", "r", ".", ".", ".", ".", ".", "b"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "b", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["K", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
    ])
    AIMateButMaterial = np.array([
        [".", ".", ".", ".", ".", ".", "k", "."],
        ["Q", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "K", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["q", ".", ".", ".", ".", "R", ".", "."],
    ])
    AIMateBlackButMaterial = np.array([
        [".", ".", ".", ".", ".", ".", "k", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", "B", ".", ".", ".", ".", ".", "."],
        ["r", "q", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "K", "."],
    ])
# VISION

    VisionBoard1 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", "P", ".", "."],
        ["P", "P", "P", "P", "P", ".", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    VisionBoard2 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", ".", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", "p", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", "P", ".", "."],
        ["P", "P", "P", "P", "P", ".", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    VisionBoard3 = np.array([
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", ".", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", "p", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "."],
        [".", ".", ".", ".", ".", "P", ".", "."],
        ["P", "P", "P", "P", "P", ".", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    VisionBoard4 = np.array([
        ["r", "n", "b", ".", "k", "b", "n", "r"],
        ["p", "p", "p", "p", ".", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", "p", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", "P", "q"],
        [".", ".", ".", ".", ".", "P", ".", "."],
        ["P", "P", "P", "P", "P", ".", ".", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ])

    CheckmateInOne = np.array([
        [".", ".", ".", ".", ".", ".", ".", "r"],
        [".", ".", ".", ".", ".", ".", "b", "p"],
        [".", "p", ".", ".", ".", "k", ".", "."],
        [".", ".", ".", "B", "Q", ".", "p", "."],
        [".", ".", ".", "p", "P", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", ".", ".", "P", ".", "P", "P", "P"],
        ["R", "N", "B", ".", "K", ".", ".", "R"],
    ])
