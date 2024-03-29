from src.abstraction.scripts.board import Board
import unittest
import pytest
from testboards import Testboards as TB
import src.abstraction.scripts.ai as ai
from src.abstraction.scripts.extra import Coordinate


class TestAI(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # WhiteMoves
    def test_freeMate(self):
        self.board.board = TB.AIFreeMate
        self.board.move(0, 3, 0, 2)
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        # 2 fields give checkmate: either 0:0 or 1:2
        if endCoord.row == 0:
            self.assertEqual(endCoord.column, 0)
        else:
            self.assertEqual(endCoord.column, 2)

    @pytest.mark.xfail
    def test_rookOverBishopWhite(self):
        self.board.board = TB.AIFreeWhitePiece
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        self.board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
        actual = Coordinate(3, 2)
        self.assertEqual(endCoord.row, actual.row)
        self.assertEqual(endCoord.column, actual.column)

    def test_noWastedMaterial(self):
        self.board.board = TB.AINotWasteMaterial
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        notActual = Coordinate(2, 1)
        result = notActual == beginCoord

        self.assertEqual(result, False)

    # prioritises taking queen over immediate checkmate on h7
    @pytest.mark.xfail
    def test_mateButFreeQueen(self):
        self.board.board = TB.AIMateButMaterial
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        result = Coordinate(1, 7)
        self.assertEqual(result, endCoord)

    # Black
    # @pytest.mark.xfail
    # Wants to move 1:0 to 0:0 despite no piece being present on 1:0
    # already moves in calculatedMove, is this intended?
    # also gives away free queen
    @pytest.mark.xfail
    def test_rookOverBishop(self):
        board = Board()
        board.board = TB.AIFreePiece
        board.move(2, 0, 1, 1)
        print(board)
        beginCoord, endCoord = ai.calculateMove(3, board, False)
        print(board)
        print(board.getAllValidMoves())
        print(beginCoord, endCoord)
        board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
        actual = Coordinate(3, 2)
        self.assertEqual(endCoord, actual)

    @pytest.mark.xfail
    def test_freeBishopMate(self):
        self.board.board = TB.AIblackbisshopMate
        self.board.move(6, 0, 7, 0)
        print(self.board)
        beginCoord, endCoord = ai.calculateMove(3, self.board, False)
        self.board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
        actualEnd = Coordinate(1, 6)
        actualStart = Coordinate(2, 7)
        print(self.board)
        self.assertEqual(endCoord, actualEnd)
        self.assertEqual(beginCoord, actualStart)

    # prefers bishop over checkmate
    @pytest.mark.xfail
    def test_mateBlackButFreeBishop(self):
        self.board.board = TB.AIMateBlackButMaterial
        self.board.isWhitePlayerTurn = not self.board.isWhitePlayerTurn
        beginCoord, endCoord = ai.calculateMove(3, self.board, False)
        actualEnd = Coordinate(6, 6)
        self.assertEqual(actualEnd, endCoord)

    # prefers pawn over checkmate
    @pytest.mark.xfail
    def test_mateBlackButFreePawn(self):
        self.board.board = TB.AIMateBlackButMaterial
        self.board.board[5][1] = "P"
        self.board.isWhitePlayerTurn = not self.board.isWhitePlayerTurn
        beginCoord, endCoord = ai.calculateMove(3, self.board, False)
        actualEnd = Coordinate(6, 6)
        self.assertEqual(actualEnd, endCoord)


if __name__ == '__main__':
    unittest.main()
