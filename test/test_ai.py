import unittest
import pytest
from board import *
from testboards import Testboards as TB
import ai

class TestAI(unittest.TestCase):

    def setUp(self):
        self.board = Board()
#WhiteMoves
    def test_freeMate(self):
        self.board.board = TB.AIFreeMate
        self.board.move(0, 3, 0, 2)
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        #2 fields give checkmate: either 0:0 or 1:2
        if endCoord.row == 0:
            self.assertEqual(endCoord.column, 0)
        else:
            self.assertEqual(endCoord.column, 2)

    def test_rookOverBishopWhite(self):
        self.board.board = TB.AIFreeWhitePiece
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        self.board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
        actual = Coordinate(3,2)
        self.assertEqual(endCoord, actual)

    def test_noWastedMaterial(self):
        self.board.board = TB.AINotWasteMaterial
        beginCoord, endCoord = ai.calculateMove(3, self.board, True)
        notActual = Coordinate(2,1)
        result = notActual == beginCoord

        self.assertEqual(result, False)
        #Black

    def test_rookOverBishop(self):
        self.board.board = TB.AIFreePiece
        self.board.move(2,0,1,1)
        beginCoord, endCoord = ai.calculateMove(3, self.board, False)
        self.board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
        actual = Coordinate(3,2)
        self.assertEqual(endCoord, actual)

    @pytest.mark.xfail
    def test_freeBishopMate(self):
        self.board.board = TB.AIblackbisshopMate
        self.board.move(6, 0, 7, 0)
        print(self.board)
        beginCoord, endCoord = ai.calculateMove(3, self.board, False)
        self.board.move(beginCoord.row, beginCoord.column, endCoord.row, endCoord.column)
        actualEnd = Coordinate(1,6)
        actualStart = Coordinate(2,7)
        print(self.board)
        self.assertEqual(endCoord, actualEnd)
        self.assertEqual(beginCoord, actualStart)


if __name__ == '__main__':
    unittest.main()
