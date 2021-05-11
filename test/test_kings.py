import unittest
import pytest
from src.abstraction.scripts.board import Board
from testboards import Testboards as TB
import numpy as np

class TestboardKing(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_MoveKing(self):
        self.board.board = TB.kingMoves
        actual = str(self.board.getPossibleMoves(3, 4, self.board.board))
        results = ["2:3", "2:4", "2:5", "4:3", "4:4", "4:5", "3:3", "3:5"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_UnderThreatKingMoves(self):
        self.board.clearBoard()
        self.board.board[0][0] = "K"
        self.board.board[1][1] = "r"
        self.board.updateBlackThreat()
        self.board.updateWhiteThreat()
        actual = str(self.board.getPossibleMoves(0, 0, self.board.board))
        results = ["1:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


if __name__ == '__main__':
    unittest.main()
