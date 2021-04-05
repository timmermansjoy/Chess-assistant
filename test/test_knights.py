import unittest
import pytest
from board import *
from testboards import Testboards as TB


class TestboardWhiteKnights(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Knight moves
    def test_UnrestrictedWhiteKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "N"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_CapturingWhiteKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "N"
        self.board.board[2][2] = 'n'
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_RestrictedWhiteKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "N"
        self.board.board[2][2] = 'n'
        self.board.board[2][4] = 'n'
        self.board.board[3][5] = 'N'
        self.board.board[5][5] = 'N'
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["2:2", "2:4", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(30, len(actual))

    # Black knight moves


class TestboardBlackKnights(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_UnrestrictedBlackKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "n"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_CapturingBlackKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "n"
        self.board.board[2][2] = 'N'
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_RestrictedBlackKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "n"
        self.board.board[2][2] = 'N'
        self.board.board[2][4] = 'N'
        self.board.board[3][5] = 'n'
        self.board.board[5][5] = 'n'
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["2:2", "2:4", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(30, len(actual))

    def test_corneredKnights(self):
        self.board.clearBoard()
        self.board.board[0][0] = "n"
        self.board.board[7][7] = "n"
        self.board.board[0][7] = "N"
        self.board.board[7][0] = "N"
        actual1 = str(self.board.getPossibleMoves(0, 0, self.board.board))
        results1 = ["2:1", "1:2"]
        for i in results1:
            self.assertIn(i, actual1)
        self.assertEqual(len(results1) * 5, len(actual1))
        actual2 = str(self.board.getPossibleMoves(0, 7, self.board.board))
        results2 = ["2:6", "1:5"]
        for i in results2:
            self.assertIn(i, actual2)
        self.assertEqual(len(results2) * 5, len(actual2))
        actual3 = str(self.board.getPossibleMoves(7, 0, self.board.board))
        results3 = ["5:1", "6:2"]
        for i in results3:
            self.assertIn(i, actual3)
        self.assertEqual(len(results3) * 5, len(actual3))
        actual4 = str(self.board.getPossibleMoves(7, 7, self.board.board))
        results4 = ["5:6", "6:5"]
        for i in results4:
            self.assertIn(i, actual4)
        self.assertEqual(len(results4) * 5, len(actual4))


if __name__ == '__main__':
    unittest.main()
