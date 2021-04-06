import unittest
import pytest
from board import *
from testboards import Testboards as TB


class TestboardWhiteBishops(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Bishop Testing
    # White Bishop moves
    def test_UnrestrictedWhiteBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "B"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingWhiteBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "B"
        self.board.board[1][0] = "b"
        self.board.board[0][7] = "b"
        self.board.board[6][1] = "b"
        self.board.board[7][6] = "b"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedWhiteBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "B"
        self.board.board[1][0] = "B"
        self.board.board[0][7] = "B"
        self.board.board[6][1] = "B"
        self.board.board[7][6] = "b"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "5:2", "5:4", "6:5", "7:6", "3:2", "2:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardBlackBishops(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Black Bishop moves
    def test_UnrestrictedBlackBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "b"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingBlackBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "b"
        self.board.board[1][0] = "B"
        self.board.board[0][7] = "B"
        self.board.board[6][1] = "B"
        self.board.board[7][6] = "B"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedBlackBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "b"
        self.board.board[1][0] = "b"
        self.board.board[0][7] = "b"
        self.board.board[6][1] = "b"
        self.board.board[7][6] = "B"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "5:2", "5:4", "6:5", "7:6", "3:2", "2:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardMiscBishops(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_corneredBishops(self):
        self.board.clearBoard()
        self.board.board[0][0] = "B"
        self.board.board[7][7] = "b"
        self.board.board[0][7] = "b"
        self.board.board[7][0] = "B"
        actual1 = str(self.board.getPossibleMoves(0, 0, self.board.board))
        results1 = ["1:1", "2:2", "3:3", "4:4", "5:5", "6:6", "7:7"]
        for i in results1:
            self.assertIn(i, actual1)
        self.assertEqual(len(results1) * 5, len(actual1))
        # second
        actual2 = str(self.board.getPossibleMoves(0, 7, self.board.board))
        results2 = ["6:1", "5:2", "4:3", "3:4", "2:5", "1:6", "7:0"]
        for i in results2:
            self.assertIn(i, actual2)
        self.assertEqual(len(results2) * 5, len(actual2))
        # third
        actual3 = str(self.board.getPossibleMoves(7, 0, self.board.board))
        results3 = ["6:1", "5:2", "4:3", "3:4", "2:5", "1:6", "0:7"]
        for i in results3:
            self.assertIn(i, actual3)
        self.assertEqual(len(results3) * 5, len(actual3))
        # fourth
        actual4 = str(self.board.getPossibleMoves(7, 7, self.board.board))
        results4 = ["1:1", "2:2", "3:3", "4:4", "5:5", "6:6", "0:0"]
        for i in results4:
            self.assertIn(i, actual4)
        self.assertEqual(len(results4) * 5, len(actual4))


if __name__ == '__main__':
    unittest.main()
