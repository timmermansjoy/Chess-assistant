import unittest
import pytest
from board import *
from testboards import Testboards as TB


class TestboardWhiteRook(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Rook Testing
    # White Rook Test
    def test_UnrestrictedWhiteRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["4:4", "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingWhiteRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        self.board.board[3][3] = "n"
        self.board.board[4][4] = "p"
        self.board.board[4][2] = "r"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:3", "4:4", "4:2", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedWhiteRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        self.board.board[3][3] = "N"
        self.board.board[4][4] = "P"
        self.board.board[4][2] = "R"
        self.board.board[5][3] = "r"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["5:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

        # Black Rook Test


class TestboardBlackRook(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_UnrestrictedBlackRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "r"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["4:4", "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingBlackRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "r"
        self.board.board[3][3] = "N"
        self.board.board[4][4] = "P"
        self.board.board[4][2] = "R"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:3", "4:4", "4:2", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedBlackRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "r"
        self.board.board[3][3] = "n"
        self.board.board[4][4] = "p"
        self.board.board[4][2] = "r"
        self.board.board[5][3] = "R"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["5:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardMiscRook(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_corneredRooks(self):
        self.board.clearBoard()
        self.board.board[0][0] = "r"
        self.board.board[7][7] = "R"
        self.board.board[0][7] = "r"
        self.board.board[7][0] = "R"
        actual1 = str(self.board.getPossibleMoves(0, 0, self.board.board))
        results1 = ["0:1", "0:2", "0:3", "0:4", "0:5", "0:6", "1:0", "2:0", "3:0", "4:0", "5:0", "6:0", "7:0"]
        for i in results1:
            self.assertIn(i, actual1)
        self.assertEqual(len(results1) * 5, len(actual1))
        # second
        actual2 = str(self.board.getPossibleMoves(0, 7, self.board.board))
        results2 = ["0:1", "0:2", "0:3", "0:4", "0:5", "0:6", "7:7", "6:7", "5:7", "4:7", "3:7", "2:7", "1:7"]
        for i in results2:
            self.assertIn(i, actual2)
        self.assertEqual(len(results2) * 5, len(actual2))
        # third
        actual3 = str(self.board.getPossibleMoves(7, 0, self.board.board))
        results3 = ["7:6", "7:5", "7:4", "7:3", "7:2", "7:1", "1:0", "2:0", "3:0", "4:0", "5:0", "6:0", "0:0"]
        for i in results3:
            self.assertIn(i, actual3)
        self.assertEqual(len(results3) * 5, len(actual3))
        # fourth
        actual4 = str(self.board.getPossibleMoves(7, 7, self.board.board))
        results4 = ["7:6", "7:5", "7:4", "7:3", "7:2", "7:1", "0:7", "1:7", "2:7", "3:7", "4:7", "5:7", "6:7"]
        for i in results4:
            self.assertIn(i, actual4)
        self.assertEqual(len(results4) * 5, len(actual4))


if __name__ == '__main__':
    unittest.main()
