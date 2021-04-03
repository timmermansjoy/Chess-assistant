import unittest
import pytest
from board import *
from testboards import Testboards as TB


class TestboardWhiteQueen(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Queen Tests
    # White Queen Tests
    def test_UnrestrictedWhiteQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "Q"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4",
                   "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingWhiteQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "Q"
        self.board.board[1][0] = "b"
        self.board.board[0][7] = "b"
        self.board.board[6][1] = "b"
        self.board.board[7][6] = "b"
        self.board.board[4][0] = "b"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4", "4:5",
                   "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedWhiteQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "Q"
        self.board.board[4][4] = "B"
        self.board.board[4][2] = "B"
        self.board.board[5][3] = "B"
        self.board.board[5][4] = "B"
        self.board.board[5][2] = "B"
        self.board.board[3][3] = "B"
        self.board.board[3][4] = "b"
        self.board.board[3][2] = "b"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "3:2"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardBlackQueen(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Black Queen Tests
    def test_UnrestrictedBlackQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "q"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4",
                   "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingBlackQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "q"
        self.board.board[1][0] = "B"
        self.board.board[0][7] = "B"
        self.board.board[6][1] = "B"
        self.board.board[7][6] = "B"
        self.board.board[4][0] = "B"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4", "4:5",
                   "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedBlackQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "q"
        self.board.board[4][4] = "q"
        self.board.board[4][2] = "q"
        self.board.board[5][3] = "q"
        self.board.board[5][4] = "b"
        self.board.board[5][2] = "p"
        self.board.board[3][3] = "n"
        self.board.board[3][4] = "B"
        self.board.board[3][2] = "B"
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:4", "3:2"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardMiscQueen(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_corneredQueen(self):
        self.board.clearBoard()
        self.board.board[0][0] = "Q"
        self.board.board[7][7] = "q"
        self.board.board[0][7] = "q"
        self.board.board[7][0] = "Q"
        actual1 = str(self.board.getPossibleMoves(0, 0, self.board.board))
        results1 = ["1:1", "2:2", "3:3", "4:4", "5:5", "6:6", "7:7", "0:1", "0:2", "0:3", "0:4", "0:5", "0:6", "1:0",
                    "2:0", "3:0", "4:0", "5:0", "6:0", "0:7"]
        for i in results1:
            self.assertIn(i, actual1)
        self.assertEqual(len(results1) * 5, len(actual1))
        # second
        actual2 = str(self.board.getPossibleMoves(0, 7, self.board.board))
        results2 = ["6:1", "5:2", "4:3", "3:4", "2:5", "1:6", "7:0", "0:1", "0:2", "0:3", "0:4", "0:5", "0:6", "0:0",
                    "6:7", "5:7", "4:7", "3:7", "2:7", "1:7"]
        for i in results2:
            self.assertIn(i, actual2)
        self.assertEqual(len(results2) * 5, len(actual2))
        # third
        actual3 = str(self.board.getPossibleMoves(7, 0, self.board.board))
        results3 = ["6:1", "5:2", "4:3", "3:4", "2:5", "1:6", "0:7", "7:6", "7:5", "7:4", "7:3", "7:2", "7:1", "1:0",
                    "2:0", "3:0", "4:0", "5:0", "6:0", "7:7"]
        for i in results3:
            self.assertIn(i, actual3)
        self.assertEqual(len(results3) * 5, len(actual3))
        # fourth
        actual4 = str(self.board.getPossibleMoves(7, 7, self.board.board))
        results4 = ["1:1", "2:2", "3:3", "4:4", "5:5", "6:6", "0:0", "7:6", "7:5", "7:4", "7:3", "7:2", "7:1", "7:0",
                    "1:7", "2:7", "3:7", "4:7", "5:7", "6:7"]
        for i in results4:
            self.assertIn(i, actual4)
        self.assertEqual(len(results4) * 5, len(actual4))


if __name__ == '__main__':
    unittest.main()
