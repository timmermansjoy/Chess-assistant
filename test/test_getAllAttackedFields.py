import unittest
import pytest
from src.abstraction.scripts.board import Board
from testboards import Testboards as TB
import numpy as np

class TestboardGetAllAttackedFields(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_getAllAttackingFieldsOfOnePiece(self):
        self.board.clearBoard()
        self.board.board[4][3] = "q"
        actual = str(self.board.getAllAttackedFields(False, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4",
                   "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_getAllAttackingFieldsOfTwoPieces(self):
        self.board.clearBoard()
        self.board.board[4][3] = "q"
        self.board.board[0][0] = "k"
        actual = str(self.board.getAllAttackedFields(False, self.board.board))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4",
                   "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3", "0:1",
                   "1:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_getAllAttackingFieldsOfStandardBoard(self):
        actual = str(self.board.getAllAttackedFields(False, self.board.board))
        results = ["2:0", "2:1", "2:2", "2:3", "2:4", "2:5", "2:6", "2:7"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_getFreeStyleBoard(self):
        self.board.clearBoard()
        self.board.board[7][0] = "Q"
        self.board.board[0][0] = "R"
        self.board.board[7][1] = "K"
        self.board.board[6][1] = "P"
        self.board.board[0][4] = "r"
        self.board.board[5][5] = "r"
        self.board.board[6][6] = "B"
        actual = str(self.board.getAllAttackedFields(True, self.board.board))
        results = ["1:0", "2:0", "3:0", "4:0", "5:0", "6:0", "0:1", "0:2", "0:3", "0:4", "5:2", "6:2", "7:2", "5:5",
                   "7:5", "5:7", "7:7"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


if __name__ == '__main__':
    unittest.main()
