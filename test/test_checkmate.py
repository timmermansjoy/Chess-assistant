import unittest
import pytest
from src.abstraction.scripts.board import Board
from src.abstraction.scripts.testboards import Testboards as TB
from copy import deepcopy


class CheckmateTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_notCheckmate(self):
        self.board.board = deepcopy(TB.checkSetup)
        print(self.board)
        self.board.move(7, 7, 7, 6)
        self.board.board[2][5] = "r"
        self.board.board[2][7] = "r"
        self.board.board[3][0] = "R"
        self.board.move(0, 7, 0, 6)
        self.assertEqual(self.board.isInCheckmate(True), False)

    def test_isCheckmate(self):
        self.board.board = deepcopy(TB.checkSetup)
        self.board.move(7, 7, 7, 6)
        self.board.board[2][5] = "r"
        self.board.board[2][7] = "r"
        self.board.move(0, 7, 0, 6)
        self.assertEqual(self.board.isInCheckmate(True), True)

    def test_isStalemate(self):
        self.board.board = TB.AIFreeMate
        self.board.move(0, 3, 0, 4)
        self.board.move(1, 7, 1, 4)
        self.board.move(0, 4, 0, 3)
        self.board.move(1, 0, 1, 1)
        print(self.board.board)
        self.assertEqual(self.board.isInCheckmate(True), False)

    # Black

    def test_notCheckmateBlack(self):
        self.board.board = deepcopy(TB.checkSetup)
        self.board.move(1, 2, 0, 3)
        self.assertEqual(self.board.isInCheckmate(False), False)

    def test_isCheckmateBlack(self):
        self.board.board = deepcopy(TB.checkSetup)
        self.board.move(1, 2, 0, 2)
        self.board.board[1][6] = "N"
        self.assertEqual(self.board.isInCheckmate(False), True)

    def test_knightsCheckmateBlack(self):
        self.board.board = deepcopy(TB.checkSetup)
        self.board.move(1, 2, 0, 2)
        self.board.board[1][6] = "N"
        self.board.board[2][4] = "N"
        self.board.board[0][2] = "."
        self.assertEqual(self.board.isInCheckmate(False), True)

    def test_knightsNotCheckmateBlack(self):
        self.board.board = deepcopy(TB.checkSetup)
        self.board.move(1, 2, 0, 2)
        self.board.board[1][6] = "N"
        self.board.board[2][3] = "N"
        self.board.board[0][2] = "."
        print(self.board)
        self.assertEqual(self.board.isInCheckmate(False), False)

    @pytest.mark.xfail
    def test_EnPassantPossibleNotCheckmate(self):
        self.board.board = deepcopy(TB.checkSetup4)
        self.board.move(6, 7, 5, 7)
        self.board.move(1, 1, 3, 1)
        print(self.board)
        self.assertEqual(self.board.isInCheckmate(True), False)
