import unittest
import pytest
from src.abstraction.scripts.board import Board
from src.abstraction.scripts.testboards import Testboards as TB


class CheckmateTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_notCheckmate(self):
        self.board.board = TB.checkSetup
        print(self.board)
        self.board.move(7, 7, 7, 6)
        self.board.board[2][5] = "r"
        self.board.board[2][7] = "r"
        self.board.board[3][0] = "R"
        self.board.move(0, 7, 0, 6)
        self.assertEqual(self.board.isInCheckmate(True), False)

    def test_isCheckmate(self):
        self.board.board = TB.anotherCheckSetup
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

    #Black

    def test_notCheckmateBlack(self):
        self.board.board = TB.checkSetup
