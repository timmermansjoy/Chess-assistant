import unittest
import pytest
from board import *
from testboards import Testboards as TB


class TestboardWhitePawns(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Pawn Moves
    # White Pawn moves

    def test_WhitePawnNoTakeOptions(self):
        actual = str(self.board.getPossibleMoves(6, 3, self.board.board))
        results = ["5:3", "4:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnWithTakeOptions(self):
        self.board.board = TB.TwoPawnTakeOptions
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:2", "3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def test_WhitePawnWithOneTakeOneMove(self):
        self.board.board = TB.OneTakeOneMoveOption
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

        # Black Pawn moves


class TestboardBlackPawns(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_BlackPawnNoTakeOptions(self):
        actual = str(self.board.getPossibleMoves(1, 3, self.board.board))
        results = ["3:3", "2:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnWithTwoTakeOptions(self):
        self.board.board = TB.BlackTwoPawnTakeOptions
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["5:2", "5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def test_BlackPawnWithOneTakeOneMove(self):
        self.board.board = TB.BlackOneTakeOneMoveOption
        actual = str(self.board.getPossibleMoves(4, 3, self.board.board))
        results = ["5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))


class TestboardPawnsEnPassant(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_BlackPawnEnPassentLeft(self):
        self.board.board = TB.enPassentStartBoard2
        self.whiteMove = True
        self.board.move(6, 1, 4, 1)
        actual = str(self.board.getPossibleMoves(4, 2, self.board.board))
        result = ["5:2", "5:1"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnEnPassentRight(self):
        self.board.board = TB.enPassentStartBoard2
        self.whiteMove = True
        self.board.move(6, 3, 4, 3)
        actual = str(self.board.getPossibleMoves(4, 2, self.board.board))
        result = ["5:2", "5:3"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnNoEnPassentLeft(self):
        self.board.board = TB.noEnPassentStartBoard3
        self.whiteMove = True
        self.board.move(7, 2, 5, 0)
        actual = str(self.board.getPossibleMoves(4, 2, self.board.board))
        result = ["5:2"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

    def test_BlackPawnNoEnPassentRight(self):
        self.board.board = TB.noEnPassentStartBoard4
        self.whiteMove = True
        self.board.move(7, 3, 6, 3)
        actual = str(self.board.getPossibleMoves(4, 2, self.board.board))
        result = ["5:2"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

    def test_WhitePawnEnPassentLeft(self):
        # self.board.board = TB.enPassentStartBoard1
        # self.whiteMove = False
        self.board.move(6, 1, 4, 1)
        self.board.move(1, 7, 2, 7)
        self.board.move(4, 1, 3, 1)
        self.board.move(1, 0, 3, 0)
        actual = str(self.board.getPossibleMoves(3, 1, self.board.board))
        result = ["2:1", "2:0"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnEnPassentRight(self):
        # self.board.board = TB.enPassentStartBoard1
        # self.whiteMove = False
        self.board.move(6, 1, 4, 1)
        self.board.move(1, 7, 2, 7)
        self.board.move(4, 1, 3, 1)
        self.board.move(1, 2, 3, 2)
        actual = str(self.board.getPossibleMoves(3, 1, self.board.board))
        result = ["2:1", "2:2"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnNoEnPassentLeft(self):
        self.board.board = TB.noEnPassentStartBoard1
        self.whiteMove = False
        self.board.isWhitePlayerTurn = False
        self.board.move(0, 0, 1, 0)
        actual = str(self.board.getPossibleMoves(3, 1, self.board.board))
        result = ["2:1"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

    def test_WhitePawnNoEnPassentRight(self):
        self.board.board = TB.noEnPassentStartBoard2
        self.whiteMove = False
        self.board.isWhitePlayerTurn = False
        self.board.move(0, 3, 1, 2)
        actual = str(self.board.getPossibleMoves(3, 1, self.board.board))
        result = ["2:1"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

    def test_PromitionCheck(self):
        self.board.clearBoard()
        self.board.board[1][0] = "P"
        self.board.board[6][0] = "p"
        self.board.board[1][7] = "k"
        self.board.board[6][7] = "K"
        self.board.move(1, 0, 0, 0)
        self.board.move(6, 0, 7, 0)
        self.assertEqual("Q", self.board.board[0][0])
        self.assertEqual("q", self.board.board[7][0])

    def test_actuallyTakingEnPassant(self):
        self.board.move(6, 1, 4, 1)
        self.board.move(1, 7, 2, 7)
        self.board.move(4, 1, 3, 1)
        self.board.move(1, 2, 3, 2)
        self.board.move(3, 1, 2, 2)
        assert self.board.board[2][2] == "P"
        assert self.board.board[3][2] == "."
        assert self.board.board[3][1] == "."


if __name__ == '__main__':
    unittest.main()
