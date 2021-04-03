import unittest
import pytest
from board import *
from testboards import Testboards as TB


# Basic structure per piece:
# 1 test with unrestricted movement
# 1 test with the option to take a piece and regular movement
# 1 test with both the option to take an enemy piece and restricted movement by a friendly piece
# TODO: tests with pieces on the edge of the board
# Basic structure per test:
# create a Board-object and give it a Board.board
# if needed, edit the board with the piece you want to test
# create a "Results"-array with all the expected moves
# create an "actual"-string using board.getPossibleMoves()
# verify all moves in "Results" are present in "actual"
# verify the length of "actual" (exactly 5 characters per possible moves)

class TestboardKnights(unittest.TestCase):

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


class TestboardRook(unittest.TestCase):

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


class TestboardPawns(unittest.TestCase):

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

        # Black Pawn moves

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


class TestboardBishops(unittest.TestCase):

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


class TestboardQueen(unittest.TestCase):

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
    # King Test


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
    # board tests


class TestboardMiscTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_NotationMoveKnight(self):
        self.assertEqual('N', self.board.board[7][1])
        self.assertEqual('.', self.board.board[5][2])
        self.board.notationMove("b1c3")
        self.assertEqual('.', self.board.board[7][1])
        self.assertEqual('N', self.board.board[5][2])

    def test_clearingBoard(self):
        self.board.clearBoard()
        for i in self.board.board:
            for j in i:
                self.assertEqual(j, ".")

    def test_placePiece(self):
        assert (self.board.board[3][1]) == '.'
        self.board.placePieceOnNotation('k', 'b5')
        assert (self.board.board[3][1]) == 'k'

    # method 1 of testing exceptions
    def test_placeWrongPiece(self):
        self.assertRaises(Exception, lambda: self.board.placePieceOnNotation('H', 'b5'))

    # method 2 of testing exceptions
    def test_placeOutsideBoard(self):
        with self.assertRaises(Exception):
            self.board.placePieceOnNotation('Q', 'm5')

        with self.assertRaises(Exception):
            self.board.placePieceOnNotation('k', 'a99')


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


class CastleTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()
        initialMoves = ["e2e4", "e7e5", "f1a6", "f8a3", "d2d4", "d7d5", "g1h3", "g8h6", "b1a3", "b8a6"]
        for i in initialMoves:
            coords = self.board.notationToCords(str(i))
            self.board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)

    def test_castleProperlyWhite(self):
        self.board.castling(True, False)
        resultBoard = np.array([
            ['r', '.', 'b', 'q', 'k', '.', '.', 'r'],
            ['p', 'p', 'p', '.', '.', 'p', 'p', 'p'],
            ['n', '.', '.', '.', '.', '.', '.', 'n'],
            ['.', '.', '.', 'p', 'p', '.', '.', '.'],
            ['.', '.', '.', 'P', 'P', '.', '.', '.'],
            ['N', '.', '.', '.', '.', '.', '.', 'N'],
            ['P', 'P', 'P', '.', '.', 'P', 'P', 'P'],
            ['R', '.', 'B', 'Q', '.', 'R', 'K', '.']
        ])
        for i in range(8):
            for j in range(8):
                self.assertEquals(resultBoard[i][j], self.board.board[i][j], str(i) + str(j))

    def test_castleFailsAfterKMoveWhite(self):
        self.board.move(7, 4, 6, 4)
        self.board.move(0, 4, 1, 4)
        self.board.move(6, 4, 7, 4)
        self.board.move(1, 4, 0, 4)
        with self.assertRaises(Exception):
            self.board.castling(True, False)

    def test_castleFailsAfterRMoveWhite(self):
        self.board.move(7, 7, 7, 6)
        self.board.move(0, 7, 0, 6)
        self.board.move(7, 6, 7, 7)
        self.board.move(0, 6, 0, 7)
        with self.assertRaises(Exception):
            self.board.castling(True, False)

    def test_castleFailsWhileAttackedFieldWhite(self):
        self.board.board[5][6] = "n"
        self.board.move(6, 5, 5, 5)
        self.board.move(1, 1, 2, 1)
        with self.assertRaises(Exception):
            self.board.castling(True, False)

    def test_castleFailsWhileAttackedKingWhite(self):
        self.board.board[4][3] = "n"
        self.board.move(6, 6, 5, 6)
        self.board.move(4, 3, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(True, False)

    def test_castleProperlyBlack(self):
        self.board.move(6, 5, 5, 5)
        self.board.castling(False, False)
        resultBoard = np.array([
            ['r', '.', 'b', 'q', '.', 'r', 'k', '.'],
            ['p', 'p', 'p', '.', '.', 'p', 'p', 'p'],
            ['n', '.', '.', '.', '.', '.', '.', 'n'],
            ['.', '.', '.', 'p', 'p', '.', '.', '.'],
            ['.', '.', '.', 'P', 'P', '.', '.', '.'],
            ['N', '.', '.', '.', '.', 'P', '.', 'N'],
            ['P', 'P', 'P', '.', '.', '.', 'P', 'P'],
            ['R', '.', 'B', 'Q', 'K', '.', '.', 'R']
        ])
        for i in range(8):
            for j in range(8):
                self.assertEqual(resultBoard[i][j], self.board.board[i][j], str(i) + str(j))

    def test_castleFailsAfterKMoveBlack(self):
        self.board.move(7, 4, 6, 4)
        self.board.move(0, 4, 1, 4)
        self.board.move(6, 4, 7, 4)
        self.board.move(1, 4, 0, 4)
        self.board.move(6, 5, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(False, False)

    def test_castleFailsAfterRMoveBlack(self):
        self.board.move(7, 7, 7, 6)
        self.board.move(0, 7, 0, 6)
        self.board.move(7, 6, 7, 7)
        self.board.move(0, 6, 0, 7)
        self.board.move(6, 5, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(False, False)

    def test_castleFailsWhileAttackedFielBlack(self):
        self.board.board[2][6] = "N"
        self.board.move(6, 5, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(False, False)

    def test_castleFailsWhileAttackedKingBlack(self):
        self.board.board[2][5] = "N"
        self.board.move(6, 6, 5, 6)
        with self.assertRaises(Exception):
            self.board.castling(False, False)


if __name__ == '__main__':
    unittest.main()
