import unittest

from Board import Board
from Testboards import Testboards as TB


# Basic structure per piece:
# 1 test with unrestricted movement
# 1 test with the option to take a piece and regular movement
# 1 test with both the option to take an enemy piece and restricted movement by a friendly piece
# Basic structure per test:
# create a Board-object and give it a Board.board
# if needed, edit the board with the piece you want to test
# create a "Results"-array with all the expected moves
# create an "actual"-string using board.getPossibleMoves()
# verify all moves in "Results" are present in "actual"
# verify the length of "actual" (exactly 5 characters per possible moves)

class Testboard(unittest.TestCase):
    # Knight moves
    # White Knight Moves
    def test_UnrestrictedWhiteKnight(self):
        board = Board()
        board.board = TB.BlankBoard
        board.board[4][3] = "N"
        actual = str(board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_CapturingWhiteKnight(self):
        board = Board()
        board.board = TB.BlankBoard
        board.board[4][3] = "N"
        board.board[2][2] = 'n'
        actual = str(board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_RestrictedWhiteKnight(self):
        board = Board()
        board.board = TB.BlankBoard
        board.board[4][3] = "N"
        board.board[2][2] = 'n'
        board.board[2][4] = 'n'
        board.board[3][5] = 'N'
        board.board[5][5] = 'N'
        actual = str(board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(30, len(actual))
        # Black Knight Moves

    def test_knightTesting(self):
        board = Board()
        board.board = TB.knightAccessTest
        self.assertEqual("[2:4, 3:5, 5:5, 6:4, 6:2, 5:1, 3:1, 2:2]", str(board.getPossibleMoves(4, 3)))

    def test_RookTesting(self):
        board = Board()
        board.board = TB.rookAccessTest
        self.assertEqual("[5:3, 3:3, 2:3, 1:3, 4:4, 4:5, 4:6, 4:7, 4:2, 4:1, 4:0]", str(board.getPossibleMoves(4, 3)))

    def test_bishopTesting(self):
        board = Board()
        board.board = TB.bishopAccessTest
        results = ["3:4", "2:5", "5:2", "6:1", "5:4", "6:5", "3:2", "2:1"]
        actual = str(board.getPossibleMoves(4, 3))
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(actual), 40)

    # Pawn Moves
    def test_PawnMoves(self):
        board = Board()
        self.assertEqual("[2:7, 3:7]", str(board.getPossibleMoves(1, 7)))

        # White Pawn moves

    def test_WhitePawnNoTakeOptions(self):
        board = Board()
        actual = str(board.getPossibleMoves(6, 3))
        results = ["5:3", "4:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnWithTakeOptions(self):
        board = Board()
        board.board = TB.TwoPawnTakeOptions
        actual = str(board.getPossibleMoves(4, 3))
        results = ["3:2", "3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def test_WhitePawnWithOneTakeOneMove(self):
        board = Board()
        board.board = TB.OneTakeOneMoveOption
        actual = str(board.getPossibleMoves(4, 3))
        results = ["3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

        # Black Pawn moves

    def test_BlackPawnNoTakeOptions(self):
        board = Board()
        actual = str(board.getPossibleMoves(1, 3))
        results = ["3:3", "2:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnWithTwoTakeOptions(self):
        board = Board()
        board.board = TB.BlackTwoPawnTakeOptions
        actual = str(board.getPossibleMoves(4, 3))
        results = ["5:2", "5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def test_BlackPawnWithOneTakeOneMove(self):
        board = Board()
        board.board = TB.BlackOneTakeOneMoveOption
        actual = str(board.getPossibleMoves(4, 3))
        results = ["5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))


if __name__ == '__main__':
    unittest.main()
