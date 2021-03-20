import unittest
from board import *
from testboards import Testboards as TB


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

    def setUp(self):
        self.board = Board()

    # Knight moves
    def test_UnrestrictedWhiteKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "N"
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_CapturingWhiteKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "N"
        self.board.board[2][2] = 'n'
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(30, len(actual))

    def test_UnrestrictedBlackKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "n"
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "3:5", "5:5", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

    def test_CapturingBlackKnight(self):
        self.board.clearBoard()
        self.board.board[4][3] = "n"
        self.board.board[2][2] = 'N'
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["2:2", "2:4", "6:4", "6:2", "5:1", "3:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(30, len(actual))

    # Rook Testing
    # White Rook Test
    def test_UnrestrictedRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["4:4", "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_CapturingRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        self.board.board[3][3] = "n"
        self.board.board[4][4] = "p"
        self.board.board[4][2] = "r"
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:3", "4:4", "4:2", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_RestrictedRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        self.board.board[3][3] = "N"
        self.board.board[4][4] = "P"
        self.board.board[4][2] = "R"
        self.board.board[5][3] = "r"
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["5:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    def test_bishopTesting(self):
        self.board.board = TB.bishopAccessTest
        results = ["3:4", "2:5", "5:2", "6:1", "5:4", "6:5", "3:2", "2:1"]
        actual = str(self.board.getPossibleMoves(4, 3))
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(actual), 40)

    # Pawn Moves
    def test_PawnMoves(self):
        board = Board()
        self.assertEqual("[2:7, 3:7]", str(self.board.getPossibleMoves(1, 7)))
        del board

        # White Pawn moves

    def test_WhitePawnNoTakeOptions(self):
        actual = str(self.board.getPossibleMoves(6, 3))
        results = ["5:3", "4:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnWithTakeOptions(self):
        self.board.board = TB.TwoPawnTakeOptions
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:2", "3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def test_WhitePawnWithOneTakeOneMove(self):
        self.board.board = TB.OneTakeOneMoveOption
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

        # Black Pawn moves

    def test_BlackPawnNoTakeOptions(self):
        actual = str(self.board.getPossibleMoves(1, 3))
        results = ["3:3", "2:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnWithTwoTakeOptions(self):
        self.board.board = TB.BlackTwoPawnTakeOptions
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["5:2", "5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def test_BlackPawnWithOneTakeOneMove(self):
        self.board.board = TB.BlackOneTakeOneMoveOption
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    # board tests
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
        self.assertEqual(self.board.board[3][1], '.')
        self.board.placePieceOnNotation('k', 'b5')
        self.assertEqual(self.board.board[3][1], 'k')


if __name__ == '__main__':
    unittest.main()
