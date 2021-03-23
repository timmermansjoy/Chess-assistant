import unittest
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

    # Black knight moves

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


class TestboardRook(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Rook Testing
    # White Rook Test
    def test_UnrestrictedWhiteRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "R"
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["5:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

        # Black Rook Test

    def test_UnrestrictedBlackRookTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "r"
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["5:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardPawns(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Pawn Moves
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

    def test_WhitePawnEnPassentLeft(self):
        self.board.board = TB.enPassentStartBoard1
        self.whiteMove = False
        self.board.move(1, 0, 3, 0)
        actual = str(self.board.getPossibleMoves(3, 1))
        result = ["2:1", "2:0"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnEnPassentRight(self):
        self.board.board = TB.enPassentStartBoard1
        self.whiteMove = False
        self.board.move(1, 2, 3, 2)
        actual = str(self.board.getPossibleMoves(3, 1))
        result = ["2:1", "2:2"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_WhitePawnNoEnPassentLeft(self):
        self.board.board = TB.noEnPassentStartBoard1
        self.whiteMove = False
        self.board.move(0, 0, 1, 0)
        actual = str(self.board.getPossibleMoves(3, 1))
        result = ["2:1"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

    def test_WhitePawnNoEnPassentRight(self):
        self.board.board = TB.noEnPassentStartBoard2
        self.whiteMove = False
        self.board.move(0, 3, 1, 2)
        actual = str(self.board.getPossibleMoves(3, 1))
        result = ["2:1"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

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

    def test_BlackPawnEnPassentLeft(self):
        self.board.board = TB.enPassentStartBoard2
        self.whiteMove = True
        self.board.move(6, 1, 4, 1)
        actual = str(self.board.getPossibleMoves(4, 2))
        result = ["5:2", "5:1"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnEnPassentRight(self):
        self.board.board = TB.enPassentStartBoard2
        self.whiteMove = True
        self.board.move(6, 3, 4, 3)
        actual = str(self.board.getPossibleMoves(4, 2))
        result = ["5:2", "5:3"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

    def test_BlackPawnNoEnPassentLeft(self):
        self.board.board = TB.noEnPassentStartBoard3
        self.whiteMove = True
        self.board.move(7, 2, 5, 0)
        actual = str(self.board.getPossibleMoves(4, 2))
        result = ["5:2"]
        for i in result:
            self.assertIn(i, actual)
        self.assertEqual(5, len(actual))

    def test_BlackPawnNoEnPassentLeft2(self):
        self.board.board = TB.noEnPassentStartBoard4
        self.whiteMove = True
        self.board.move(7, 3, 6, 3)
        actual = str(self.board.getPossibleMoves(4, 2))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:4", "2:5", "1:6", "5:2", "5:4", "6:5", "7:6", "3:2", "2:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    # Black Bishop moves
    def test_UnrestrictedBlackBishopTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "b"
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:4", "2:5", "1:6", "5:2", "5:4", "6:5", "7:6", "3:2", "2:1"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))


class TestboardQueen(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    # Queen Tests
    # White Queen Tests
    def test_UnrestrictedWhiteQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "Q"
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:4", "3:2"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    # Black Queen Tests
    def test_UnrestrictedBlackQueenTesting(self):
        self.board.clearBoard()
        self.board.board[4][3] = "q"
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
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
        actual = str(self.board.getPossibleMoves(4, 3))
        results = ["3:4", "3:2"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

    # King Test


class TestboardKing(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_MoveKing(self):
        self.board.board = TB.kingMoves
        actual = str(self.board.getPossibleMoves(3, 4))
        results = ["2:3", "2:4", "2:5", "4:3", "4:4", "4:5", "3:3", "3:5"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(40, len(actual))

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
        actual = str(self.board.getAllAttackedFields(False))
        results = ["3:4", "2:5", "1:6", "0:7", "5:2", "6:1", "7:0", "5:4", "6:5", "7:6", "3:2", "2:1", "1:0", "4:4",
                   "4:5", "4:6", "4:7", "4:2", "4:1", "4:0", "0:3", "1:3", "2:3", "3:3", "5:3", "6:3", "7:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(len(results) * 5, len(actual))

if __name__ == '__main__':
    unittest.main()
