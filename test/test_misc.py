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

    def test_strFunction(self):
        assert self.board.__str__() == "r n b q k b n r \np p p p p p p p \n. . . . . . . . \n. . . . . . . . \n. . . . . . . . \n. . . . . . . . \nP P P P P P P P \nR N B Q K B N R \n"


class TestboardNotationMoveTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_NotationMoveInvalidMoveThrowsException(self):
        with pytest.raises(Exception):
            self.board.notationMove("a4a5")

    def test_NotationMoveNonexistentFieldThrowsException(self):
        with pytest.raises(Exception):
            self.board.notationMove("i4i9")

    def test_NotationMoveWrongLengthThrowsException(self):
        with pytest.raises(Exception):
            self.board.notationMove("na2a4")


class TestboardMoveTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_MoveInvalidPiecehrowsException(self):
        self.board.board[6][1] = "J"
        with pytest.raises(Exception):
            self.board.move(6, 1, 4, 1)

    def test_MoveOnOpponentsTurnThrowsException(self):
        with pytest.raises(Exception):
            self.board.move(1, 1, 3, 1)

    def test_MoveInvalidPiecehrowsException(self):
        self.board.board[6][1] = "J"
        with pytest.raises(Exception):
            self.board.move(6, 1, 4, 1)

    def test_MoveNoPieceThrowsException(self):
        with pytest.raises(Exception):
            self.board.move(4, 1, 5, 1)

    def test_MoveWhileCheckThrowsException(self):
        self.board.board = deepcopy(TB.checkSetup)
        print(self.board)
        self.board.move(6, 1, 5, 1)
        self.board.move(0, 7, 0, 6)
        with pytest.raises(Exception):
            self.board.move(5, 1, 4, 1)

    def test_MoveOutOfCheckThrowsNoException(self):
        self.board.board = deepcopy(TB.checkSetup)
        self.board.move(6, 1, 5, 1)
        self.board.move(0, 7, 0, 6)
        self.board.move(5, 6, 5, 5)


class TestboardMovelogTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()

    def test_getChessNotationSingleMoveTest(self):
        self.board.move(6, 0, 4, 0)
        assert self.board.GetChessNotation() == "1. a2 a4 "

    def test_getChessNotationMoreMovesTest(self):
        self.board.move(6, 0, 4, 0)
        self.board.move(1, 0, 3, 0)
        assert self.board.GetChessNotation() == "1. a2 a4 --- a7 a5 "

    def test_getChessNotationTooManyMovesTest(self):
        self.board.move(6, 0, 4, 0)
        self.board.move(1, 0, 3, 0)
        self.board.move(6, 1, 4, 1)
        self.board.move(1, 1, 3, 1)
        self.board.move(6, 2, 4, 2)
        self.board.move(1, 2, 3, 2)
        assert self.board.GetChessNotation() == "1. a2 a4 --- a7 a5 \n2. b2 b4 --- b7 b5 \n3. c2 c4 --- c7 c5 "


if __name__ == '__main__':
    unittest.main()
