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




if __name__ == '__main__':
    unittest.main()
