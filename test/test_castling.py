import unittest
import pytest
from board import *
from testboards import Testboards as TB


class CastleTests(unittest.TestCase):

    def setUp(self):
        self.board = Board()
        initialMoves = ["e2e4", "e7e5", "f1a6", "f8a3", "d2d4", "d7d5", "g1h3", "g8h6", "b1a3", "b8a6"]
        for i in initialMoves:
            coords = self.board.notationToCords(str(i))
            self.board.move(coords[0].row, coords[0].column, coords[1].row, coords[1].column)

    def test_castleProperlyWhiteKing(self):
        self.board.castling(True, False, self.board.board)
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
                self.assertEqual(resultBoard[i][j], self.board.board[i][j], str(i) + str(j))

    def test_castleFailsAfterKMoveWhite(self):
        self.board.move(7, 4, 6, 4)
        self.board.move(0, 4, 1, 4)
        self.board.move(6, 4, 7, 4)
        self.board.move(1, 4, 0, 4)
        with self.assertRaises(Exception):
            self.board.castling(True, False, self.board.board)

    def test_castleFailsAfterRMoveWhite(self):
        self.board.move(7, 7, 7, 6)
        self.board.move(0, 7, 0, 6)
        self.board.move(7, 6, 7, 7)
        self.board.move(0, 6, 0, 7)
        with self.assertRaises(Exception):
            self.board.castling(True, False, self.board.board)

    def test_castleFailsWhileAttackedFieldWhite(self):
        self.board.board[5][6] = "n"
        self.board.move(6, 5, 5, 5)
        self.board.move(1, 1, 2, 1)
        with self.assertRaises(Exception):
            self.board.castling(True, False, self.board.board)

    def test_castleFailsWhileAttackedKingWhite(self):
        self.board.board[4][3] = "n"
        self.board.move(6, 6, 5, 6)
        self.board.move(4, 3, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(True, False, self.board.board)

    def test_castleProperlyBlack(self):
        self.board.move(6, 5, 5, 5)
        self.board.castling(False, False, self.board.board)
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
            self.board.castling(False, False, self.board.board)

    def test_castleFailsAfterRMoveBlack(self):
        self.board.move(7, 7, 7, 6)
        self.board.move(0, 7, 0, 6)
        self.board.move(7, 6, 7, 7)
        self.board.move(0, 6, 0, 7)
        self.board.move(6, 5, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(False, False, self.board.board)

    def test_castleFailsWhileAttackedFielBlack(self):
        self.board.board[2][6] = "N"
        self.board.move(6, 5, 5, 5)
        with self.assertRaises(Exception):
            self.board.castling(False, False, self.board.board)

    def test_castleFailsWhileAttackedKingBlack(self):
        self.board.board[2][5] = "N"
        self.board.move(6, 6, 5, 6)
        with self.assertRaises(Exception):
            self.board.castling(False, False, self.board.board)

    def test_castleFailsWhileMovedRookQueenSideWhite(self):
        self.board.move(7, 0, 7, 1)
        self.board.move(0, 0, 0, 1)
        self.board.move(7, 1, 7, 0)
        self.board.move(0, 1, 0, 0)
        with self.assertRaises(Exception):
            self.board.castling(True, True, self.board.board)

    def test_castleFailsWhileMovedRookQueenSideBlack(self):
        self.board.move(7, 0, 7, 1)
        self.board.move(0, 0, 0, 1)
        self.board.move(7, 1, 7, 0)
        self.board.move(0, 1, 0, 0)
        self.board.move(7, 0, 7, 1)
        with self.assertRaises(Exception):
            self.board.castling(False, True, self.board.board)

    def test_castleFailsWhilePieceInbetweenWhite(self):
        self.board.board[7][1] = "n"
        with self.assertRaises(Exception):
            self.board.castling(True, True, self.board.board)

    def test_castleFailsWhilePieceInbetweenBlack(self):
        self.board.board[0][1] = "n"
        self.board.move(7, 0, 7, 1)
        with self.assertRaises(Exception):
            self.board.castling(False, True, self.board.board)

    def test_castleProperlyWhiteQueen(self):
        self.board.move(7, 3, 5, 3)
        self.board.move(0, 3, 2, 3)
        self.board.move(7, 2, 5, 4)
        self.board.move(0, 2, 2, 4)
        self.board.castling(True, True, self.board.board)
        resultBoard = np.array([
            ['r', '.', '.', '.', 'k', '.', '.', 'r'],
            ['p', 'p', 'p', '.', '.', 'p', 'p', 'p'],
            ['n', '.', '.', 'q', 'b', '.', '.', 'n'],
            ['.', '.', '.', 'p', 'p', '.', '.', '.'],
            ['.', '.', '.', 'P', 'P', '.', '.', '.'],
            ['N', '.', '.', 'Q', 'B', '.', '.', 'N'],
            ['P', 'P', 'P', '.', '.', 'P', 'P', 'P'],
            ['.', '.', 'K', 'R', '.', '.', '.', 'R']
        ])
        for i in range(8):
            for j in range(8):
                self.assertEqual(resultBoard[i][j], self.board.board[i][j], str(i) + str(j))

    def test_castleProperlyBlackQueen(self):
        self.board.move(7, 3, 5, 3)
        self.board.move(0, 3, 2, 3)
        self.board.move(7, 2, 5, 4)
        self.board.move(0, 2, 2, 4)
        self.board.move(6, 5, 5, 5)
        self.board.castling(False, True, self.board.board)
        resultBoard = np.array([
            ['.', '.', 'k', 'r', '.', '.', '.', 'r'],
            ['p', 'p', 'p', '.', '.', 'p', 'p', 'p'],
            ['n', '.', '.', 'q', 'b', '.', '.', 'n'],
            ['.', '.', '.', 'p', 'p', '.', '.', '.'],
            ['.', '.', '.', 'P', 'P', '.', '.', '.'],
            ['N', '.', '.', 'Q', 'B', 'P', '.', 'N'],
            ['P', 'P', 'P', '.', '.', '.', 'P', 'P'],
            ['R', '.', '.', '.', 'K', '.', '.', 'R']
        ])
        for i in range(8):
            for j in range(8):
                self.assertEqual(resultBoard[i][j], self.board.board[i][j], str(i) + str(j))

    def test_BlackcastleFailsOnOpponentsTurn(self):
        with self.assertRaises(Exception):
            self.board.castling(False, False, self.board.board)

    def test_WhitecastleFailsOnOpponentsTurn(self):
        self.board.move(7, 3, 5, 3)
        with self.assertRaises(Exception):
            self.board.castling(True, True, self.board.board)


if __name__ == '__main__':
    unittest.main()
