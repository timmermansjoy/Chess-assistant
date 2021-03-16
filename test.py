import unittest

from Board import Board
from Testboards import Testboards as TB

class Testboard(unittest.TestCase):

    def test_knightMoves(self):
        board = Board()
        board.board=TB.knightmoverTest
        self.assertEqual("[2:4, 3:5, 5:5, 6:4, 6:2, 5:1, 3:1, 2:2]", str(board.getPossibleMoves(4,3)))


    def test_RookMoves(self):
        board = Board()
        board.board=TB.rookAccessTest
        self.assertEqual("[5:3, 3:3, 2:3, 1:3, 4:4, 4:5, 4:6, 4:7, 4:2, 4:1, 4:0]", str(board.getPossibleMoves(4,3)))

    def test_PawnMoves(self):
        board = Board()
        self.assertEqual("[1:6, 1:5]", str(board.getPossibleMoves(1,7)))

if __name__ == '__main__':
    unittest.main()
