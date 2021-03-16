import unittest

from Board import Board
from Testboards import Testboards as TB

class Testboard(unittest.TestCase):

    def test_knightTesting(self):
        board = Board()
        board.board=TB.knightAccessTest
        self.assertEqual("[2:4, 3:5, 5:5, 6:4, 6:2, 5:1, 3:1, 2:2]", str(board.getPossibleMoves(4,3)))


    def test_RookTesting(self):
        board = Board()
        board.board=TB.rookAccessTest
        self.assertEqual("[5:3, 3:3, 2:3, 1:3, 4:4, 4:5, 4:6, 4:7, 4:2, 4:1, 4:0]", str(board.getPossibleMoves(4,3)))

    def test_bishopTesting(self):
        board = Board()
        board.board = TB.bishopAccessTest
        results = ["3:4" , "2:5", "5:2", "6:1", "5:4", "6:5", "3:2", "2:1"]
        actual = str(board.getPossibleMoves(4,3))
        for i in results:
            self.assertIn(i, actual)

    def testWhitePawnNoTakeOptions(self):
        board = Board()
        actual = str(board.getPossibleMoves(6,3))
        results = ["5:3","4:3"]
        for i in results:
            self.assertIn(i, actual)


if __name__ == '__main__':
    unittest.main()
