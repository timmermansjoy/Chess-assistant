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
        self.assertEqual(len(actual), 40)

    def testWhitePawnNoTakeOptions(self):
        board = Board()
        actual = str(board.getPossibleMoves(6,3))
        results = ["5:3","4:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual)) #each move is exactely 5 long

    def testWhitePawnWithTakeOptions(self):
        board = Board()
        board.board = TB.TwoPawnTakeOptions
        actual = str(board.getPossibleMoves(4,3))
        results = ["3:2", "3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def testWhitePawnWithOneTakeOneMove(self):
        board = Board()
        board.board = TB.OneTakeOneMoveOption
        actual = str(board.getPossibleMoves(4,3))
        results = ["3:3", "3:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))
    #Black Pawns
    def testBlackPawnNoTakeOptions(self):
        board = Board()
        actual = str(board.getPossibleMoves(1,3))
        results = ["3:3","2:3"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual)) #each move is exactely 5 long

    def testBlackPawnWithTakeOptions(self):
        board = Board()
        board.board = TB.BlackTwoPawnTakeOptions
        actual = str(board.getPossibleMoves(4,3))
        results = ["5:2", "5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(15, len(actual))

    def testBlackPawnWithOneTakeOneMove(self):
        board = Board()
        board.board = TB.BlackOneTakeOneMoveOption
        actual = str(board.getPossibleMoves(4,3))
        results = ["5:3", "5:4"]
        for i in results:
            self.assertIn(i, actual)
        self.assertEqual(10, len(actual))

if __name__ == '__main__':
    unittest.main()
