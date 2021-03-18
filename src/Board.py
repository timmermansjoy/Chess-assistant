import logging
import numpy as np
from extra import Coordinate

ranksToRows = {"1": 7, "2": 6, "3": 5, "4": 4, "5": 3, "6": 2, "7": 1, "8": 0}
rowsToRanks = {v: k for k, v in ranksToRows.items()}
filesToColumns = {"a": 0, "b": 1, "c": 2, "d": 3, "e": 4, "f": 5, "g": 6, "h": 7}
columnsToFiles = {v: k for k, v in filesToColumns.items()}


class Board:
    def __init__(self):
        self.board = np.array([
            ["r", "n", "b", "q", "k", "b", "n", "r"],
            ["p", "p", "p", "p", "p", "p", "p", "p"],
            [".", ".", ".", ".", ".", ".", ".", "."],
            [".", ".", ".", ".", ".", ".", ".", "."],
            [".", ".", ".", ".", ".", ".", ".", "."],
            [".", ".", ".", ".", ".", ".", ".", "."],
            ["P", "P", "P", "P", "P", "P", "P", "P"],
            ["R", "N", "B", "Q", "K", "B", "N", "R"],
        ])

        self.whiteMove = True
        self.whiteARookMoved = False
        self.whiteHRookMoved = False
        self.blackARookMoved = False
        self.blackHRookMoved = False
        self.moveLog = []

    def getPossibleMoves(self, row, column):
        piece = self.board[row][column]
        if piece != ".":
            # commenting out a part of the coordinate processing because it's messing with everything and pawns have become broken enough to do ANYTHING SEND HELP
            #coord = Coordinate(int(self.ranksToRows[str(row)]), column)
            coord = Coordinate(row, column)
            self.whiteMove = piece.isupper()
            piece = piece.upper()
            moves = []
            if piece == "P":
                moves = self.getPawnMoves(coord)
            elif piece == "R":
                moves = self.getRookMoves(coord)
            elif piece == "N":
                moves = self.getKnightMoves(coord)
            elif piece == "B":
                moves = self.getBishopMoves(coord)
            elif piece == "Q":
                moves = self.getQueenMoves(coord)
            elif piece == "K":
                moves = self.getKingMoves(coord)
            else:
                logging.warning(piece + " is not a valid piece ??")  # maybe throw an error instead
            return moves
        else:
            logging.warning(piece + " is not a valid piece ??")
            return None

    # compiles to machine code
    def getPawnMoves(self, coord):

        # Initiate the moves list
        moves = []

        # Set vertical step to one, if it is Black's move the row should be increased with one
        step = 1

        # If it is white's move the pawn should move up, so the row must decrease with one, multiplying the step by -1 will make sure this is done correctly
        if self.whiteMove:
            step *= -1

        # Check to see if there is a piece on the square in front of the pawn
        if self.board[coord.row + step][coord.column] == ".":

            # If the square in front of the pawn is empty it can move one square up (or down for black)
            moves.append(Coordinate(coord.row + step, coord.column))

            # If the square in front of the pawn is empty and it has not yet moved, it can move two squares up (or down for black) IF that square is empty
            if coord.row == 3.5 + (-2.5 * step) and self.board[coord.row + (step * 2)][coord.column] == ".":
                moves.append(Coordinate(coord.row + (2 * step), coord.column))

        # If the pawn is not on the left edge of the board, check to see if it can capture a piece one square to the right and one square up (or down for black)
        if coord.column > 0:

            # If it is white's move, there must be a lowercase (black) piece on this square for it to capture
            if self.whiteMove:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column - 1] != "." and self.board[coord.row + step][coord.column - 1].islower():
                    moves.append(Coordinate(coord.row + step, coord.column - 1))

            # If it is black's move, there must be an uppercase (white) piece on this square for it to capture
            else:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column - 1] != "." and self.board[coord.row + step][coord.column - 1].isupper():
                    moves.append(Coordinate(coord.row + step, coord.column - 1))

        # If the pawn is not on the right edge of the board, check to see if it can capture a piece one square to the left and one square up (or down for black)
        if coord.column < 7:

            # If it is white's move, there must be a lowercase (black) piece on this square for it to capture
            if self.whiteMove:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column + 1] != "." and self.board[coord.row + step][coord.column + 1].islower():
                    moves.append(Coordinate(coord.row + step, coord.column + 1))

                # If it is black's move, there must be an uppercase (white) piece on this square for it to capture
            else:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column + 1] != "." and self.board[coord.row + step][coord.column + 1].isupper():
                    moves.append(Coordinate(coord.row + step, coord.column + 1))
                # TODO: en passant
        return moves

    def getRookMoves(self, coord):

        # Initiate the moves list
        moves = []

        # If the rook is not on the upper edge of the board, get the possible up moves
        if coord.column < 7:
            moves.extend(self.getVertical(coord, 1))

        # If the rook is not on the lower edge of the board, get the possible down moves
        if coord.column > 0:
            moves.extend(self.getVertical(coord, -1))

        # If the rook is not on the right edge of the board, get the possible right moves
        if coord.row < 7:
            moves.extend(self.getHorizontal(coord, 1))

        # If the rook is not on the left edge of the board, get the possible left moves
        if coord.row > 0:
            moves.extend(self.getHorizontal(coord, -1))

        return moves

    def getVertical(self, start, step):

       # Initiate the moves list
        moves = []

        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the rook be able to keep moving in the specified direction
        pathBlocked = False

        # The first possible move is the square it is currently standing on
        current = start.row

        # For vertical moves, the column will not change
        col = start.column

        # If the step is > 0, the end square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the step is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        end = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and current != end:
            current += step

            # Get the value of the target square
            target = self.board[current][col]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(current, col))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.whiteMove and target.islower():

                # If this is the case, add the capturing move to moves list
                moves.append(Coordinate(current, col))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.whiteMove and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(current, col))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the piece can not move there and the path is blocked
            else:
                pathBlocked = True

        return moves

    def getHorizontal(self, start, step):

        # Initiate the moves list
        moves = []

        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the rook be able to keep moving in the specified direction
        pathBlocked = False

        # The first possible move is the square it is currently standing on
        current = start.column + step

        # For the horizontal moves, the row will not change
        row = start.row

        # If the step is > 0, the end square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the step is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        end = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and current != end:

            # Get the value of the target square
            target = self.board[row][current]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(row, current))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.whiteMove and target.islower():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(row, current))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.whiteMove and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(row, current))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the piece can not move there and the path is blocked
            else:
                pathBlocked = True
            current += step

        return moves

    def getKnightMoves(self, coord):

        # Initiate the moves list
        moves = []

        # Initiate a list of that contains the target squares for all 8 directions
        targets = [
            Coordinate(coord.row - 2, coord.column + 1),
            Coordinate(coord.row - 1, coord.column + 2),
            Coordinate(coord.row + 1, coord.column + 2),
            Coordinate(coord.row + 2, coord.column + 1),
            Coordinate(coord.row + 2, coord.column - 1),
            Coordinate(coord.row + 1, coord.column - 2),
            Coordinate(coord.row - 1, coord.column - 2),
            Coordinate(coord.row - 2, coord.column - 1)
        ]

        # For each of these targets, check if they are valid moves for the knight.
        for target in targets:

            # Check if the target square is on the board
            if target.row >= 0 and target.row <= 7 and target.column >= 0 and target.column <= 7:

                # Get the value of the target square
                square = self.board[target.row][target.column]

                # If the square is empty it is a valid move, and it can be added to the moves list
                if square == ".":
                    moves.append(target)

                # Else, if it's white's move and the target square contains a black piece, or the other way around, add the capturing move to the moves list
                elif self.whiteMove and square.islower():
                    moves.append(target)
                elif not self.whiteMove and square.isupper():
                    moves.append(target)

        return moves

    def getDiagonalNorthWest(self, start, step):

        # Initiate the moves list
        moves = []

        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the piece be able to keep moving in the specified diagonal direction
        pathBlocked = False

        # Define the starting square of the piece that plans to move diagonally
        currentColumn = start.column
        currentRow = start.row

        # If the rowStep is > 0, the verticalEnd square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the columnStep is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        verticalEnd = 8 if step > 0 else -1
        horizontalEnd = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and currentRow != verticalEnd and currentColumn != horizontalEnd:
            currentRow += step
            currentColumn += step

            # Get the value of the target square
            target = self.board[currentRow][currentColumn]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.whiteMove and target.islower():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.whiteMove and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the rook can
            else:
                pathBlocked = True

        return moves

    def getDiagonalNorthEast(self, start, step):
        # Initiate the moves list
        moves = []
        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the piece be able to keep moving in the specified diagonal direction
        pathBlocked = False

        # Define the starting square of the piece that plans to move diagonally
        currentColumn = start.column
        currentRow = start.row

        # If the rowStep is > 0, the verticalEnd square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the columnStep is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        verticalEnd = 8 if step > 0 else -1
        horizontalEnd = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and currentRow != verticalEnd and currentColumn != horizontalEnd:
            currentRow -= step
            currentColumn += step

            # Get the value of the target square
            target = self.board[currentRow][currentColumn]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.whiteMove and target.islower():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.whiteMove and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the bishop can
            else:
                pathBlocked = True

        return moves

    def getBishopMoves(self, coord):

        # Initiate the moves list
        moves = []

        # If the bishop is not on the upper or right edge of the board, get the UR moves
        if coord.column < 7:
            moves.extend(self.getDiagonalNorthEast(coord, 1))

        # If the bishop is not on the lower or left edge of the board, get the possible DL moves
        if coord.column > 0:
            moves.extend(self.getDiagonalNorthEast(coord, -1))

        # If the bishop is not on the left or top edge of the board, get the possible UL moves
        if coord.row < 7:
            moves.extend(self.getDiagonalNorthWest(coord, 1))

        # If the bishop is not on the lower or right edge of the board, get the possible DR moves
        if coord.row > 0:
            moves.extend(self.getDiagonalNorthWest(coord, -1))

        return moves

    def getQueenMoves(self, coord):
        pass

    def getKingMoves(self, coord):
        pass

    def isValid(self, coord):  # Je mag jezelf niet check zetten, move moet valide zijn,....
        pass

    def move(self, startRow, startColumn, endRow, endColumn):
        # check if the move is on a piece and is a valid move
        if self.board[startRow][startColumn] != "." and "{}:{}".format(endRow, endColumn) in str(self.getPossibleMoves(startRow, startColumn)):
            piece = self.board[startRow][startColumn]
            self.board[startRow][startColumn] = "."
            self.board[endRow][endColumn] = piece
        else:
            logging.warning('That is not a valid move')

    def notationToCords(self, notation):
        # strip the input of spaces
        notation = notation.strip()
        notation = notation.lower()
        if len(notation) == 4:
            startColumn = filesToColumns[notation[0]]
            startRow = ranksToRows[notation[1]]
            endColumn = filesToColumns[notation[2]]
            endRow = ranksToRows[notation[3]]

        if "{}:{}".format(endRow, endColumn) in str(self.getPossibleMoves(startRow, startColumn)):
            startCord = Coordinate(startRow, startColumn)
            endCord = Coordinate(endRow, endColumn)
            return startCord, endCord
        else:
            logging.warning('That is not a valid move')

    def __str__(self):
        result = ""
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                result += self.board[row][column] + " "
            result += "\n"
        return result
