import numpy as np
from extra import Coordinate

ranksToRows = {"1": 7, "2": 6, "3": 5, "4": 4, "5": 3, "6": 2, "7": 1, "8": 0}
rowsToRanks = {v: k for k, v in ranksToRows.items()}
filesToColumns = {"a": 0, "b": 1, "c": 2, "d": 3, "e": 4, "f": 5, "g": 6, "h": 7}
columnsToFiles = {v: k for k, v in filesToColumns.items()}
pieces = ["r", "n", "b", "q", "k", "p"]


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

        self.isWhitePiece = True
        self.isWhitePlayerTurn = True
        self.whiteARookMoved = False
        self.whiteHRookMoved = False
        self.blackARookMoved = False
        self.blackHRookMoved = False
        self.moveLog = []

    def getDirections(self, coord):
        directions = {'up': False, 'right': False, 'down': False, 'left': False, 'upLeft': False, 'upRight': False, 'downRight': False, 'downLeft': False}

        piece = self.board[coord.row][coord.column].upper()

        if coord.row > 0:
            directions['up'] = True
        if coord.row < 7:
            directions['down'] = True
        if coord.column > 0:
            directions['left'] = True
        if coord.column < 7:
            directions['right'] = True
        
        directions['upLeft'] = directions['up'] + directions['left']
        directions['upRight'] = directions['up'] + directions['right']
        directions['downRight'] = directions['down'] + directions['right']
        directions['downLeft'] = directions['down'] + directions['left']

        return directions

    def clearBoard(self):
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                self.board[row][column] = "."

    def getPossibleMoves(self, row, column):
        piece = self.board[row][column]
        if piece != ".":
            # commenting out a part of the coordinate processing because it's messing with everything and pawns have
            # become broken enough to do ANYTHING SEND HELP coord = Coordinate(int(self.ranksToRows[str(row)]), column)
            coord = Coordinate(row, column)
            self.isWhitePiece = piece.isupper()
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
                raise Exception(piece + " is not a valid piece ??")  # maybe throw an error instead
            return moves
        else:
            raise Exception(piece + " is not a valid piece ??")

    # compiles to machine code
    def getPawnMoves(self, coord):

        # Initiate the moves list
        moves = []

        # Set vertical step to one, if it is Black's move the row should be increased with one
        step = 1

        # Get the directions the piece can move to
        directions = self.getDirections(coord)

        # If it is white's move the pawn should move up, so the row must decrease with one, multiplying the step by -1 will make sure this is done correctly
        if self.isWhitePiece:
            step *= -1

        # Check to see if there is a piece on the square in front of the pawn
        if self.board[coord.row + step][coord.column] == ".":

            # If the square in front of the pawn is empty it can move one square up (or down for black)
            moves.append(Coordinate(coord.row + step, coord.column))

            # If the square in front of the pawn is empty and it has not yet moved, it can move two squares up (or down for black) IF that square is empty
            if coord.row == 3.5 + (-2.5 * step) and self.board[coord.row + (step * 2)][coord.column] == ".":
                moves.append(Coordinate(coord.row + (2 * step), coord.column))

        # If the pawn is not on the left edge of the board, check to see if it can capture a piece one square to the right and one square up (or down for black)
        if directions['left']:

            # If it is white's move, there must be a lowercase (black) piece on this square for it to capture
            if self.isWhitePiece:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column - 1] != "." and self.board[coord.row + step][
                        coord.column - 1].islower():
                    moves.append(Coordinate(coord.row + step, coord.column - 1))

            # If it is black's move, there must be an uppercase (white) piece on this square for it to capture
            else:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column - 1] != "." and self.board[coord.row + step][
                        coord.column - 1].isupper():
                    moves.append(Coordinate(coord.row + step, coord.column - 1))

        # If the pawn is not on the right edge of the board, check to see if it can capture a piece one square to the left and one square up (or down for black)
        if directions['right']:

            # If it is white's move, there must be a lowercase (black) piece on this square for it to capture
            if self.isWhitePiece:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column + 1] != "." and self.board[coord.row + step][
                        coord.column + 1].islower():
                    moves.append(Coordinate(coord.row + step, coord.column + 1))

                # If it is black's move, there must be an uppercase (white) piece on this square for it to capture
            else:

                # If this is the case, add the capturing move to the moves list
                if self.board[coord.row + step][coord.column + 1] != "." and self.board[coord.row + step][
                        coord.column + 1].isupper():
                    moves.append(Coordinate(coord.row + step, coord.column + 1))

        # EN PASSENT
        piece = self.board[coord.row][coord.column]

        # Get the last move from the moveLog
        if len(self.moveLog) > 0:
            lastMoveEndCoord = self.moveLog[-1][1]

            # If the pawn is a white piece, it should be on row 3.
            if coord.row == 3 and self.isWhitePiece:

                # If this is the case, see if the last move was done with a pawn (of the opposite colour) and if the pawn was also placed on row 3.
                if self.board[lastMoveEndCoord.row][lastMoveEndCoord.column] == "p" and lastMoveEndCoord.row == 3:

                    # If this is the case, see if the black pawn is one square to the left of the white pawn.
                    if lastMoveEndCoord.column == coord.column - 1:
                        # If so, the en passent move can be made!
                        print("En Passant!!")
                        moves.append(Coordinate(coord.row - 1, coord.column - 1))

                    # If this is the case, see if the black pawn is one square to the right of the white pawn.
                    if lastMoveEndCoord.column == coord.column + 1:
                        # If so, the en passent move can be made!
                        print("En Passent!!")
                        moves.append(Coordinate(coord.row - 1, coord.column + 1))

            # If the pawn is a black piece, it should be on row 4.
            elif coord.row == 4 and not self.isWhitePiece:

                # If this is the case, see if the last move was done with a pawn (of the opposite colour) and if the pawn was also placed on row 4.
                if self.board[lastMoveEndCoord.row][lastMoveEndCoord.column] == "P" and lastMoveEndCoord.row == 4:

                    # If this is the case, see if the black pawn is one square to the left of the white pawn.
                    if lastMoveEndCoord.column == coord.column - 1:
                        # If so, the en passent move can be made!
                        print("En Passant!!")
                        moves.append(Coordinate(coord.row + 1, coord.column - 1))

                    # If this is the case, see if the black pawn is one square to the right of the white pawn.
                    if lastMoveEndCoord.column == coord.column + 1:
                        # If so, the en passent move can be made!
                        print("En Passant!!")
                        moves.append(Coordinate(coord.row + 1, coord.column + 1))
        return moves

    def getRookMoves(self, coord):

        # Initiate the moves list
        moves = []

        # Get directions the piece can move to
        directions = self.getDirections(coord)

        # If the rook is not on the upper edge of the board, get the possible up moves
        if directions['right']:
            moves.extend(self.getVertical(coord, 1))

        # If the rook is not on the lower edge of the board, get the possible down moves
        if directions['left']:
            moves.extend(self.getVertical(coord, -1))

        # If the rook is not on the right edge of the board, get the possible right moves
        if directions['down']:
            moves.extend(self.getHorizontal(coord, 1))

        # If the rook is not on the left edge of the board, get the possible left moves
        if directions['up']:
            moves.extend(self.getHorizontal(coord, -1))

        return moves

    def getVertical(self, start, step):

        # Initiate the moves list
        moves = []

        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the rook be able to keep moving in the specified direction
        pathBlocked = False

        # The first possible move is the square it is currently standing on
        current = start.row + step

        # For vertical moves, the column will not change
        col = start.column

        # If the step is > 0, the end square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the step is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        end = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and current != end:
            # Get the value of the target square
            target = self.board[current][col]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(current, col))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.isWhitePiece and target.islower():

                # If this is the case, add the capturing move to moves list
                moves.append(Coordinate(current, col))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.isWhitePiece and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(current, col))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the piece can not move there and the path is blocked
            else:
                pathBlocked = True
            current += step

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
            elif self.isWhitePiece and target.islower():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(row, current))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.isWhitePiece and target.isupper():

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
                elif self.isWhitePiece and square.islower():
                    moves.append(target)
                elif not self.isWhitePiece and square.isupper():
                    moves.append(target)

        return moves

    def getDiagonalNorthWest(self, start, step):

        # Initiate the moves list
        moves = []

        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the piece be able to keep moving in the specified diagonal direction
        pathBlocked = False

        # Define the starting square of the piece that plans to move diagonally
        currentColumn = start.column + step
        currentRow = start.row + step

        # If the rowStep is > 0, the verticalEnd square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the columnStep is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        verticalEnd = 8 if step > 0 else -1
        horizontalEnd = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and currentRow != verticalEnd and currentColumn != horizontalEnd:
            # Get the value of the target square
            target = self.board[currentRow][currentColumn]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.isWhitePiece and target.islower():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.isWhitePiece and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the rook can
            else:
                pathBlocked = True
            currentRow += step
            currentColumn += step

        return moves

    def getDiagonalNorthEast(self, start, step):
        # Initiate the moves list
        moves = []
        # Only if the path of the piece is not blocked by either a piece of its own color or a piece of the opponent's color,
        # will the piece be able to keep moving in the specified diagonal direction
        pathBlocked = False

        # Define the starting square of the piece that plans to move diagonally
        currentColumn = start.column + step
        currentRow = start.row - step

        # If the rowStep is > 0, the verticalEnd square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the columnStep is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        verticalEnd = 8 if step > 0 else -1
        horizontalEnd = 8 if step > 0 else -1

        # While the path is not blocked and we have not yet reached the end square,
        # increase or decrease the value of the current square and see if the piece can move there
        while not pathBlocked and currentRow != verticalEnd and currentColumn != horizontalEnd:
            # Get the value of the target square
            target = self.board[currentRow][currentColumn]

            # If the square is empty, the piece can move there, and the path is not blocked
            if target == ".":

                # If this is the case, add the move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

            # If the square is not empty, and it's white's turn, the piece can only move to this square
            # if a black piece is currently standing on it
            elif self.isWhitePiece and target.islower():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the square is not empty, and it's black's turn, the piece can only move to this square
            # if a white piece is currently standing on it
            elif not self.isWhitePiece and target.isupper():

                # If this is the case, add the capturing move to the moves list
                moves.append(Coordinate(currentRow, currentColumn))

                # The piece can not move any further when it captures, so the path is blocked
                pathBlocked = True

            # If the piece occupying the square is of the piece's own color, the bishop can
            else:
                pathBlocked = True
            currentRow -= step
            currentColumn += step

        return moves

    def getBishopMoves(self, coord):

        # Initiate the moves list
        moves = []

        # Get the directions the piece can move to 
        directions = self.getDirections(coord)

        # For each diagonal direction, see if the queen can move there
        # If this is the case, add the moves to the moves list
        if directions['upLeft']:
            moves.extend(self.getDiagonalNorthWest(coord, -1))
        if directions['upRight']:
            moves.extend(self.getDiagonalNorthEast(coord, 1))
        if directions['downRight']:
            moves.extend(self.getDiagonalNorthWest(coord, 1))
        if directions['downLeft']:
            moves.extend(self.getDiagonalNorthEast(coord, -1))

        return moves

    def getQueenMoves(self, coord):

        # Initiate the moves list
        moves = []

        # Get the directions the piece can move to
        directions = self.getDirections(coord)

        # For each horizontal and vertical diretion, see if the queen can move there.
        # If this is the case, add the moves to the moves list
        if directions['up']:
            moves.extend(self.getVertical(coord, -1))
        if directions['right']:
            moves.extend(self.getHorizontal(coord, 1))
        if directions['down']:
            moves.extend(self.getVertical(coord, 1))
        if directions['left']:
            moves.extend(self.getHorizontal(coord, -1))

        # For each diagonal direction, see if the queen can move there
        # If this is the case, add the moves to the moves list
        if directions['upLeft']:
            moves.extend(self.getDiagonalNorthWest(coord, -1))
        if directions['upRight']:
            moves.extend(self.getDiagonalNorthEast(coord, 1))
        if directions['downRight']:
            moves.extend(self.getDiagonalNorthWest(coord, 1))
        if directions['downLeft']:
            moves.extend(self.getDiagonalNorthEast(coord, -1))

        return moves

    def getKingMoves(self, coord):
        # Initiate the moves list
        moves = []

        # Get the directions that the piece can move to
        directions = self.getDirections(coord)

        row = coord.row
        col = coord.column

        if directions['up']:
            # Check if king can move to:
            # Top Left
            if directions['left']:
                target = self.board[row - 1][col - 1]
                if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                    moves.append(Coordinate(row - 1, col - 1))

            # Top Middle
            target = self.board[row - 1][col]
            if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                moves.append(Coordinate(row - 1, col))

            # Top Right
            if directions['right']:
                target = self.board[row - 1][col + 1]
                if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                    moves.append(Coordinate(row - 1, col + 1))

        if directions['down']:
            # BOTTOM LEFT
            if directions['left']:
                target = self.board[row + 1][col - 1]
                if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                    moves.append(Coordinate(row + 1, col - 1))

            # BOTTOM MIDDLE
            target = self.board[row + 1][col]
            if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                moves.append(Coordinate(row + 1, col))

            # BOTTOM RIGHT
            if directions['right']:
                target = self.board[row + 1][col + 1]
                if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                    moves.append(Coordinate(row + 1, col + 1))
            
        # MIDDLE LEFT
        if directions['left']:
            target = self.board[row][col - 1]
            if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                moves.append(Coordinate(row, col - 1))

        # MIDDLE RIGHT
        if directions['right']:
            target = self.board[row][col + 1]
            if target == "." or (self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()):
                moves.append(Coordinate(row, col + 1))

        return moves

    def isValid(self, coord):  # Je mag jezelf niet check zetten, move moet valide zijn,....
        moves = self.getAllAttackedFields(coord)
        row = coord.row
        col = coord.column
        if coord in moves:
            print("move is not valid")



    def move(self, startRow, startColumn, endRow, endColumn):
        # check if the move is on a piece and is a valid move
        if self.board[startRow][startColumn] != "." and "{}:{}".format(endRow, endColumn) in str(
                self.getPossibleMoves(startRow, startColumn)):
            self.board[endRow][endColumn] = self.board[startRow][startColumn]
            self.board[startRow][startColumn] = "."

            piece = self.board[startRow][startColumn]
            if piece.islower():
                if startColumn == 0 and startRow == 0 and not self.blackARookMoved:
                    self.blackARookMoved = True
                if startColumn == 7 and startRow == 0 and not self.blackHRookMoved:
                    self.blackHRookMoved = True
            else:
                if startColumn == 0 and startRow == 7 and not self.whiteARookMoved:
                    self.whiteARookMoved = True
                if startColumn == 7 and startRow == 7 and not self.whiteHRookMoved:
                    self.whiteHRookMoved = True
            self.moveLog.append([Coordinate(startRow, startColumn), Coordinate(endRow, endColumn)])
            self.isWhitePlayerTurn = not self.isWhitePlayerTurn
        else:
            raise Exception(startRow, endRow, endRow, endColumn, 'is not a valid move')

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
            raise Exception(notation, 'is not a valid move')

    # TODO refactor this with notationToCords
    def notationMove(self, notation):
        # strip the input of spaces
        notation = notation.strip()
        notation = notation.lower()
        if len(notation) == 4:
            startColumn = filesToColumns[notation[0]]
            startRow = ranksToRows[notation[1]]
            endColumn = filesToColumns[notation[2]]
            endRow = ranksToRows[notation[3]]

        if "{}:{}".format(endRow, endColumn) in str(self.getPossibleMoves(startRow, startColumn)):
            self.move(startRow, startColumn, endRow, endColumn)
        else:
            raise Exception(notation, 'is not a valid move')

    # TODO same refactor
    def placePieceOnNotation(self, piece, notation):
        if piece.lower() in pieces:
            notation = notation.strip()
            notation = notation.lower()
            if len(notation) == 2:
                Column = filesToColumns[notation[0]]
                Row = ranksToRows[notation[1]]
                if Row <= 7 and Column <= 7:
                    self.board[Row][Column] = piece
            else:
                raise Exception(notation, 'is not inside the board')
        else:
            raise Exception(piece, 'is not a valid piece')

    def getAllAttackedFields(self, playerIsWhite):
        moves = []
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                if self.board[row][column] != ".":
                    if playerIsWhite & self.board[row][column].isupper():
                        thesemoves = self.getPossibleMoves(row, column)
                        for i in thesemoves:
                            if i.__str__() not in str(moves):
                                moves.append(i)
                    if not playerIsWhite & self.board[row][column].islower():
                        thesemoves = self.getPossibleMoves(row, column)
                        for i in thesemoves:
                            if i.__str__() not in str(moves):
                                moves.append(i)
        return list(dict.fromkeys(moves))

    def __str__(self):
        result = ""
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                result += self.board[row][column] + " "
            result += "\n"
        return result
