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


    """Returns TRUE is target is of opposite color, else FALSE."""
    def canCapture(self, target):
        return ((self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()))


    """Returns TRUE for a direction if the piece is not on that edge of the board, else FALSE"""
    def getDirections(self, coord):
        directions = {'up': False, 'right': False, 'down': False, 'left': False, 'upLeft': False, 'upRight': False, 'downRight': False, 'downLeft': False}

        if coord.row > 0:
            directions['up'] = True
        if coord.row < 7:
            directions['down'] = True
        if coord.column > 0:
            directions['left'] = True
        if coord.column < 7:
            directions['right'] = True

        directions['upLeft'] = directions['up'] and directions['left']
        directions['upRight'] = directions['up'] and directions['right']
        directions['downRight'] = directions['down'] and directions['right']
        directions['downLeft'] = directions['down'] and directions['left']

        return directions


    def clearBoard(self):
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                self.board[row][column] = "."


    """Calls appropriate get moves method for the given coordinate and returns the moves"""
    def getPossibleMoves(self, row, column):
        piece = self.board[row][column]
        if piece != ".":
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
                raise Exception(piece + " is not a valid piece ??")
            return moves
        else:
            raise Exception(piece + " is not a valid piece ??")


    # compiles to machine code
    """Returns possible moves for a pawn piece"""
    def getPawnMoves(self, coord):
        moves = []
        step = -1 if self.isWhitePiece else 1
        directions = self.getDirections(coord)

        if self.board[coord.row + step][coord.column] == ".":
            moves.append(Coordinate(coord.row + step, coord.column))
            if coord.row == 3.5 + (-2.5 * step) and self.board[coord.row + (step * 2)][coord.column] == ".":
                moves.append(Coordinate(coord.row + (2 * step), coord.column))

        if directions['left']:
            target = self.board[coord.row + step][coord.column - 1]
            if target != "." and self.canCapture(target):
                moves.append(Coordinate(coord.row + step, coord.column - 1))

        if directions['right']:
            target = self.board[coord.row + step][coord.column + 1]
            if target != "." and self.canCapture(target):
                moves.append(Coordinate(coord.row + step, coord.column + 1))

        if len(self.moveLog) > 0:
            lastMoveEndCoord = self.moveLog[-1][1]
            if coord.row == 3 and self.isWhitePiece and self.board[lastMoveEndCoord.row][lastMoveEndCoord.column] == "p" and lastMoveEndCoord.row == 3:
                if lastMoveEndCoord.column == coord.column - 1:
                    moves.append(Coordinate(coord.row - 1, coord.column - 1))
                if lastMoveEndCoord.column == coord.column + 1:
                    moves.append(Coordinate(coord.row - 1, coord.column + 1))

            elif coord.row == 4 and not self.isWhitePiece and self.board[lastMoveEndCoord.row][lastMoveEndCoord.column] == "P" and lastMoveEndCoord.row == 4:
                if lastMoveEndCoord.column == coord.column - 1:
                    moves.append(Coordinate(coord.row + 1, coord.column - 1))
                if lastMoveEndCoord.column == coord.column + 1:
                    moves.append(Coordinate(coord.row + 1, coord.column + 1))
        return moves

    """Returns possible moves for a rook piece"""
    def getRookMoves(self, coord):
        moves = []
        directions = self.getDirections(coord)

        if directions['right']:
            moves.extend(self.getVertical(coord, 1))
        if directions['left']:
            moves.extend(self.getVertical(coord, -1))
        if directions['down']:
            moves.extend(self.getHorizontal(coord, 1))
        if directions['up']:
            moves.extend(self.getHorizontal(coord, -1))
        return moves

    """Returns possible vertical moves for a piece on coordinate 'start'"""
    def getVertical(self, start, step):
        moves = []
        pathBlocked = False
        current = start.row + step
        col = start.column
        end = 8 if step > 0 else -1

        while not pathBlocked and current != end:
            target = self.board[current][col]
            
            if target == ".":
                moves.append(Coordinate(current, col))
            elif self.canCapture(target):
                moves.append(Coordinate(current, col))
                pathBlocked = True
            else:
                pathBlocked = True
            current += step
        return moves

    """Returns possible horizontal moves for a piece on coordinate 'start'"""
    def getHorizontal(self, start, step):
        moves = []
        pathBlocked = False
        current = start.column + step
        row = start.row
        end = 8 if step > 0 else -1

        while not pathBlocked and current != end:
            target = self.board[row][current]

            if target == ".":
                moves.append(Coordinate(row, current))
            elif self.canCapture(target):
                moves.append(Coordinate(row, current))
                pathBlocked = True
            else:
                pathBlocked = True
            current += step
        return moves

    """Returns possible moves for a knight piece"""
    def getKnightMoves(self, coord):
        moves = []
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

        for target in targets:
            if target.row >= 0 and target.row <= 7 and target.column >= 0 and target.column <= 7:
                square = self.board[target.row][target.column]
                if square == ".":
                    moves.append(target)
                elif self.canCapture(self.board[target.row][target.column]):
                    moves.append(target)
        return moves

    """Returns possible diagonal NW moves for a piece on coordinate 'start'"""
    def getDiagonalNorthWest(self, start, step):
        moves = []
        pathBlocked = False
        currentColumn = start.column + step
        currentRow = start.row + step
        verticalEnd = 8 if step > 0 else -1
        horizontalEnd = 8 if step > 0 else -1

        while not pathBlocked and currentRow != verticalEnd and currentColumn != horizontalEnd:
            target = self.board[currentRow][currentColumn]
            if target == ".":
                moves.append(Coordinate(currentRow, currentColumn))
            elif self.canCapture(target):
                moves.append(Coordinate(currentRow, currentColumn))
                pathBlocked = True
            else:
                pathBlocked = True
            currentRow += step
            currentColumn += step
        return moves

    """Returns possible diagonal NE moves for piece on coordinate 'start'"""
    def getDiagonalNorthEast(self, start, step):
        moves = []
        pathBlocked = False
        currentColumn = start.column + step
        currentRow = start.row - step
        verticalEnd = 8 if step > 0 else -1
        horizontalEnd = 8 if step > 0 else -1

        while not pathBlocked and currentRow != verticalEnd and currentColumn != horizontalEnd:
            target = self.board[currentRow][currentColumn]
            if target == ".":
                moves.append(Coordinate(currentRow, currentColumn))
            elif self.canCapture(target):
                moves.append(Coordinate(currentRow, currentColumn))
                pathBlocked = True
            else:
                pathBlocked = True
            currentRow -= step
            currentColumn += step
        return moves

    """Returns possible moves for a bishop piece"""
    def getBishopMoves(self, coord):
        moves = []
        directions = self.getDirections(coord)
        if directions['upLeft']:
            moves.extend(self.getDiagonalNorthWest(coord, -1))
        if directions['upRight']:
            moves.extend(self.getDiagonalNorthEast(coord, 1))
        if directions['downRight']:
            moves.extend(self.getDiagonalNorthWest(coord, 1))
        if directions['downLeft']:
            moves.extend(self.getDiagonalNorthEast(coord, -1))
        return moves

    """Returns possible moves for a queen piece"""
    def getQueenMoves(self, coord):
        moves = []
        directions = self.getDirections(coord)
        if directions['up']:
            moves.extend(self.getVertical(coord, -1))
        if directions['right']:
            moves.extend(self.getHorizontal(coord, 1))
        if directions['down']:
            moves.extend(self.getVertical(coord, 1))
        if directions['left']:
            moves.extend(self.getHorizontal(coord, -1))
        if directions['upLeft']:
            moves.extend(self.getDiagonalNorthWest(coord, -1))
        if directions['upRight']:
            moves.extend(self.getDiagonalNorthEast(coord, 1))
        if directions['downRight']:
            moves.extend(self.getDiagonalNorthWest(coord, 1))
        if directions['downLeft']:
            moves.extend(self.getDiagonalNorthEast(coord, -1))
        return moves

    """Return possible moves for a king piece"""
    def getKingMoves(self, coord):
        moves = []
        directions = self.getDirections(coord)
        row = coord.row
        col = coord.column
        if directions['up']:
            target = self.board[row - 1][col]
            if target == "." or self.canCapture(target):
                moves.append(Coordinate(row - 1, col))

            if directions['left']:
                target = self.board[row - 1][col - 1]
                if target == "." or self.canCapture(target):
                    moves.append(Coordinate(row - 1, col - 1))

            if directions['right']:
                target = self.board[row - 1][col + 1]
                if target == "." or self.canCapture(target):
                    moves.append(Coordinate(row - 1, col + 1))

        if directions['down']:
            target = self.board[row + 1][col]
            if target == "." or self.canCapture(target):
                moves.append(Coordinate(row + 1, col))

            if directions['left']:
                target = self.board[row + 1][col - 1]
                if target == "." or self.canCapture(target):
                    moves.append(Coordinate(row + 1, col - 1))

            if directions['right']:
                target = self.board[row + 1][col + 1]
                if target == "." or self.canCapture(target):
                    moves.append(Coordinate(row + 1, col + 1))

        if directions['left']:
            target = self.board[row][col - 1]
            if target == "." or self.canCapture(target):
                moves.append(Coordinate(row, col - 1))

        if directions['right']:
            target = self.board[row][col + 1]
            if target == "." or self.canCapture(target):
                moves.append(Coordinate(row, col + 1))
        return moves

    """Returns whether a move is valid or not"""
    def isValid(self, coord):  # Je mag jezelf niet check zetten, move moet valide zijn,....
        moves = self.getAllAttackedFields(coord)
        row = coord.row
        col = coord.column
        if coord in moves:
            print("move is not valid")

    """Moves a piece from the startpoint to the endpoint"""
    def move(self, startRow, startColumn, endRow, endColumn):
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

    """Converts chess notation to coordinates"""
    def notationToCords(self, notation):
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
    """???"""
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
    # TODO: implement en passant

    def getAllAttackedFields(self, playerIsWhite):
        moves = []
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                if self.board[row][column] != ".":
                    if playerIsWhite and self.board[row][column].isupper():
                        if self.board[row][column] != "P":
                            thesemoves = self.getPossibleMoves(row, column)
                        if self.board[row][column] == "P":
                            thesemoves = self.getPawnThreat(row - 1, column)
                        for i in thesemoves:
                            if i.__str__() not in str(moves):
                                moves.append(i)
                    if not playerIsWhite and self.board[row][column].islower():
                        if self.board[row][column] != "p":
                            thesemoves = self.getPossibleMoves(row, column)
                        if self.board[row][column] == "p":
                            thesemoves = self.getPawnThreat(row + 1, column)
                        for i in thesemoves:
                            if i.__str__() not in str(moves):
                                moves.append(i)
        return list(dict.fromkeys(moves))

    def getPawnThreat(self, row, column):
        moves = []
        if column != 0:
            move = Coordinate(row, column - 1)
            moves.append(move)
        if column != 7:
            move = Coordinate(row, column + 1)
            moves.append(move)
        return moves

    def __str__(self):
        result = ""
        for row in range(len(self.board)):
            for column in range(len(self.board[0])):
                result += self.board[row][column] + " "
            result += "\n"
        return result
