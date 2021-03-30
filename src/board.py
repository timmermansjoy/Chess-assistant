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
        self.fieldsUnderWhiteThreat = []
        self.fieldsUnderBlackThreat = []

    """Returns TRUE is target is of opposite color, else FALSE."""

    def canCapture(self, target):
        return ((self.isWhitePiece and target.islower()) or (not self.isWhitePiece and target.isupper()))

    def getDirections(self, coord):
        """Returns TRUE for a direction if the piece is not on that edge of the board, else FALSE"""

        directions = {'up': False, 'right': False, 'down': False, 'left': False, 'upLeft': False, 'upRight': False,
                      'downRight': False, 'downLeft': False}

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

    def getPossibleMoves(self, row, column):
        """Calls appropriate get moves method for the given coordinate and returns the moves"""
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

    def getPawnMoves(self, coord):
        """Returns possible moves for a pawn piece"""

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
            if not ("O - O" in self.moveLog[-1] or "O - O - O" in self.moveLog[-1]):
                lastMoveEndCoord = self.moveLog[-1][1]
                if coord.row == 3 and self.isWhitePiece and self.board[lastMoveEndCoord.row][
                    lastMoveEndCoord.column] == "p" and lastMoveEndCoord.row == 3:
                    if lastMoveEndCoord.column == coord.column - 1:
                        moves.append(Coordinate(coord.row - 1, coord.column - 1))
                    if lastMoveEndCoord.column == coord.column + 1:
                        moves.append(Coordinate(coord.row - 1, coord.column + 1))

                elif coord.row == 4 and not self.isWhitePiece and self.board[lastMoveEndCoord.row][
                    lastMoveEndCoord.column] == "P" and lastMoveEndCoord.row == 4:
                    if lastMoveEndCoord.column == coord.column - 1:
                        moves.append(Coordinate(coord.row + 1, coord.column - 1))
                    if lastMoveEndCoord.column == coord.column + 1:
                        moves.append(Coordinate(coord.row + 1, coord.column + 1))
        return moves

    def getRookMoves(self, coord):
        """Returns possible moves for a rook piece"""

        moves = []
        directions = self.getDirections(coord)

        if directions['right']:
            moves.extend(self.getHorizontal(coord, 1))
        if directions['left']:
            moves.extend(self.getHorizontal(coord, -1))
        if directions['down']:
            moves.extend(self.getVertical(coord, 1))
        if directions['up']:
            moves.extend(self.getVertical(coord, -1))
        return moves

    def getVertical(self, start, step):
        """Returns possible vertical moves for a piece on coordinate 'start'"""

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

    def getHorizontal(self, start, step):
        """Returns possible horizontal moves for a piece on coordinate 'start'"""

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

    def getKnightMoves(self, coord):
        """Returns possible moves for a knight piece"""

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

    def getDiagonalNorthWest(self, start, step):
        """Returns possible diagonal NW moves for a piece on coordinate 'start'"""
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

    def getDiagonalNorthEast(self, start, step):
        """Returns possible diagonal NE moves for piece on coordinate 'start'"""

        moves = []
        pathBlocked = False
        currentColumn = start.column + step
        currentRow = start.row - step

        # If the rowStep is > 0, the verticalEnd square is the top most square + 1 (without the + 1 it will not check the last square)
        # If the columnStep is < 0, the end square is the bottom most square - 1 (without the - 1 it will not check the last square)
        verticalEnd = 8 if step < 0 else -1
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

    def getBishopMoves(self, coord):
        """Returns possible moves for a bishop piece"""

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

    def getQueenMoves(self, coord):
        """Returns possible moves for a queen piece"""

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

    def getKingMoves(self, coord):
        """Return possible moves for a king piece"""

        moves = []
        directions = self.getDirections(coord)
        if self.board[coord.row][coord.column] == "K":
            fieldsUnderThreat = self.fieldsUnderBlackThreat
        else:
            fieldsUnderThreat = self.fieldsUnderWhiteThreat
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
        # TODO: don't ask me why this works, I've tried too many things already. Feel free to unjank
        for i in moves:
            for j in fieldsUnderThreat:
                if (i.row == j.row) & (i.column == j.column):
                    moves.remove(i)
        return moves

    def isValid(self, coord):  # Je mag jezelf niet check zetten, move moet valide zijn,....
        """Returns whether a move is valid or not"""

        moves = self.getAllAttackedFields(coord)
        row = coord.row
        col = coord.column
        if coord in moves:
            print("move is not valid")

    def move(self, startRow, startColumn, endRow, endColumn):
        """Moves a piece from the startpoint to the endpoint"""
        if (self.isWhitePlayerTurn and str(self.board[startRow][startColumn]).islower()) or (
                not (self.isWhitePlayerTurn) and str(self.board[startRow][startColumn]).isupper()):
            raise Exception("It is not your turn! Let the other player make their move first!")
        if self.board[startRow][startColumn] != "." and "{}:{}".format(endRow, endColumn) in str(
                self.getPossibleMoves(startRow, startColumn)):
            originalTarget = self.board[endRow][endColumn]
            self.board[endRow][endColumn] = self.board[startRow][startColumn]
            self.board[startRow][startColumn] = "."
            piece = self.board[startRow][startColumn]
            if (startColumn == 0 and startRow == 0) or (endColumn == 0 and endRow == 0):
                self.blackARookMoved = True
            if (startColumn == 7 and startRow == 0) or (endColumn == 7 and endRow == 0):
                self.blackHRookMoved = True
            if (startColumn == 0 and startRow == 7) or (endColumn == 0 and endRow == 7):
                self.whiteARookMoved = True
            if (startColumn == 7 and startRow == 7) or (endColumn == 7 and endRow == 7):
                self.whiteHRookMoved = True
            # when a king moves, both sides are no longer permitted to castle
            if startColumn == 4:
                if startRow == 0:
                    self.blackHRookMoved = True
                    self.blackARookMoved = True
                if startRow == 7:
                    self.whiteARookMoved = True
                    self.whiteHRookMoved = True
            self.moveLog.append([Coordinate(startRow, startColumn), Coordinate(endRow, endColumn)])
            self.isWhitePlayerTurn = not self.isWhitePlayerTurn
            if (self.board[endRow][endColumn] == "p" or self.board[endRow][endColumn] == "P") and (
                    self.board[startRow][endColumn] == "p" or self.board[startRow][endColumn] == "P") and (
                    originalTarget == "."):
                self.board[startRow][endColumn] = "."
            self.promotionCheck(Coordinate(endRow, endColumn))
        else:
            raise Exception(startRow, endRow, endRow, endColumn, 'is not a valid move')
        self.updateWhiteThreat()
        self.updateBlackThreat()

    def notationToCords(self, notation):
        """Converts chess notation to coordinates"""
        notation = notation.strip()
        notation = notation.lower()
        try:
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
        except KeyError as ke:
            raise Exception("the coordinates you passed were not valid coordinates, please try again")
        raise Exception("don't ask, you did something wrong")

    # TODO refactor this with notationToCords

    def notationMove(self, notation):
        """Move a piece with peudo chess notation i.e -> c4b6"""

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

    def updateWhiteThreat(self):
        self.fieldsUnderWhiteThreat = self.getAllAttackedFields(True)

    def updateBlackThreat(self):
        self.fieldsUnderBlackThreat = self.getAllAttackedFields(False)

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

    # TODO: promotion auto goes into queen, needs fixing
    def promotionCheck(self, endCoord):
        if (endCoord.row == 7 and self.board[endCoord.row][endCoord.column] == "p"):
            self.board[endCoord.row][endCoord.column] = "q"
        if (endCoord.row == 0 and self.board[endCoord.row][endCoord.column] == "P"):
            self.board[endCoord.row][endCoord.column] = "Q"

    # TODO: cry - I mean proofread cuz I don't know for shit how to implement a test for this
    # TODO: current logging of castle risks breaking the parsing for getPawnMoves (see line 116)
    def castling(self, white, queen):  # 2 boolean values
        # check if king is present in the correct spot (potentially redundant) +if a "white" castle takes place on a "white" turn/ "black" castle on "black" turn
        if white and self.board[7][4] == "K" and self.isWhitePlayerTurn:
            inBetweenFields = []
            # define fields that have to be free, meanwhile check for moved rooks on the related side of the board
            if queen:
                inBetweenFields.extend([Coordinate(7, 3), Coordinate(7, 2), Coordinate(7, 1)])
                if self.whiteARookMoved:
                    raise Exception("this rook already mooved you dummy, alternatively your king did")
            else:
                inBetweenFields.extend([Coordinate(7, 5), Coordinate(7, 6)])
                if self.whiteHRookMoved:
                    raise Exception("this rook already mooved you dummy, alternatively your king did")
            # verify path is empty
            for i in inBetweenFields:
                if self.board[i.row][i.column] != ".":
                    raise Exception("this is not a free path to castle on")
            inBetweenFields.append(Coordinate(7, 4))
            # verify path isn't under attack
            for i in inBetweenFields:
                for j in self.fieldsUnderBlackThreat:
                    if (i.row == j.row) & (i.column == j.column):
                        raise Exception("this path or your king is attacked, you can't do that shit here")
            # actually move pieces
            self.board[7][4] = "."
            if queen:
                self.board[7][2] = "K"
                self.board[7][3] = "R"
                self.board[7][0] = "."
                self.moveLog.append(["O - O - O"])
            else:
                self.board[7][6] = "K"
                self.board[7][5] = "R"
                self.board[7][7] = "."
                self.moveLog.append(["O - O"])
            self.isWhitePlayerTurn = False
        elif (not white) and self.board[0][4] == "k" and (not self.isWhitePlayerTurn):
            inBetweenFields = []
            # define fields that have to be free, meanwhile check for moved rooks on the related side of the board
            if queen:
                inBetweenFields.extend([Coordinate(0, 3), Coordinate(0, 2), Coordinate(0, 1)])
                if self.blackARookMoved:
                    raise Exception("this rook already mooved you dummy, alternatively your king did")
            else:
                inBetweenFields.extend([Coordinate(0, 5), Coordinate(0, 6)])
                if self.blackHRookMoved:
                    raise Exception("this rook already mooved you dummy, alternatively your king did")
            # verify path is empty
            for i in inBetweenFields:
                if self.board[i.row][i.column] != ".":
                    raise Exception("this is not a free path to castle on")
            inBetweenFields.append(Coordinate(0, 4))
            # verify path isn't under attack
            for i in inBetweenFields:
                for j in self.fieldsUnderWhiteThreat:
                    if (i.row == j.row) & (i.column == j.column):
                        raise Exception("this path or your king is attacked, you can't do that shit here")
            # actually move pieces
            self.board[0][4] = "."
            if queen:
                self.board[0][2] = "k"
                self.board[0][3] = "r"
                self.board[0][0] = "."
                self.moveLog.append(["O - O - O"])

            else:
                self.board[0][6] = "k"
                self.board[0][5] = "r"
                self.board[0][7] = "."
                self.moveLog.append(["O - O"])
            self.isWhitePlayerTurn = True
        else:
            raise Exception("I think it's not your turn, sir. PUT THE PIECES DOWN, SIR")
