
class Coordinate:
    def __init__(self, row, column):
        self.row = row
        self.column = column
    def __repr__(self):
        return str(self.row) + ":" + str(self.column)
