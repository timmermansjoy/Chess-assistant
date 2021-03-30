
class Coordinate:
    def __init__(self, row, column):
        self.row = row
        self.column = column

    def __repr__(self):
        return str(self.row) + ":" + str(self.column)

    def __eq__(self, other):
        if isinstance(other, Coordinate):
            return (self.row == other.row) and (self.column == other.column)
        else:
            return False
