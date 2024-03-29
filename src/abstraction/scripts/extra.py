
class Coordinate:
    def __init__(self, row, column):
        self.row = row
        self.column = column
        self.rep = str(self.row) + ":" + str(self.column)

    def __repr__(self):
        return self.rep

    def __eq__(self, other):
        if isinstance(other, Coordinate):
            return (self.row == other.row) and (self.column == other.column)
        else:
            return False

    def __iter__(self):
        return iter([self.row, self.column])
