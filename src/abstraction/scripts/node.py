class Node():
    def __init(self, max, board, move=None, parent=None):
        self.parent = parent
        self.max = max
        self.board = board
        self.move = move,
        self.value = 0
        self.children = []
    
    def add_child(self, child):
        self.children.append(child)
    
    def get_children(self):
        for move in self.board.getAllValidMoves():
            b = self.move(move)
            self.add_child(Node(not self.max, b, move, self))
    
    def move(self, move):
        board = self.board.deepcopy()
        piece = board.board[move[0].row][move[0].column]
        board.move(move[0].row, move[0].column, move[1].row, move[1].column)


    
    

