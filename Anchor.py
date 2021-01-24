class anchor:
    def __init__(self, NUM, X_POS, Y_POS, TDOA_BASE):
        """Gives x/y positions in meters and TDoA in nanoseconds"""
        self.NUM = NUM
        self.X_POS = X_POS
        self.Y_POS = Y_POS
        self.TDOA_BASE = TDOA_BASE