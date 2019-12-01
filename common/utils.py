class Test:
    def __init__(self, _x):
        self.x = _x
    def print_x(self):
        print(self.x)
a = Test(4)
a.y = 3
print(a.y)