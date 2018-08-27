
class Node:
	def __init__(self, x, y, parent):
		self.x = x
		self.y = y
		self.parent = parent

class NodeMatrix:
	def __init__(self, size):
		self.Matrix = [ [0 for j in range(size)] for i in range(size) ]
		self.size = size


if __name__ == "__main__":
	m = NodeMatrix(3)
	print(m.Matrix)
