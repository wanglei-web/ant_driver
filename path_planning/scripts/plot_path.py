import matplotlib.pyplot as plt

class Points:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []
		self.curvature = []
		
	def load(self, file_name):
		with open(file_name, 'r') as f:
			lines = f.readlines()
		for line in lines:
			msg = line.split()
			self.x.append(float(msg[0]))
			self.y.append(float(msg[1]))
			self.yaw.append(float(msg[2]))
	
	def plot(self):
		plt.plot(self.x, self.y)
		plt.show()
	
	def dump(self, file_name):
		with open(file_name, 'w') as f:
			for i in range(len(self.x)):
				f.write('%.3f\t%.3f\t%.3f\t%.5f\n' % (
				    self.x[i], self.y[i], self.yaw[i], self.curvature[i])
			    )
			

if __name__ == '__main__':
	raw_file = '/home/seu/Desktop/path/CLXT_001.txt'
	path = Points()
	path.load(raw_file)
	path.plot()
