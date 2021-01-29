import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import pickle
import os

class Canvas:

    def __init__(self):
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(111, projection='3d')
        self.data_path = os.path.join('ulink_pkl', 'ulink.pkl')

    def visualize(self):
        with open(self.data_path, 'rb') as fp:
            ulink = pickle.load(fp)
        
        positions = []
        for node in ulink.values():
            positions.append(node.p)
        position_mat = np.squeeze(positions).astype(float)
        # self.ax1.claer()
        self.ax1.scatter(position_mat[:,0], position_mat[:,1], position_mat[:,2])
    
    def animate(self):
        ani = animation.FuncAnimation(self.fig, self.visualize, interval=1000)
        plt.show()