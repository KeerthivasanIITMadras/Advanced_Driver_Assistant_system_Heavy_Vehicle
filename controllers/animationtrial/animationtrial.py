import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from controller import Robot

class Radviz:
    def __init__(self):
        plt.figure(figsize=(6, 6))
        self.ax = plt.subplot(1, 1, 1)
        self.line, = self.ax.plot([], [], marker="o", markersize=5, markerfacecolor="green")

        self.animation = FuncAnimation(plt.gcf(), self.update, interval=32)

    def update(self, frame):
        x = np.random.uniform(-50, 50)
        y = np.random.uniform(-50, 50)
        self.line.set_data(x, y)
        print(x)

    def run_animation(self):
        robot = Robot()
        while robot.step() != -1:
            plt.pause(0.0025)
def main():
    rad = Radviz()
    rad.run_animation()

if __name__ == "__main__":
    main()
