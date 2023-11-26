from controller import Robot, Radar, RadarTarget, Keyboard
from vehicle import Driver
import math
import matplotlib.pyplot as plt
from collections import deque
import time


class Dataprocessing:
    def __init__(self, max_length):
        self.queue = deque(maxlen=max_length)

    def addqueue(self, item: tuple):
        self.queue.append(item)

    def get_distances(self):
        return list(self.queue)


# class Truck:
    # def __init__(self, step):
        # self.driver = Driver()
        # self.keyboard = Keyboard()
        # self.keyboard.enable(step)
        # self.speed = 0
        # self.Cd = 0.3
        # self.A = 2.4
        # self.m = 2100
        # self.Rc = 0.01
        # self.Vmax = 8.3
        # self.Amax = 1.2
        # self.Dmax = 7.14
        # self.Pmax = 5000
        # self.Tmax = 110
        # self.R = 0.24
        # self.Rpmax = self.Vmax*60/(6.28*self.R)
        # self.rev_speed = 25

        # first element is the cmd_trq, second is stee-
        # -ring angle, third is gear, fourth is brakes, Fifth is cmd_vel mode for future use
        # self.command = {
            # ord("W"): [0.5, 0, 1, 0, 0],
            # ord("A"): [0, -0.2, 0, 0, 0],
            # ord("D"): [0, 0.2, 0, 0, 0],
            # ord("S"): [0.5, 0, -1, 0, 0],
            # ord(" "): [0, 0, 0, 1, 0],
            # ord("E"): [0, 0, 1, 0, 30]
        # }

    # def drag(self, speed):
        # air_drag = 0.5*1.1*self.Cd*self.A * \
            # (speed**2)     # rolling and air resistance
        # rolling_res = self.Rc*9.8*self.m
        # drag_tot = air_drag + rolling_res
        # drag_decel = drag_tot/self.m
        # to make sure that drag remains
        # drag_decelf = min(self.Dmax, drag_decel)
        # bi_drag = drag_decelf / self.Dmax         # mapped to brake intensity
        # return bi_drag

    # def drive(self, cmd_trq, cmd_steer, cmd_vel):
        # self.driver.setThrottle(cmd_trq)
        # self.driver.setSteeringAngle(cmd_steer)


class Radviz:
    def __init__(self):
        self.robot = Robot()
        self.timestep = 32  # 32ms
        self.radar = Radar('radar2')
        self.radar.enable(self.timestep)
        self.targets = None
        self.current_time = None
        plt.figure(figsize=(5, 10))
        self.ax1 = plt.subplot(2, 1, 1)  # Main plot
        plt.axis([-20, 0, 0, 20])
        # self.ax1.axis([-20, 20, 0, 25])
        plt.xlabel("Longitudinal Range")
        plt.ylabel("Lateral Range")
        self.ax2 = plt.subplot(2, 1, 2)  # Subplot for distances
        plt.xlabel("Time")
        plt.ylabel("Lateral Distance")

    def sense(self):
        self.dataprocessing = Dataprocessing(max_length=3)
        start_time = time.time()
        while self.robot.step(self.timestep) != -1:
            self.current_time = time.time()-start_time
            num_targets = self.radar.getNumberOfTargets()
            self.targets = self.radar.getTargets()
            c = 0
            if self.targets is not None:
                for target in self.targets:
                    position = target.distance
                    velocity = target.speed
                    azimuth = target.azimuth  # this is radians
                    c += 1
                    self.visualize(position, azimuth)
            plt.pause(0.0032)
        # plt.clf()
        self.robot.cleanup()

    def visualize(self, distance, azimuth):
        angle = azimuth
        if azimuth > 0:
            # angle = math.pi/2-azimuth
            x = -distance*math.cos(angle+3.14/4)  # positive
            y = distance*math.sin(angle+3.14/4)  # negative
        else:
            # angle = math.pi/2+azimuth
            x = -distance*math.cos(angle+3.14/4)
            y = distance*math.sin(angle+3.14/4)
        
        # if azimuth > 0:
            # angle = math.pi/2-azimuth
            # x = distance*math.cos(angle)  # positive
            # y = distance*math.sin(angle)  # negative
        # else:
            # angle = math.pi/2+azimuth
            # x = -distance*math.cos(angle)
            # y = distance*math.sin(angle)
            
        self.dataprocessing.addqueue((self.current_time, y))
        distances = self.dataprocessing.get_distances()
        times, y_values = zip(*distances)
        # TODO: Using moving average to calculate the current velocity
        self.ax1.set_title("Detected vehicles trajectories")
        self.ax1.plot(x, y, marker="o", markersize=5, markerfacecolor="green")
        self.ax2.plot(times, y_values, label="Distances",
                      color="blue")  # Subplot
        self.ax2.set_title("Lateral Distance versus Time")


def main():
    rad = Radviz()
    rad.sense()


if __name__ == "__main__":
    main()
