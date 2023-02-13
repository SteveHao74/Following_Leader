#!/usr/bin/env python3

import time

import numpy as np
import cv2
from dynamic_window_approch import DynamicWindowApproch
# from vector_field_histogram import VectorFieldHistogram
# from artificial_potential_field import ArtificialPotentialField
import copy
import time


class Demo(object):
    def __init__(self):
        # 1 px = 0.1 m
        # That's why everything is multiplied or divided by 10.
        cv2.namedWindow('cvwindow')
        cv2.setMouseCallback('cvwindow', self.callback)
        self.drawing = False

        self.point_cloud = []
        self.draw_points = []

        # Planner Settings
        self.vel = [0.0, 0.0]
        self.pose = [30.0, 30.0, 0]
        # self.pose = [10.0, 10.0, 0]
        self.goal = None
        self.base = 2.0
        self.Planner = DynamicWindowApproch()
        # self.Planner = VectorFieldHistogram()
        # self.Planner = ArtificialPotentialField()
        self.Planner.config(
            max_speed=3.0,
            max_yawrate=np.radians(40.0),
            base=self.base)

    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                if [x, y] not in self.draw_points:
                    self.draw_points.append([x, y])
                    self.point_cloud.append([x/10, y/10])
                    self.goal = None
            else:
                self.goal = [x/10, y/10]
                # self.goal = [50,50]
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False

    def main(self):
        import argparse
        parser = argparse.ArgumentParser(description='DWA Demo')
        parser.add_argument('--save', dest='save', action='store_true')
        parser.set_defaults(save=False)
        args = parser.parse_args()
        if args.save:
            import imageio
            writer = imageio.get_writer('./dwa.gif', mode='I', duration=0.05)
        while True:
            prev_time = time.time()
            self.map = np.zeros((600, 600, 3), dtype=np.uint8)
            for point in self.draw_points:
                cv2.circle(self.map, tuple(point), 4, (255, 255, 255), -1)
            if self.goal is not None:
                cv2.circle(self.map, (int(self.goal[0]*10), int(self.goal[1]*10)),
                           4, (0, 255, 0), -1)
                if len(self.point_cloud):
                    # Planning
                    self.vel = self.Planner.planning(
                        self.pose, self.vel, self.goal, self.point_cloud)
                    
                    # Simulate motion
                    self.pose = self.Planner.motion(
                        self.pose, self.vel, self.Planner.dt)
                    # break

            pose = copy.deepcopy(self.pose)
            pose[0] = pose[0]*10
            pose[1] = pose[1]*10

            cv2.circle(self.map, (int(pose[0]), int(pose[1])), int(
                self.base*8), (0, 255, 255), -1)

            # Prevent divide by zero
            fps = int(1.0 / (time.time() - prev_time + 1e-10))
            cv2.putText(self.map, f'FPS: {fps}', (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(self.map, f'Point Cloud Size: {len(self.point_cloud)}',
                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            if args.save:
                writer.append_data(self.map)
            cv2.imshow('cvwindow', self.map)
            key = cv2.waitKey(1)
            if key == 27:
                break
            elif key == ord('r'):
                self.point_cloud = []
                self.draw_points = []

        if args.save:
            writer.close()


if __name__ == '__main__':
    Demo().main()
