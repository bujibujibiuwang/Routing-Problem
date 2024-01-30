"""
2-opt for TSP
author：carrot
time：10/1/2023
"""
import numpy as np
import math


class Local2opt(object):
    def __init__(self, points, init):
        self.points = points
        self.num_point = len(points)
        self.init_path = init

        self.best_path = []
        self.best_dist = 0

    def update(self, path, dist):
        self.best_path = path
        self.best_dist = dist
        return self.best_path, self.best_dist

    def two_opt(self, improvement_threshold=0.001):
        self.best_path = self.init_path
        self.best_dist = self.calculate_dist(self.best_path)
        improvement_factor = 1

        while improvement_factor > improvement_threshold:
            pre_best = self.best_dist
            for one in range(1, self.num_point - 2):
                for two in range(one + 1, self.num_point - 1):
                    one_first_point = self.points[self.best_path[one - 1]]
                    one_end_point = self.points[self.best_path[one]]
                    two_first_point = self.points[self.best_path[two]]
                    two_end_point = self.points[self.best_path[two + 1]]
                    before = self.length(one_first_point, one_end_point) + self.length(two_first_point, two_end_point)
                    after = self.length(one_first_point, two_first_point) + self.length(one_end_point, two_end_point)
                    if after < before:
                        new_path = self.swap(self.best_path, one, two)
                        new_dist = self.best_dist + after - before
                        self.update(new_path, new_dist)
            improvement_factor = 1 - self.best_dist / pre_best
        return self.best_path, self.best_dist

    def calculate_dist(self, path):
        path_dist = 0
        for i in range(len(path) - 1):
            point1 = self.points[path[i]]
            point2 = self.points[path[i+1]]
            path_dist += self.length(point1, point2)
        path_dist += self.length(self.points[path[-1]], self.points[path[0]])
        return path_dist

    def valid(self, path):
        return (len(set(path)) == self.num_point) and (sorted(path) == list(range(self.num_point)))

    @staticmethod
    def swap(path, one, two):
        a = np.concatenate((path[0:one], path[two:-len(path) + one - 1:-1], path[two + 1:])).astype(int)
        return a.tolist()

    @staticmethod
    def length(point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)