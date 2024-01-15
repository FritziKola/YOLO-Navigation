import numpy as np
import pygame
from pygame.locals import *

class AntColony(object):

    def __init__(self, coordinates, n_ants, n_best, n_iterations, decay, alpha=1, beta=1):
        self.coordinates = coordinates
        self.distances = self.calculate_distances()
        self.pheromone = np.ones(self.distances.shape) / len(coordinates)
        self.all_inds = range(len(coordinates))
        self.n_ants = n_ants
        self.n_best = n_best
        self.n_iterations = n_iterations
        self.decay = decay
        self.alpha = alpha
        self.beta = beta
        self.screen = None
        self.my_font = None

        """
        Args:
           
            n_ants (int): Number of ants running per iteration
            n_best (int): Number of best ants who deposit pheromone
            n_iteration (int): Number of iterations
            decay (float): Rate it which pheromone decays. The pheromone value is multiplied by decay, so 0.95 will lead to decay, 0.5 to much faster decay.
            alpha (int or float): exponenet on pheromone, higher alpha gives pheromone more weight. Default=1
            beta (int or float): exponent on distance, higher beta give distance more weight. Default=1

        Example:
            ant_colony = AntColony(german_distances, 100, 20, 2000, 0.95, alpha=1, beta=2)          
        """

        # Pygame setup
        self.screen_width = 800
        self.screen_height = 600
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.clock = pygame.time.Clock()

    def calculate_distances(self):
        num_cities = len(self.coordinates)
        distances = np.zeros((num_cities, num_cities))
        for i in range(num_cities):
            for j in range(i+1, num_cities):
                x1, y1 = self.coordinates[i]
                x2, y2 = self.coordinates[j]
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                distances[i, j] = distance
                distances[j, i] = distance
        return distances

    def run(self, visualize = True):
        shortest_path = None
        pygame.init()
        self.initialize_screen()
        running = True

        all_time_shortest_path = ("placeholder", np.inf)

        #pygame.quit()
        i = 0
        while running:
            if i < self.n_iterations:
                all_paths = self.gen_all_paths()
                self.spread_pheromone(all_paths, self.n_best, shortest_path=shortest_path)
                shortest_path = min(all_paths, key=lambda x: x[1])
                print(shortest_path)
                if shortest_path[1] < all_time_shortest_path[1]:
                    all_time_shortest_path = shortest_path
                self.pheromone = self.pheromone * self.decay
                self.update_screen(all_time_shortest_path)
                self.clock.tick(30)
                i += 1

            if not visualize:
                running = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
        return self.decode_path(all_time_shortest_path[0])


    def initialize_screen(self):
        screen_width = 1200
        screen_height = 900
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        self.screen.fill((255, 255, 255))
        pygame.font.init()
        # you have to call this at the start,
        # if you want to use this module.
        self.my_font = pygame.font.SysFont('Comic Sans MS', 30)

        #pygame.display.flip()

    def update_screen(self, all_time_shortest_path):
        self.screen.fill((255, 255, 255))
        windowsize = (self.screen.get_width(), self.screen.get_height())
        coordinate_scale = self.coordinate_scale(self.coordinates, windowsize)

        for i in range(len(coordinate_scale)):
            coordinate_x = int(coordinate_scale[i][0])
            coordinate_y = int(coordinate_scale[i][1])
            pygame.draw.circle(self.screen, (0, 0, 0), ( coordinate_x, coordinate_y ), 5)
            text_surface = self.my_font.render(str(next((index for index, tpl in enumerate(all_time_shortest_path[0]) if tpl[0] == i), -1)), False, (0, 0, 0))

            # here you draw
            self.screen.blit(text_surface, (coordinate_x + 10, coordinate_y - 35))
        pygame.draw.lines(self.screen, (255, 0, 0), True, self.coordinate_scale(self.decode_path(all_time_shortest_path[0]), windowsize), 2)
        #pygame.display.flip()
        pygame.display.update()

    def coordinate_scale(self, coordinates, windowsize):
        scaled_coordinate = []
        for x, y in coordinates:
            scaled_coordinate.append((x * 30 + windowsize[0]/2, y * 30 + windowsize[1]/2))
        return scaled_coordinate

    def spread_pheromone(self, all_paths, n_best, shortest_path):
        sorted_paths = sorted(all_paths, key=lambda x: x[1])
        for path, dist in sorted_paths[:n_best]:
            for move in path:
                self.pheromone[move] += 1.0 / self.distances[move]

    def gen_path_dist(self, path):
        total_dist = 0
        for ele in path:
            total_dist += self.distances[ele]
        return total_dist

    def gen_all_paths(self):
        all_paths = []
        for i in range(self.n_ants):
            path = self.gen_path(0)
            all_paths.append((path, self.gen_path_dist(path)))
        return all_paths

    def gen_path(self, start):
        path = []
        visited = set()
        visited.add(start)
        prev = start
        for i in range(len(self.coordinates) - 1):
            move = self.pick_move(self.pheromone[prev], self.distances[prev], visited)
            path.append((prev, move))
            prev = move
            visited.add(move)
        path.append((prev, start))  # going back to where we started
        return path

    def pick_move(self, pheromone, dist, visited):
        pheromone = np.copy(pheromone)
        pheromone[list(visited)] = 0

        epsilon = 1e-10  # small constant to avoid division by zero
        row = pheromone ** self.alpha * ((1.0 / (dist + epsilon)) ** self.beta)

        norm_row = row / row.sum()
        move = np.random.choice(self.all_inds, 1, p=norm_row)[0]
        return move

    def decode_path(self, path):
        #print("Coordinates " + str(len(self.coordinates)) + " path " + str(path))
        return [self.coordinates[coords[0]] for coords in path]
