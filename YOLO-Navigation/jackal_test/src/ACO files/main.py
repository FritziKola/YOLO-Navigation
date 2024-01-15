import numpy as np

from ant_colony import AntColony

coordinates = np.array([(2, 0),
                    (5, 4),
                    (1, 8),
                    (-4, -6),
                    (-3, 6),
                    (9, 0),
                    (12, 11),
                    (9, -8),
                    (4, -8),
                    (-3, -1)])

ant_colony = AntColony(coordinates, n_ants=20, n_best=3, n_iterations=150, decay=0.95)
shortest_path = ant_colony.run()
print(shortest_path)

