import turtle
import math
import numpy as np
turtle.bgcolor('black')
t = turtle.Turtle()
points =[(20,0), (50,40), (10,80), (-40,-60), (-30,60), (90,0), (120,110), (90,-80), (40,-80), (-30,-10)]

def index_to_array(index, array):
    return array[index]

def nearest_neighbor(points):
    n = len(points)
    coords = np.zeros((n,2))
    for i , point in enumerate(points):
        coords[i,0] = point[0]
        coords[i,1] = point[1]
    
    remaining_points = set(range(1,n))
    nn_points = [0]

    while remaining_points:
        last_point = nn_points[-1]
        indices = list(remaining_points)
        distances = np.linalg.norm(coords[indices] - coords[last_point], axis=1)
        nearest_point = np.argmin(distances)
        nearest_indice = indices[nearest_point]
        remaining_points.discard(nearest_indice)
        nn_points.append(nearest_indice)
    
    nn_points.append(0)
    return list(map(lambda x: index_to_array(x, points), nn_points ))

print("Sorted nearest neighbor:" + str(nearest_neighbor(points)))

sorted_points = nearest_neighbor(points)

for point in sorted_points:
    x, y = point
    t.goto(x, y)
    t.pencolor('red')
    t.speed(1)

turtle.done()
