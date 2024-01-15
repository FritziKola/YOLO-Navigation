import turtle
import math
turtle.bgcolor('black')
t = turtle.Turtle()
points =[(20,0), (50,40), (10,80), (-40,-60), (-30,60), (90,0), (120,110), (90,-80), (40,-80), (-30,-10)]


def magnitude(vect):
    return math.sqrt(math.pow(vect[0], 2) + math.pow(vect[1], 2))

def subtract(pose1, pose0):
    return [pose1[0] - pose0[0], pose1[1] - pose0[1]]

def distance(points):
    return sum(magnitude(subtract(points[i],points[i-1])) for i in range (1,len(points)))

def sort_2_opt(points):
    improved = True
    while improved:
        improved= False
        for i in range(1, len(points)-1):
            for j in range(i +1, len(points)):
                new_points = points[:]
                new_points[i:j] = points[j-1:i-1:-1]
                if distance(new_points) < distance(points):
                    points = new_points
                    improved = True
    return points
print("Sorted 2-opt: " + str(sort_2_opt(points)))
sorted_points = sort_2_opt(points)


for point in sorted_points:
    x, y = point
    t.goto(x, y)
    t.pencolor('red')
    t.speed(1)

turtle.done()