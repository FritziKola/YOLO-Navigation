import turtle
turtle.screensize(canvwidth=300, canvheight=300, bg='black')
t = turtle.Turtle()
points =[(20,0), (50,40), (10,80), (-40,-60), (-30,60), (90,0), (120,110), (90,-80), (40,-80), (-30,-10)]
for point in points:
    x, y = point
    t.goto(x, y)
    t.pencolor('red')
    t.speed(1)

turtle.done()
