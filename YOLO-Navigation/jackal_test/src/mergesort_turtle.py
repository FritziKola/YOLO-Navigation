import turtle
turtle.bgcolor('black')
t = turtle.Turtle()
points =[(20,0), (50,40), (10,80), (-40,-60), (-30,60), (90,0), (120,110), (90,-80), (40,-80), (-30,-10)]

def mergesort_wp(points):
    if len(points) <=1:
        return points
    
    #splitting list in half
    middle= len(points)//2
    left_side = points[:middle]
    right_side = points[middle:]
    #sort recursively
    left_side = mergesort_wp(left_side)
    right_side = mergesort_wp(right_side)

    return merge(left_side,right_side)

def merge(left_side, right_side):
    merge_wp = []
    left = right =0
    while left < len(left_side) and right < len(right_side):
        if left_side[left][0]< right_side[right][0]:
            merge_wp.append(left_side[left])
            left+=1
        elif left_side[left][0] > right_side[right][0]:
            merge_wp.append(right_side[right])
            right+=1
        else:
            if left_side[left][1] <= right_side[right][1]:
                merge_wp.append(left_side[left])
                left+=1
            else: 
                merge_wp.append(right_side[right])
                right+=1
    merge_wp += left_side[left:]
    merge_wp += right_side[right:]
    return merge_wp

print("Sorted merge:" + str(mergesort_wp(points)))

sorted_points = mergesort_wp(points)

for point in sorted_points:
    x, y = point
    t.goto(x, y)
    t.pencolor('red')
    t.speed(1)

turtle.done()