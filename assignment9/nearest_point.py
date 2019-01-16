from math import sqrt

def find_nearest_point(x, y):
    if x >= 1.84 and x <= 4.16:
        if y < 2.15:
            point = (x, 0.95)
        else:
            point = (x, 3.35)
    else:
        if x < 1.84:
            new_x = 1.84 + 1.20 * ((x-1.84)/float(sqrt(((x-1.84)**2) + ((y-2.15)**2))))
            new_y = 2.15 + 1.20 * ((y-2.15)/float(sqrt(((x-1.84)**2) + ((y-2.15)**2))))
            point = (new_x, new_y)
        else:
            new_x = 4.16 + 1.20 * ((x-4.16)/float(sqrt(((x-4.16)**2) + ((y-2.15)**2))))
            new_y = 2.15 + 1.20 * ((y-2.15)/float(sqrt(((x-4.16)**2) + ((y-2.15)**2))))
            point = (new_x, new_y)
    return point

print(find_nearest_point(0, 0))
print(find_nearest_point(2, 4))
print(find_nearest_point(1, 3))
print(find_nearest_point(4, 0.2))
#print(find_nearest_point(4.5, 1.5))
