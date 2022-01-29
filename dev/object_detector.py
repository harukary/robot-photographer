import math

focal_length = 0.3
img_W = 640
img_H = 480
FOV_W = 80
FOV_H = 60

xmin = 0.
xmax = 0.

ymin = 0.45
ymax = 0.55

r_x = (xmax+xmin-1.)/2
x_m = r_x*2*focal_length*math.tan(math.radians(FOV_W)/2)
# r_y =
# y_m 
print('r_x',r_x)
print('math.tan(math.radians(FOV_W)/2)',math.tan(math.radians(FOV_W)/2))
print(x_m)
theta_w = math.atan2(x_m,focal_length)

print(math.degrees(theta_w))