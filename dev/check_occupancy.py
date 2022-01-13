import numpy as np
import math

from sensor_msgs.msg import LaserScan
from collections import deque

# 0: 正面、1~:左回り
def check_occupancy(scan=None, r_min=1, r_max=6, n=8):
    incr = scan.angle_increment
    theta_a = np.pi / n
    target_a = (r_max**2-r_min**2) * theta_a
    scan_d = deque(scan.ranges)
    scan_d.rotate(int((scan.angle_min-theta_a)/incr))
    occupancy = []
    for i in range(n):
        scan_d.rotate(int(theta_a/incr))
        scan_n = list(scan_d)[:int(theta_a*2/incr)]
        for j,r in enumerate(scan_n):
            if r > r_max:
                scan_n[j] = r_max
            elif r < r_min:
                scan_n[j] = r_min
        if i == 0:
            print(target_a)
            print(r_max**2 * theta_a, sum([r**2*incr/2 for r in scan_n]), scan_n[0]**2*incr/2)
        occupied = r_max**2 * theta_a - sum([r**2*incr/2 for r in scan_n])
        occupancy.append((occupied/target_a))
    return occupancy

scan = LaserScan()
scan.angle_increment = math.radians(1.)
scan.angle_min = - np.pi
scan.angle_max = np.pi
scan.ranges = [3. for i in range(0,360)]

occupancy = check_occupancy(scan)
print(occupancy)