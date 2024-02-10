import urx
import numpy as np
import time
import matplotlib.pyplot as plt

def deg2rad(deg):
    return deg*np.pi/180

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def home(robot, acc, vel):
    home = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home,acc,vel)
      
def rotation_matrix_to_rotation_vector(rot):
    theta = np.arccos((np.trace(rot) - 1) / 2)
    if theta != 0:
        rx = (rot[2,1] - rot[1,2]) / (2 * np.sin(theta))
        ry = (rot[0,2] - rot[2,0]) / (2 * np.sin(theta))
        rz = (rot[1,0] - rot[0,1]) / (2 * np.sin(theta))
        return np.array([rx, ry, rz]) * theta
    else:
        return np.array([0, 0, 0])
    
def findSurface():
    p1 = np.array([-0.04, -0.77, 0.200])
    p2 = np.array([-0.04, -0.56, 0.200])
    p3 = np.array([-0.48, -0.56, 0.200])
    p4 = np.array([-0.48, -0.77, 0.200]) 
    targetSurface = (p1, p2, p3, p4)
    return targetSurface
   
def getOrientation():
    points = findSurface()
    p1 = points[0]
    p2 = points[1]
    p4 = points[3]
    v1 = p2 - p1
    v2 = p4 - p1
    z_axis = normalize(np.cross(v1, v2))
    x_axis = normalize(np.cross([0, 0, -1], z_axis)) 
    if np.linalg.norm(x_axis) == 0: 
        x_axis = normalize(np.cross([0, 1, 0], z_axis))
    y_axis = normalize(np.cross(z_axis, x_axis))
    rot = np.array([x_axis, y_axis, z_axis]).T
    rotation_vector = rotation_matrix_to_rotation_vector(rot)
    return rotation_vector

def generateWaypoints(grid_size, lift_distance, rx,ry,rz):
    points = findSurface()
    p1 = points[0]
    p2 = points[1]
    p3 = points[2]
    p4 = points[3]
    move_vector = p2 - p1
    shift_vector = normalize(p4 - p1) * grid_size
    num_passes = int(np.linalg.norm(p4 - p1) / grid_size) + 1   
    waypoints = []
    current_position = p1.copy()    
    for pass_num in range(num_passes):
        move_end = current_position + move_vector
        waypoints.append((current_position[0], current_position[1], current_position[2], rx,ry,rz))
        waypoints.append((move_end[0], move_end[1], move_end[2], rx,ry,rz))
        lifted_position = move_end + np.array([0, 0, lift_distance])
        waypoints.append((lifted_position[0], lifted_position[1], lifted_position[2], rx,ry,rz))
        if pass_num < num_passes - 1: 
            next_start_at_lifted_height = current_position + shift_vector + np.array([0, 0, lift_distance])
            waypoints.append((next_start_at_lifted_height[0], next_start_at_lifted_height[1], next_start_at_lifted_height[2], rx,ry,rz))
            waypoints.append((next_start_at_lifted_height[0], next_start_at_lifted_height[1], next_start_at_lifted_height[2], rx,ry,rz))
            next_start_lowered = next_start_at_lifted_height - np.array([0, 0, lift_distance])
            waypoints.append((next_start_lowered[0], next_start_lowered[1], next_start_lowered[2], rx,ry,rz))
            current_position = next_start_lowered
    return waypoints

def grindSurface(robot, acc, vel, plt):
    home(robot,0.5,0.5)
    waypoints = []
    gridSize = 0.01
    liftDistance = 0.01
    orientation = getOrientation()
    rx = orientation[0]
    ry = orientation[1]
    rz = orientation[2]
    if rx == 0 and ry == 0 and rz == 0:
        rx = 0
        ry = 3.14
        rz = 0
    waypoints = generateWaypoints(gridSize, liftDistance, rx, ry, rz+0.420)
    tempx=[]
    tempy=[]
    numPasses = 0
    robot.movel(waypoints[0], 0.7,0.7)
    while numPasses < 5:
        for x in waypoints:
            robot.movel(x,acc,vel)
            tempx.append(x[0])
            tempy.append(x[1])
        numPasses =+ 1
    plt.plot(tempx, tempy)
    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    plt.title('My first graph!')
    plt.show()

def main():
    robot = urx.Robot("172.16.3.114")
    robot.set_payload(2.300)
    home(robot,0.8,0.8)
    grindSurface(robot, 0.3,0.3, plt)
    home(robot,0.8,0.8)
    robot.close()
    
if __name__ == "__main__":
    main()