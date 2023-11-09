import argparse
import cv2
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                        prog= "Waypoint Annotater",
                        description='This program takes in the map created by the SLAM toolbox and creates waypoints for pure pursuit')

    # Add arguments
    parser.add_argument("-m", "--map_file", default="nsh3305_02.pgm")
    parser.add_argument("-w", "--waypoints", default="final_waypoints15.txt")
    args = parser.parse_args()

    # Read image
    map = cv2.imread(args.map_file)
    resolution = 0.05
    h, w, _ = map.shape
    waypts = np.loadtxt(args.waypoints)

    T_bottom_left = np.eye(3)
    T_bottom_left[:2, -1]  = np.array([-0.14, -6.39+w/resolution])

    # Rotate to the map
    R_map = np.eye(3)
    R_map[0, :2] = np.array([1, 0])
    R_map[1, :2] = np.array([0, -1])

    out_pts = R_map@T_bottom_left @ (resolution*waypts.T)

    np.savetxt(f"{args.waypoints}_transformed", out_pts.T)
    