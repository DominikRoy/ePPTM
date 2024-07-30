import math

import matplotlib.pyplot as plt
import numpy as np
import datetime as dt

from porto_data_handler.trajectory_data import lon_to_int, lat_to_int

BOUNDARIES_PORTO_MAP = (
    lon_to_int(-8.7353),
    lon_to_int(-8.5087),
    lat_to_int(41.1068),
    lat_to_int(41.2175)
)
MAP_PORTO = plt.imread('./data/porto.png')


def show_on_map(trajectory, name="Trajectory"):
    # split into x and y
    lon, lat, _ = zip(*trajectory)

    # plot coordinates on map
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.scatter(lon[0], lat[0], zorder=2, alpha=1, c='b', s=25)
    ax.scatter(lon[len(lon) - 1], lat[len(lat) - 1], zorder=2, alpha=1, c='r', s=25)
    ax.plot(lon, lat, zorder=1, alpha=1, color='black', linestyle='solid')
    ax.set_title(name)
    lon_min, lon_max, lat_min, lat_max = BOUNDARIES_PORTO_MAP
    ax.set_xlim(lon_min, lon_max)
    ax.set_ylim(lat_min, lat_max)
    ax.imshow(MAP_PORTO, zorder=0, extent=BOUNDARIES_PORTO_MAP, aspect="equal")
    plt.show()


def show_on_map_with_capsule(trajectory, radius, length, angle, name="Trajectory", only_one=False):
    # split into x and y
    lon, lat = zip(*trajectory)

    # plot coordinates on map
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.scatter(lon[0], lat[0], zorder=2, alpha=1, c='b', s=25)
    ax.scatter(lon[len(lon) - 1], lat[len(lat) - 1], zorder=2, alpha=1, c='r', s=25)
    ax.plot(lon, lat, zorder=1, alpha=1, color='black', linestyle='solid')
    ax.set_title(name)
    lon_min, lon_max, lat_min, lat_max = BOUNDARIES_PORTO_MAP
    ax.set_xlim(lon_min, lon_max)
    ax.set_ylim(lat_min, lat_max)
    ax.imshow(MAP_PORTO, zorder=0, extent=BOUNDARIES_PORTO_MAP, aspect="equal")

    x_dif = math.cos(angle) * length
    y_dif = math.sin(angle) * length

    ver_dif = math.cos(angle) * 2 * radius
    hor_dif = math.sin(angle) * 2 * radius

    if trajectory[0][0] == trajectory[-1][0] and trajectory[0][1] == trajectory[-1][1]:
        for ver in range(-30, 30):
            for hor in range(-30, 30):
                if only_one and (ver != 0 or hor != 0):
                    continue
                x = trajectory[0][0] + x_dif * hor + hor_dif * ver
                y = trajectory[0][1] + y_dif * hor - ver_dif * ver
                plt.Circle((x, y), radius, color="magenta")

    else:
        for ver in range(-30, 30):
            for hor in range(-30, 30):
                if only_one and (ver != 0 or hor != 0):
                    continue
                x1 = trajectory[0][0] + x_dif * hor + hor_dif * ver
                x2 = trajectory[-1][0] + x_dif * hor + hor_dif * ver
                y1 = trajectory[0][1] + y_dif * hor - ver_dif * ver
                y2 = trajectory[-1][1] + y_dif * hor - ver_dif * ver
                plt.plot(
                    [x1, x2],
                    [y1, y2],
                    alpha=0.2,
                    color="magenta",
                    lw=radius / 171,
                    solid_capstyle="round"
                )

    plt.show()


def print_colliding_coords(colliding_coordinates, color):
    col_coords_to_print = np.array(colliding_coordinates)
    print(col_coords_to_print)
    shape = col_coords_to_print.shape
    print(shape)
    col_coords_to_print = np.unique(col_coords_to_print.reshape(shape[0] * shape[1], 3), axis=0)
    print(col_coords_to_print)
    print("START COPY, to plot on https://mobisoftinfotech.com/tools/plot-multiple-points-on-map/:")
    for x, y, t in col_coords_to_print:
        print(f"{y},{x},{color},circle,'{dt.datetime.utcfromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S')}'")
    print("STOP COPY")


def print_trajectory(trajectory, color):
    print("START COPY, to plot on https://mobisoftinfotech.com/tools/plot-multiple-points-on-map/:")
    for x, y, t in trajectory:
        print(f"{y},{x},{color},circle,'{dt.datetime.utcfromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S')}'")
    print("STOP COPY")

def print_trajectory_to_file(trajectory, color, name,index):
    with open(name, 'a') as f:
        print(index,file=f)
        print("START COPY, to plot on https://mobisoftinfotech.com/tools/plot-multiple-points-on-map/:",file=f)
        for x, y, t in trajectory:
            print(f"{y},{x},{color},circle,'{dt.datetime.utcfromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S')}'", file=f)
        print("STOP COPY",file=f)

def print_colliding_to_file(colliding_coordinates, color, name,index):
    if(len(colliding_coordinates) != 0): 
        coll_size_2 = []
        coll_size_3 = []
        col_coords_to_print_2 = np.array([])
        for col in colliding_coordinates:
            coll_size_2 = np.array(col)
            shape2 = coll_size_2.shape
            coll_size_2 = np.unique(coll_size_2.reshape(shape2[0] , 3), axis=0)
            if len(col_coords_to_print_2) != 0:
                col_coords_to_print_2 = np.concatenate((col_coords_to_print_2,coll_size_2),axis=0)
            else :
                col_coords_to_print_2  = coll_size_2
        col_coords_to_print_2 = np.unique(col_coords_to_print_2,axis=0)

        # for col in colliding_coordinates:
        #     if len(col) == 2:
        #         #coll_size_2 += np.array(col)
        #         coll_size_2.append(col)
        #     else:
                
        #         coll_size_3.append(col)
        # if len(coll_size_2) !=0:
        #     col_coords_to_print_2 = np.array(coll_size_2)        
        #     shape2 = col_coords_to_print_2.shape 

        
        # #col_coords_to_print = np.array(colliding_coordinates)
        # #shape = col_coords_to_print.shape
        #     col_coords_to_print_2 = np.unique(col_coords_to_print_2.reshape(shape2[0] * shape2[1], 3), axis=0)
        # if len(coll_size_3) !=0:
        #     col_coords_to_print_3 = np.array(coll_size_3)
        #     shape3 = col_coords_to_print_3.shape
        #     col_coords_to_print_3 = np.unique(col_coords_to_print_3.reshape(shape3[0] * shape3[1], 3), axis=0)
        #     if len(coll_size_2)!=0:
        #         col_coords_to_print_2 = np.unique(np.concatenate((col_coords_to_print_2,col_coords_to_print_3),axis=0),axis=0)
        #     else:
        #         col_coords_to_print_2 = col_coords_to_print_3
    with open(name, 'a') as f:
        print(index,file=f)
        if(len(colliding_coordinates) != 0):
            print("START COPY, to plot on https://mobisoftinfotech.com/tools/plot-multiple-points-on-map/:",file=f)
            for x, y, t in col_coords_to_print_2:
                print(f"{y},{x},{color},circle,'{dt.datetime.utcfromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S')}'", file=f)
            # if len(coll_size_3) !=0:
            #     for x, y, t in col_coords_to_print_3:
            #         print(f"{y},{x},{color},circle,'{dt.datetime.utcfromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S')}'", file=f)
            print("STOP COPY",file=f)