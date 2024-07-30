import json
import pandas as pd
import numpy as np
from pyproj import Proj
from pyproj import CRS
from pyproj import Transformer
SCALE = 1000000


def load(file_path, trip_id_col, space_and_time_col):
    # read csv file into pandas data frame
    return pd.read_csv(
        file_path,
        index_col=trip_id_col,
        usecols=[trip_id_col, space_and_time_col],
        engine="python",
        on_bad_lines='skip'
    )


def load_raw(file_path, trip_id_col, timestamp_col, polyline_col, missing_data_col):
    # read csv file into pandas data frame
    df = pd.read_csv(
        file_path,
        index_col=trip_id_col,
        usecols=[trip_id_col, timestamp_col, polyline_col, missing_data_col],
        engine="python",
        on_bad_lines='skip'
    )
    # return data frame
    df = df[df[polyline_col] != "[]"]
    return df.loc[df[missing_data_col] != True]


def gps_to_cartesian(gps_trajectories):
    cartesian_trajectories = []
    for gps_trajectory in gps_trajectories:
        gps_trajectory = json.loads(gps_trajectory)
        cartesian_trajectory = []
        for lon, lat, t in gps_trajectory:
            # create integers from longitude and latitude
            cartesian_trajectory.append([lon_to_int(lon), lat_to_int(lat), t])
        cartesian_trajectories.append(np.array(cartesian_trajectory, dtype=np.object_))
    return cartesian_trajectories

def gps_to_cart(gps_trajectories):
    #p = Proj(proj='utm',zone=10,ellps='WGS84', preserve_units=False)
    p = Proj("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    cartesian_trajectories = []
    for gps_trajectory in gps_trajectories:
        gps_trajectory = json.loads(gps_trajectory)
        cartesian_trajectory = []
        for lon, lat, t in gps_trajectory:
            # create integers from longitude and latitude
            (x,y) = p(longitude=lon, latitude=lat)
            cartesian_trajectory.append([x,y, t])
        cartesian_trajectories.append(np.array(cartesian_trajectory, dtype=np.object_))
    return cartesian_trajectories,p

def cart_to_gps(cartesian_trajectories):
    p = Proj("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    gps_trajectories = []
    for cartesian_trajectory in cartesian_trajectories:
        gps_trajectory = []
        for x, y, t in cartesian_trajectory:
            # create integers from longitude and latitude
            (lon,lat) = p(x,y,inverse=True)
            gps_trajectory.append([lon, lat, t])
        gps_trajectories.append(gps_trajectory)
    return gps_trajectories

def cart_to_gps_one(cartesian_trajectories):
    if isinstance(cartesian_trajectories[0][0],np.ndarray):
        return cart_to_gps(cartesian_trajectories=cartesian_trajectories)
    else:
        p = Proj("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        gps_trajectories = []
        for x, y, t  in cartesian_trajectories:
            gps_trajectory = []
        
            # create integers from longitude and latitude
            (lon,lat) = p(x,y,inverse=True)
            gps_trajectory.append([lon, lat, t])
            gps_trajectories.append(gps_trajectory)
        return gps_trajectories


def cartv2_to_gps(cartesian_trajectories):
    p = Proj("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    gps_trajectories = []
    #print(cartesian_trajectories)
    for cartesian_trajectory in cartesian_trajectories:
        gps_trajectory = []
        if len(cartesian_trajectory)>0:
            #print(cartesian_trajectory[0])
        #for x, y, t in cartesian_trajectory:
            # create integers from longitude and latitude
            (lon,lat) = p(cartesian_trajectory[0],cartesian_trajectory[1],inverse=True)
            gps_trajectory.append([lon, lat, cartesian_trajectory[2]])
        gps_trajectories.append(gps_trajectory)
    return gps_trajectories

def gps_to_cartv2(gps_trajectories):
    #https://spatialreference.org/ref/sr-org/16/
    crs_4326 = CRS("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    crs_proj = CRS("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    pipeline_str = "+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
    transformer = Transformer.from_pipeline(pipeline_str)
    cartesian_trajectories = []
    for gps_trajectory in gps_trajectories:
        gps_trajectory = json.loads(gps_trajectory)
        cartesian_trajectory = []
        for lon, lat, t in gps_trajectory:
            # create integers from longitude and latitude
            (x,y) = transformer.transform(lon, lat)
            cartesian_trajectory.append([x, y, t])
        cartesian_trajectories.append(np.array(cartesian_trajectory, dtype=np.object_))
    return cartesian_trajectories




def cartesian_to_gps(cartesian_trajectories):
    gps_trajectories = []
    for cartesian_trajectory in cartesian_trajectories:
        gps_trajectory = []
        for int_lon, int_lat, t in cartesian_trajectory:
            # create integers from longitude and latitude
            gps_trajectory.append([int_to_lon(int_lon), int_to_lat(int_lat), t])
        gps_trajectories.append(gps_trajectory)
    return gps_trajectories


def lat_to_int(lat):
    return int(lat * SCALE)


def lon_to_int(lon):
    return int(lon * SCALE)


def int_to_lat(int_lat):
    return int_lat / SCALE


def int_to_lon(int_lon):
    return int_lon / SCALE
