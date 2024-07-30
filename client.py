import numpy as np
import network.tls as tls
import network.certificates as cert
import pandas as pd
import json
import sys
from time import sleep
from porto_data_handler import trajectory_data
from porto_data_handler.trajectory_data import gps_to_cartesian, cartesian_to_gps, gps_to_cart, cart_to_gps, cart_to_gps_one
from porto_data_handler.visualise import show_on_map, print_colliding_coords, print_trajectory, print_trajectory_to_file, print_colliding_to_file
from trajectory_matching.setup import get_n_values, run_client_trajectory_matching
from trajectory_matching.util import remove_duplicates
from Crypto.Util import number


def run(ipv4: str, port: int, trajectories: np.ndarray, p: int, q: int, pair_ids: np.ndarray, index:int):
    certificate_file_name = "self-signed_client.crt"
    private_key_file_name = "private_client.key"

    # generate certificate
    cert.generate_certificate(private_key_file_name, certificate_file_name)

    # start tls client
    #print("Waiting for connection with server ...")
    connection = None
    while connection is None:
        try:
            connection = tls.create_client_socket(ipv4, port, private_key_file_name, certificate_file_name)
        except (ConnectionRefusedError, TimeoutError):
            pass
    #print("Connected with server!")

    # size trajectories
    size_trajectories = len(trajectories)

    # index for which trajectory is next
    #index = 0
    df = pd.DataFrame()
    #while index < size_trajectories:
        # get new trajectory
    trajectory = trajectories[index]
    pair_id = pair_ids[index]
        # exchange modular field (n) with other side
    n_self, n_other = get_n_values(connection, p, q)

        # output spacer
    #print("",{pair_id})

        # run trajectory matching protocol
    colliding_coordinates_space, colliding_coordinates,mode = run_client_trajectory_matching(
            traj_index=index,
            conn=connection,
            trajectory=trajectory,
            p=p, q=q,
            n_self=n_self, n_other=n_other,
        )

        # output colliding coordinates
        #colliding_coordinates_space = cart_to_gps(colliding_coordinates_space)
        #colliding_coordinates = cart_to_gps(colliding_coordinates)
        # with open("collision-v28-client-72.txt", 'a') as f:
        #         print(pair_id, file=f)
        #         print(f"Colliding coordinates found in space: {cart_to_gps(remove_duplicates(colliding_coordinates_space))}",file=f)
        #         if (len(colliding_coordinates)>0):
        #             print(f"Colliding coordinates found in time: {cart_to_gps_one(remove_duplicates(colliding_coordinates))}", file=f)
        #         print(f"Colliding coordinates found in space: {remove_duplicates(colliding_coordinates_space)}",file=f)
        #         if (len(colliding_coordinates)>0):
        #             print(f"Colliding coordinates found in time: {remove_duplicates(colliding_coordinates)}", file=f)

        
        #print_trajectory_to_file(cart_to_gps([trajectory])[0], "#00FF00","collisionsv28-70_output_client_full.txt",pair_id)
    #with open("benchmarkcollisions_client_explap_space_MD_trunc.txt", 'a') as f:
    #        print(f"{pair_id},{'client'},{json.dumps(space_eval)}", file=f)
    #if (len(colliding_coordinates)>0):
    #        with open("benchmarkcollisions_client_explap_time_MD_trunc.txt", 'a') as f:
    #            print(f"{pair_id},{'client'},{json.dumps(time_eval)}", file=f)

        # # if collision found, multiple output options
        # if len(colliding_coordinates) != 0:
        #     # output trajectory points
        #     print_trajectory(cart_to_gps([trajectory])[0], "#FF0000")
        #     dataframe_one(pair_id,"space",trajectory).to_csv("trajectory_client_70.csv", index=False)

        #     # plot trajectory on map
        #     #show_on_map(cart_to_gps([trajectory])[0], f"Trajectory {index + 1}")
        
        #     # output trajectory to print on https://mobisoftinfotech.com/tools/plot-multiple-points-on-map/
        #     #print_colliding_coords(colliding_coordinates, '#00FF00')
        #     with open("collisions_client_bakv28-70_full_points.txt", 'a') as f:
        #         print(pair_id, file=f)
        #         print(f"Colliding coordinates found in space: {cart_to_gps(colliding_coordinates_space)}",file=f)
        #         if (len(colliding_coordinates)>0): 
        #             print(f"Colliding coordinates found in time: {cart_to_gps_one(colliding_coordinates)}", file=f)
        #             print_colliding_to_file(cart_to_gps_one(colliding_coordinates),'#00FF00',"collisions_client_bakv28-70_full.txt",pair_id)
        # else:
        #     with open("collisions_client_bakv28-70_full.txt", 'a') as f:
        #         print(pair_id, file=f)
        #         print(f"Colliding coordinates found in space: {cart_to_gps(colliding_coordinates_space)}",file=f)
        #         if (len(colliding_coordinates)>0):
        #             print(f"Colliding coordinates found in time: {cart_to_gps_one(colliding_coordinates)}", file=f)
        #     if (len(colliding_coordinates)>0):
        #         print_colliding_to_file(cart_to_gps_one(colliding_coordinates),'#00FF00',"collisions_client_bakv28-70_full.txt",pair_id)

        # increment index
        #print(remove_duplicates(colliding_coordinates_space))
        #print(remove_duplicates(colliding_coordinates))
    if mode:
            df_list=[df,dataframe(pair_id,"space",colliding_coordinates_space),dataframe_one(pair_id,"time",colliding_coordinates)]
    else:
            df_list=[df,dataframe_one(pair_id,"space",colliding_coordinates_space),dataframe_one(pair_id,"time",colliding_coordinates)]
    df = pd.concat(df_list,ignore_index=True)
    #    index += 1
    df.to_csv('client_collisions_explap_MD_truncnocoll.csv', index=False) 
    # close connection
    sleep(2)
    connection.close()


def gen_safe_prime(BITS):
    qq = 1
    while not number.isPrime(qq):
        pp = number.getPrime(BITS - 1)
        qq = 2 * pp + 1
    return qq


def main():
    # set to the ipv4 of the server for Wi-Fi Ad Hoc connection
    server_ipv4_address ="192.168.0.100" #"localhost"#"131.155.69.242""192.168.0.100"
    server_port = 60000

    # set number of trajectories to run on
    sample_size = 4000

    # set file path csv
    #file_path_csv = "data/taxi_trajectories_22_00_03_09_2013.csv"
    file_path_csv = "data/nocollisions.csv"

    # load trajectory data
    print(f"Loading {file_path_csv} ...")
    df = trajectory_data.load(
        file_path=file_path_csv,
        trip_id_col="PAIR_ID",
        space_and_time_col="CLIENT"
    )
    df = df.head(sample_size)
    pair_ids = df.index.values
    trajectory_samples_gps = df["CLIENT"].to_numpy()
    trajectory_samples, proj = gps_to_cart(trajectory_samples_gps)

    # generate safe primes
    print("Generating safe primes p and q ...")
    #p = gen_safe_prime(512)
    #q = gen_safe_prime(512)
    index = int(sys.argv[1])
    p =int(sys.argv[2])
    q = int(sys.argv[3])
    run(
        ipv4=server_ipv4_address,
        port=server_port,
        trajectories=trajectory_samples,
        p=p, q=q, pair_ids=pair_ids, index=index
    )

def dataframe(id,domain,trajectories):
    if len(trajectories)==0:
        return pd.DataFrame()
    df = pd.DataFrame(columns=['id','domain','lon','lat','timestamp'])
    for trajectory in trajectories:
        if isinstance(trajectory[0][0],np.ndarray):
            for t in trajectory:
               
                    for lon, lat, t in t:
                        c = [id,domain,lon,lat,t]
                        df.loc[len(df)] = c
        elif isinstance(trajectory[0][0],np.float64):
            
            for lon, lat, t in trajectory:
                        c = [id,domain,lon,lat,t]
                        df.loc[len(df)] = c
    df.drop_duplicates(subset=None, keep="first", inplace=True)
    return df
        
def dataframe_one(id,domain,trajectories):
    
    if len(trajectories)==0:
        return pd.DataFrame()
   
       
    if isinstance (trajectories[0][0],np.ndarray):
        return dataframe(id,domain,trajectories)
    else:

        df = pd.DataFrame(columns=['id','domain','lon','lat','timestamp'])
    #for trajectory in trajectories:
        
        for lon, lat, t in trajectories:
            c = [id,domain,lon,lat,t]
            df.loc[len(df)] = c
        df.drop_duplicates(subset=None, keep="first", inplace=True)
        return df
if __name__ == "__main__":
    main()
