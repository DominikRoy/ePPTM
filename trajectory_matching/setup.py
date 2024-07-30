import json

import numpy as np
import network.codes as codes
import network.tls as tls

from ssl import SSLSocket
from trajectory_matching.space import space_matching_initiator, space_matching_responder, space_matching_initiator_updated, space_matching_responder_updated
from trajectory_matching.time import time_matching_initiator, time_matching_responder
#from timeit import default_timer as timer
import trajectory_matching.tesselation as sd
import time
COLLISION_THRESHOLD = 50 #200
# max gps deviation constant
MAX_GPS_DEVIATION = 50
def run_client_trajectory_matching(
        traj_index: int, conn: SSLSocket, trajectory: np.array, p: int, q: int, n_self: int, n_other: int
):
    """
    Runs the space matching protocol for the client.
    """
    # (2) SEND AND RECEIVE NUMBER OF REMAINING POINTS AND DETERMINE INITIATOR
    traj_size_client, traj_size_server = get_traj_sizes(conn, trajectory)

    # I'm initiator
    if traj_size_client >= traj_size_server:
        return trajectory_matching_initiator(
            traj_index=traj_index, conn=conn, trajectory=trajectory, n=n_other
        )

    # I'm responder
    else:
        return trajectory_matching_responder(
            traj_index=traj_index, conn=conn, trajectory=trajectory, n=n_self, phi=(p - 1) * (q - 1)
        )


def run_server_trajectory_matching(
        traj_index: int, conn: SSLSocket, trajectory: np.array, p: int, q: int, n_self: int, n_other: int
):
    """
    Runs the space matching protocol for the server.
    """
    # (2) SEND AND RECEIVE NUMBER OF REMAINING POINTS AND DETERMINE INITIATOR
    traj_size_server, traj_size_client = get_traj_sizes(conn, trajectory)

    # I'm initiator
    if traj_size_client < traj_size_server:
        return trajectory_matching_initiator(
            traj_index=traj_index, conn=conn, trajectory=trajectory, n=n_other
        )

    # I'm responder
    else:
        return trajectory_matching_responder(
            traj_index=traj_index, conn=conn, trajectory=trajectory, n=n_self, phi=(p - 1) * (q - 1)
        )


def trajectory_matching_initiator(traj_index, conn, trajectory, n):
    print(f"Space matching on trajectory {traj_index} as initiator...")
    # colliding_space_coordinates = space_matching_initiator(
    #      conn=conn, trajectories=[trajectory], n=n
    # )
    #start = time.perf_counter()
    colliding_space_coordinates, reindexed_colliding, mode = space_matching_initiator_updated (
        conn=conn, trajectories=[trajectory], n=n
    )
    #end = time.perf_counter()
    #space_exectuion_time_initiator = (end - start)*1000
    print(f"Time matching on trajectory {traj_index} as initiator...")
    #start = time.perf_counter()
    colliding_time_coordinates = time_matching_initiator(
        conn=conn, collisions=colliding_space_coordinates, reindexed_collisions=reindexed_colliding, n=n, mode=mode
    )
    #end = time.perf_counter()
    #time_execution_time_initiator = (end - start)*1000
    return colliding_space_coordinates, colliding_time_coordinates, mode


def trajectory_matching_responder(traj_index, conn, trajectory, n, phi):
    print(f"Space matching on trajectory {traj_index} as responder ...")
    # colliding_space_coordinates = space_matching_responder(
    #     conn=conn, trajectories=[trajectory], n=n, phi=phi
    # )
    #start = time.perf_counter()
    colliding_space_coordinates, reindexed_colliding, indexing, mode = space_matching_responder_updated(
        conn=conn, trajectories=[trajectory], n=n,phi=phi
    )
    #end = time.perf_counter()
    #space_execution_time_responder = (end - start)*1000
    #start = time.perf_counter()
    print(f"Time matching on trajectory {traj_index} as responder...")
    colliding_time_coordinates = time_matching_responder(
        conn=conn, collisions=colliding_space_coordinates,reindexed_collisions=reindexed_colliding,indexing=indexing ,n=n, phi=phi, mode=mode
    )
    #end = time.perf_counter()
    #time_execution_time_responder = (end - start)*1000
    return colliding_space_coordinates, colliding_time_coordinates,mode


def get_traj_sizes(connection: SSLSocket, trajectory: list):
    """
    Exchanges and returns the trajectory sizes of the client and server.
    """
    # my trajectory size
    traj_size = len(trajectory)
    h, r, angle, angle_np = sd.compute_ref_capsule(
                    np.array(trajectory)[:, 0:2],
                    COLLISION_THRESHOLD,
                    MAX_GPS_DEVIATION
    )
    #traj_size = r
    # send trajectory size
    tls.send(connection, msg=json.dumps({
        "code": codes.REMAIN_POINTS_TRAJ,
        "size": traj_size
    }))
    # receive other's trajectory size
    msg = json.loads(tls.receive(connection))
    code = msg["code"]
    # check message code
    if code != codes.REMAIN_POINTS_TRAJ:
        tls.wrong_code_received(codes.REMAIN_POINTS_TRAJ, code)
    # extract trajectory size from other
    traj_size_other = msg["size"]
    # return trajectory sizes
    return traj_size, traj_size_other


def get_n_values(connection: SSLSocket, p, q):
    """
    Exchanges the modulus between the client and server.
    """
    # send n
    tls.send(connection, msg=json.dumps({
        "code": codes.MODULAR_FIELD,
        "n": p * q,
    }))
    # receive other's n
    msg = json.loads(tls.receive(connection))
    code = msg["code"]
    # check message code
    if code != codes.MODULAR_FIELD:
        tls.wrong_code_received(codes.MODULAR_FIELD, code)
    # extract n from other
    n_other = msg["n"]
    # return n and n_other
    return p * q, n_other
