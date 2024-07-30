import hashlib
import json
import math
import random
import sys
import numpy as np
import network.codes as codes
import network.tls as tls
import trajectory_matching.tesselation as sd
import trajectory_matching.util as util

from ssl import SSLSocket
from porto_data_handler.trajectory_data import cart_to_gps
from collections import defaultdict
import time

# collision threshold constant
COLLISION_THRESHOLD = 50 #200
# max gps deviation constant
MAX_GPS_DEVIATION = 50#200
# full mode (True) or truncated mode (False)
FULL_MODE = False


def space_matching_initiator(conn: SSLSocket, trajectories: np.array, n: int, i=1):
    """
    Runs the space matching protocol for the initiator (A).
    """
    # stores used nonces
    traj_nonces_vector = []
    # stores colliding coordinates
    colliding_coords_vector = []
    # stores capsule id's
    capsule_id_vector = []

    # storage before sending first message
    encr_challenge_vector, origin_vector, caps_angle_vector, caps_r_vector, caps_h_vector, caps_h_rotated_vector = [], [], [], [], [], []
    # storage before sending second message
    w_proofs = []

    # boolean indicating max caps for initiator
    A_max_caps = max(map(len, trajectories)) == 2

    # for each A-capsule
    for caps_i in range(len(trajectories)):
        # define sub trajectory
        trajectory = np.delete(trajectories[caps_i], 2, 1)

        # (3) COMPUTE REFERENCE CAPSULE
        # compute length, radius and angle
        caps_h, caps_r, caps_angle = sd.compute_ref_capsule(trajectory, COLLISION_THRESHOLD, MAX_GPS_DEVIATION)
        caps_angle_vector.append(caps_angle)
        caps_r_vector.append(caps_r)
        caps_h_vector.append(caps_h)
        # generating a rototranslation of the points in trajectory
        horizontal_trajectory = []
        for x, y in trajectory:
            new_x = x * math.cos(-caps_angle) - y * math.sin(-caps_angle)
            new_y = x * math.sin(-caps_angle) + y * math.cos(-caps_angle)
            horizontal_trajectory.append([new_x, new_y])
        # compute random origin point
        nonce_s1 = 0#random.randint(1, 2500)
        nonce_s2 = 0#random.randint(1, 2500)
        caps_origin_x = horizontal_trajectory[0][0] - caps_r - nonce_s1 * caps_h
        caps_origin_y = horizontal_trajectory[0][1] - caps_r - nonce_s2 * 2 * caps_r
        origin_vector.append([caps_origin_x, caps_origin_y])
        # generating a rototranslation of the points in trajectory
        shifted_trajectory = []
        for x, y in horizontal_trajectory:
            new_x = x - caps_origin_x
            new_y = y - caps_origin_y
            shifted_trajectory.append([new_x, new_y])

        # (4) MAP CAPSULE TO A UNIQUE IDENTIFIER
        # unique identifier vector
        x_1, y_1 = shifted_trajectory[0]
        id_x, id_y = util.get_capsule_identifier(x_1, y_1, caps_r, caps_h)
        capsule_id = util.get_hashed_capsule_identifier(id_x, id_y)
        capsule_id_vector.append(capsule_id)
        # (5) SET d
        d = capsule_id

        # (6) EXTRACTS A NONCE, AND COMPUTES ENCRYPTED CHALLENGE
        # extract a nonce
        nonce = random.randint(1, n)
        traj_nonces_vector.append(nonce)
        # compute encrypted challenges
        encr_challenge_vector.append(pow(nonce, d, n))

    # (7) SEND: ENCRYPTED CHALLENGE, CAPSULE ORIGIN POINT, CAPSULE ANGLE, CAPSULE RADIUS, CAPSULE LENGTH
    tls.send(conn, json.dumps({
        "code": codes.SPACE_CAPS_INFO,
        "challenges": encr_challenge_vector,
        "origins": origin_vector,
        "angles": caps_angle_vector,
        "radiuss": caps_r_vector,
        "lengths": caps_h_vector,
        "max_caps_A": A_max_caps
    }))

    table = defaultdict(list)
    # (12) CHECK IF CAPSULES IDENTIFIERS ARE THE SAME
    # receive capsule info
    msg = json.loads(tls.receive(conn))
    code = msg["code"]
    # check message code
    if code != codes.SPACE_ENCR_RESPONSES:
        tls.wrong_code_received(codes.SPACE_ENCR_RESPONSES, code)
    # boolean indicating max caps for responder
    B_max_caps = msg["max_caps_B"]
    # for each A-capsule
    for caps_i in range(len(trajectories)):
        # extract vector with encrypted responses
        encr_responses = msg["responses"][caps_i]
        # compute check value w
        nonce = traj_nonces_vector[caps_i]
        w = hashlib.sha256(str(nonce % n).encode('utf-8')).hexdigest()
        # for each B-capsule
        for j in range(len(encr_responses)):
            encr_rsp = encr_responses[j]
            if encr_rsp == w:
                # B collides with this A capsule
                colliding_coords_vector.append(trajectories[caps_i])
                table[caps_i].append(j)
                #print(table)
                #print(i)

        # (13) COMPUTE ENCRYPTED PROOF OF PROXIMITY
        # compute ecnrypted proof
        w_proof = hashlib.sha256((str(nonce % n) + str(capsule_id_vector[caps_i])).encode('utf-8')).hexdigest()
        # store encrypted proof
        w_proofs.append(w_proof)

    # (14) SEND TO B THE ENCRYPTED PROOFS OF PROXIMITY
    tls.send(conn, json.dumps({
        "code": codes.SPACE_PROOF,
        "proofs": w_proofs,
    }))

    # no collisions -> return no collisions
    if len(colliding_coords_vector) == 0:
        return []

    # TRUNCATED MODE end condition
    elif not FULL_MODE and (A_max_caps or B_max_caps):
        return cartesian_to_gps(colliding_coords_vector)

    # FULL MODE end condition
    elif FULL_MODE and A_max_caps and B_max_caps:
        return cartesian_to_gps(colliding_coords_vector)

    # no end condition -> next iteration
    else:
        # remove duplicates
        colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
        # split colliding coordinates sets
        trajectories = []
        for sub_traj in colliding_coords_vector:
            #TODO check here for splitting
            if len(sub_traj) > 2:
                trajectories += util.split_in_sections(sub_traj, 2, 1)
            else:
                trajectories.append(sub_traj)
        # next iteration
        return space_matching_initiator(conn=conn, trajectories=trajectories, n=n, i=i+1)


def space_matching_responder(conn: SSLSocket, trajectories: np.array, n: int, phi: int, i=1, table=defaultdict(list)):
    """
    Runs the space matching protocol for the responder (B).
    """
    # stores mappings from capsule ID to coordinates
    capsule_to_coords = []
    # stores colliding coordinates
    colliding_coords_vector = []
    # stores capsule id's
    capsule_id_vector = []

    # stores received challenges
    challenge_vector = []
    # stores send responses and hashed responses
    response_vector, hashed_response_vector = [], []
    res = []
    # receive capsule info from initiator
    msg = json.loads(tls.receive(conn))
    code = msg["code"]
    # check message code
    if code != codes.SPACE_CAPS_INFO:
        tls.wrong_code_received(codes.SPACE_CAPS_INFO, code)

    # boolean indicating max capsules reached
    A_max_caps = msg["max_caps_A"]
    B_max_caps = max(map(len, trajectories)) == 2

    # for each A-capsule
    for caps_i in range(len(msg["challenges"])):
        # sub-trajectory storage
        encr_rsps = []
        responses = []
        capsule_ids = []
        capsule_to_coords_sub = {}

        # extract capsule info
        encr_challenge = msg["challenges"][caps_i]
        challenge_vector.append(encr_challenge)
        caps_origin_x, caps_origin_y = msg["origins"][caps_i]
        caps_angle = msg["angles"][caps_i]
        caps_r = msg["radiuss"][caps_i]
        caps_h = msg["lengths"][caps_i]

        #print(table)
        #print(len(msg["challenges"]))
        # for each B-capsulelen(trajectories)
        for traj_i in range(len(trajectories)):
            # if i > 1:
            #     #table = dict(table)
            #     print(i)
            #     print(table)

            #     if caps_i not in table[traj_i]: continue
            
           # table[traj_i].append(caps_i)
           # print(table)
            trajectory = np.delete(trajectories[traj_i], 2, 1)
            # (8) MAP TRAJECTORY INTO SPACE TESSELLATION INSTRUCTED BY A
            # generating a rototranslation of the points in trajectory
            shifted_trajectory = []
            for x, y in trajectory:
                new_x = (x * math.cos(-caps_angle) - y * math.sin(-caps_angle)) - caps_origin_x
                new_y = (x * math.sin(-caps_angle) + y * math.cos(-caps_angle)) - caps_origin_y
                shifted_trajectory.append([new_x, new_y, x, y])
            # get B capsule
            h, r, angle = sd.compute_ref_capsule(
                np.array(shifted_trajectory)[:, 0:2],
                COLLISION_THRESHOLD,
                MAX_GPS_DEVIATION
            )
            # identify capsule identifiers
            capsule_ids_traj = util.get_colliding_caps_ids(
                np.array(shifted_trajectory)[:, 0:2], r, angle, caps_r, caps_h
            )
            # store capsule id's
            capsule_ids += capsule_ids_traj
            # map caps id's to coordinates
            for caps_id in capsule_ids_traj:
                current_mapping = capsule_to_coords_sub.get(caps_id, [])
                current_mapping.append(np.array([[x, y, t] for x, y, t in trajectories[traj_i]]))
                capsule_to_coords_sub.update({caps_id: current_mapping})

            # (9) FOR EACH CAPSULE THAT THIS TRAJECTORY COLLIDES WITH, COMPUTE PARAMETER E
            e = []
            for k in range(len(capsule_ids_traj)):
                d = capsule_ids_traj[k]
                e.append(pow(d, -1, phi))

            # (10) FOR EACH CAPSULE THAT THIS TRAJECTORY COLLIDES WITH, COMPUTE ENCRYPTED RESPONSE
            for k in range(len(capsule_ids_traj)):
                c = encr_challenge
                response_number = pow(c, e[k], n)
                responses.append(response_number)
                encr_rsps.append(hashlib.sha256(str(response_number).encode('utf-8')).hexdigest())
                # print(encr_rsps)
                # [res.append(x) for x in encr_rsps if x not in res]
                # encr_rsps = res

        # store responses
        response_vector.append(responses)
        # store hashed responses
        hashed_response_vector.append(encr_rsps)
        # store capsule ids
        capsule_id_vector.append(capsule_ids)
        # store capsule to coordinates mapping
        capsule_to_coords.append(capsule_to_coords_sub)
    # (11) SEND: VECTOR WITH ENCRYPTED RESPONSES
    tls.send(conn, json.dumps({
        "code": codes.SPACE_ENCR_RESPONSES,
        "responses": hashed_response_vector,
        "max_caps_B": B_max_caps
    }))


    future_table = defaultdict(list)    

    # (17) CHECK WHETHER YOUR CAPSULE IDENTIFIERS MATCHED WITH THE INITIATORS' TRAJECTORY -> IF NOT: STOP PROTOCOL
    # receive capsule info
    msg = json.loads(tls.receive(conn))
    code = msg["code"]
    # check message code
    if code != codes.SPACE_PROOF:
        tls.wrong_code_received(codes.SPACE_PROOF, code)

    #print("proofs")
    #print(msg["proofs"])
    for caps_i in range(len(msg["proofs"])):
        # extract proof of proximity
        w_proof = msg["proofs"][caps_i]
        for j in range(len(capsule_id_vector[caps_i])):
            # get capsule identifier
            caps_id = capsule_id_vector[caps_i][j]
            # generate w check
            w_check = hashlib.sha256((str(response_vector[caps_i][j]) + str(caps_id)).encode('utf-8')).hexdigest()
            # check collission between capsule identifiers
            if w_check == w_proof:
                # get colliding coordinates
                colliding_coords_vector += capsule_to_coords[caps_i].get(caps_id, [])
                # only one match possible so break from loop
                val = [caps_i+caps_i,caps_i+caps_i+1]
                for item in val:
                        future_table[caps_i+caps_i].append(caps_i)
                        future_table[caps_i+caps_i+1].append(caps_i)
                #print(colliding_coords_vector)
                #print("round:")
                #print(i)
                break

    # no collisions -> return no collisions
    if len(colliding_coords_vector) == 0:
        return []

    # TRUNCATED MODE end condition
    elif not FULL_MODE and (A_max_caps or B_max_caps):
        return cartesian_to_gps(colliding_coords_vector)

    # FULL MODE end condition
    elif FULL_MODE and A_max_caps and B_max_caps:
        return cartesian_to_gps(colliding_coords_vector)

    # no end condition -> next iteration
    else:
        # (18) TAKE THE CORRESPONDING SET OF COLLIDING COORDINATES
        #      - FOR EACH, EXECUTE (8), (9), (10) AND (11)
        # remove duplicates
        colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
        # split colliding coordinates sets
        trajectories = []
        for sub_traj in colliding_coords_vector:
            if len(sub_traj) > 2:
                trajectories += util.split_in_sections(sub_traj, 2, 1)
            else:
                trajectories.append(sub_traj)
        # next iteration
        #print(future_table[0])
        return space_matching_responder(conn=conn, trajectories=trajectories, n=n, phi=phi, i=i+1, table = future_table)










####NEW VERSION


def space_matching_initiator_updated(conn: SSLSocket, trajectories: np.array, n: int, i=1, wproofs_i= [], colliding_coords_vector = [], trajectories_dict = {},wproofdict={},responder_payload = 0):
    """
    Runs the space matching protocol for the initiator (A).
    """
    # stores used nonces
    traj_nonces_vector = []
    traj_nonces_dict = {}
    # stores colliding coordinates
    
    # stores capsule id's
    capsule_id_vector = []
    capsule_id_dict = {}

    # storage before sending first message
    encr_challenge_vector, origin_vector, caps_angle_vector, caps_r_vector, caps_h_vector, caps_h_rotated_vector = [], [], [], [], [], []
    encr_challenge_dict, origin_dict, caps_angle_dict, caps_r_dict, caps_h_dict, caps_h_rotated_dict = {}, {}, {}, {}, {}, {}
    # storage before sending second message
    w_proofs = [""]*2**(i-1)
    w_proofs_dict = {}
    # define in signature

    # boolean indicating max caps for initiator
    space_exectuion = 0
    A_max_caps = max(map(len, trajectories)) == 2
    #print("round:",{i})
    
    if i == 1: 
        #dt[i]={}
        #dt[i]['role']='initiator'
        #dt[i]['domain']='space'
        #dt[i]['capsules']=len(trajectories)
        #dt[i]['capsulepoints']={}
    # for each A-capsule
        #start = time.perf_counter()
        for caps_i in range(len(trajectories)):
            #dt[i]['capsulepoints'][caps_i]=len(trajectories[caps_i])
        # define sub trajectory
            trajectory = np.delete(trajectories[caps_i], 2, 1)

        # (3) COMPUTE REFERENCE CAPSULE
        # compute length, radius and angle
            caps_h, caps_r, caps_angle, caps_angle_np = sd.compute_ref_capsule(trajectory, COLLISION_THRESHOLD, MAX_GPS_DEVIATION)
            caps_angle_vector.append(caps_angle)
            caps_r_vector.append(caps_r)
            caps_h_vector.append(caps_h)
        # generating a rototranslation of the points in trajectory
            #print(trajectory)
            horizontal_trajectory = []
            #caps_angle = float("{:.3f}".format(caps_angle))
            #caps_angle = np.round(caps_angle,4)
            for x, y in trajectory:
                #print("rotator")
                #print(-caps_angle)
                #print(math.cos(-caps_angle))
                #print(math.sin(-caps_angle))
                #print([x,y])
                #print("end")
                #x = float("{:.3f}".format(x))
                #y = float("{:.3f}".format(y))
                new_x = x * math.cos(-caps_angle)- y * math.sin(-caps_angle)
                new_y = x * math.sin(-caps_angle) + y * math.cos(-caps_angle)
                horizontal_trajectory.append([new_x, new_y])
            caps_h_rotated = sd.distance(horizontal_trajectory[0],horizontal_trajectory[-1]) + 2* caps_r
            
            caps_h_rotated_vector.append(caps_h_rotated)
        
        # compute random origin point
            nonce_s1 = random.randint(1, 2500)
            nonce_s2 = random.randint(1, 2500)
            nonce_s1 = 1 #305 #random.getrandbits(8)
            nonce_s2 = 1 #981 #random.getrandbits(8)
            #print("origin start:")
            #print(caps_r)
            #print(caps_h_rotated)
            #print(nonce_s1)
            #print(nonce_s2)
            #print( horizontal_trajectory[0][0])
            #print( horizontal_trajectory[0][1])
            #print("origin end:")
            caps_origin_x = horizontal_trajectory[0][0] - caps_r + (nonce_s1 * caps_h_rotated)
            caps_origin_y = horizontal_trajectory[0][1] - caps_r + (nonce_s2 * 2 * caps_r)
            #caps_origin_x = trajectory[0][0] - caps_r + (nonce_s1 * caps_h_rotated)
            #caps_origin_y = trajectory[0][1] - caps_r + (nonce_s2 * 2 * caps_r)
            origin_vector.append([caps_origin_x, caps_origin_y])
        # generating a rototranslation of the points in trajectory
            shifted_trajectory = []
            #for x, y in horizontal_trajectory:

            for x, y in horizontal_trajectory:
            #for x, y in trajectory:
                new_x = x - caps_origin_x
                new_y = y - caps_origin_y
                shifted_trajectory.append([new_x, new_y])
                #print("trajectory start:")
                #print([x,y])
                #print([new_x,new_y])
                #print([caps_origin_x,caps_origin_y])
                #print("trajectory end")
                #x_1, y_1 = shifted_trajectory[0]
                #id_x, id_y = util.get_capsule_identifier(new_x, new_y, caps_r, caps_h)
                #capsule_id = util.get_hashed_capsule_identifier(id_x, id_y)
                #capsule_id_vector.append(capsule_id)

        # (4) MAP CAPSULE TO A UNIQUE IDENTIFIER
        # unique identifier vector

            x_1, y_1 = shifted_trajectory[0]
            id_x, id_y = util.get_capsule_identifier(x_1, y_1, caps_r, caps_h_rotated)
            capsule_id = util.get_hashed_capsule_identifier(id_x, id_y)
            capsule_id_vector.append(capsule_id)
            #print("initator ids:")
            #print([trajectory[0][0],trajectory[0][1]])
            #print([horizontal_trajectory[0][0],horizontal_trajectory[0][1]])
            #print([x_1,y_1])
            #print([id_x,id_y])
            #print([caps_origin_x,caps_origin_y])
        # (5) SET d
            d = capsule_id

        # (6) EXTRACTS A NONCE, AND COMPUTES ENCRYPTED CHALLENGE
        # extract a nonce
            nonce = random.randint(1, n)
            traj_nonces_vector.append(nonce)

            encr_challenge_vector.append(pow(nonce, d, n))

    # (7) SEND: ENCRYPTED CHALLENGE, CAPSULE ORIGIN POINT, CAPSULE ANGLE, CAPSULE RADIUS, CAPSULE LENGTH
     
        tls.send(conn, json.dumps({
            "code": codes.SPACE_CAPS_INFO,
            "challenges": encr_challenge_vector,
            "origins": origin_vector,
            "angles": caps_angle_vector,
            "radiuss": caps_r_vector,
            "lengths": caps_h_vector,
            "max_caps_A": A_max_caps
        }))
        #end = time.perf_counter()
        #space_exectuion += (end - start)*1000
    else:
        #dt[i]={}
        #dt[i]['role']='initiator'
        #dt[i]['domain']='space'
        #dt[i]['capsules']=len(trajectories)
        #dt[i]['capsulepoints']={}
        #start = time.perf_counter()
            # for each A-capsule
        for key, trajectories_values in trajectories_dict.items():
        # define sub trajectory
            caps_angle_dict[key]={}
            caps_r_dict[key]={}
            caps_h_dict[key]={}
            caps_h_rotated_dict[key]={}
            origin_dict[key]={}
            capsule_id_dict[key]={}
            traj_nonces_dict[key]={}
            encr_challenge_dict[key]={}
            one_caps = False
            for traj in range(len(trajectories_values)):
                #np.int64
                if isinstance(trajectories_values[traj][0],np.float64) :
                     trajectory = np.delete(trajectories_values, 2, 1)
                     #traj_temp = trajectories_dict[value]
                     one_caps = True
                elif isinstance(trajectories_values[traj][0],np.ndarray):
                
                    trajectory = np.delete(trajectories_values[traj], 2, 1)
                    #traj_temp = trajectories_dict[value][l]
                #dt[i]['capsulepoints'][key+traj]=len(trajectory)
                

        # (3) COMPUTE REFERENCE CAPSULE
        # compute length, radius and angle
                caps_h, caps_r, caps_angle, caps_angle_np = sd.compute_ref_capsule(trajectory, COLLISION_THRESHOLD, MAX_GPS_DEVIATION)
                caps_angle_dict[key][traj]=(caps_angle)
                caps_r_dict[key][traj]=(caps_r)
                caps_h_dict[key][traj]=(caps_h)
        # generating a rototranslation of the points in trajectory
                horizontal_trajectory = []
                test=[]
                #print(-caps_angle)
                #print(-caps_angle_np)
                #caps_angle = float("{:.3f}".format(caps_angle))
                #print(trajectory)
                #caps_angle = np.round(caps_angle,4)
                for x, y in trajectory:
                   # x = float("{:.3f}".format(x))
                   # y = float("{:.3f}".format(y))
                    #new_x = x * float("{:.3f}".format(math.cos(-caps_angle) ))- y * float("{:.3f}".format(math.sin(-caps_angle)))
                    #new_y = x * float("{:.3f}".format(math.sin(-caps_angle))) + y * float("{:.3f}".format(math.cos(-caps_angle)))
                    new_x = x * math.cos(-caps_angle)- y *  math.sin(-caps_angle)
                    new_y = x * math.sin(-caps_angle) + y * math.cos(-caps_angle)
                    #new_x = x * np.round(math.cos(-caps_angle),4)- y * np.round(math.sin(-caps_angle),4)
                    #new_y = x * np.round(math.sin(-caps_angle),4) + y * np.round(math.cos(-caps_angle),4)
                    horizontal_trajectory.append([new_x, new_y])
                    test.append([x,y, new_x, new_y])

                    #print("trajectory start:")
                    #print(-caps_angle)
                   # print(math.cos(-caps_angle))
                    #print(math.sin(-caps_angle))
                    #print(np.cos(-caps_angle))
                    #print(np.sin(-caps_angle))
                    #print(np.rad2deg(-caps_angle))
                    #print([x,y])
                    #print([x,y])
                    #print([new_x,new_y])
                    #print("trajectory end")

                #print("horizontal points distance:",{sd.distance(horizontal_trajectory[0],horizontal_trajectory[-1])})
                caps_h_rotated = sd.distance(horizontal_trajectory[0],horizontal_trajectory[-1]) + 2* caps_r
                caps_h_rotated_dict[key][traj]=(caps_h_rotated)
                #caps_angle = 0 
        # compute random origin point
                nonce_s1 = random.randint(1, 2500)
                nonce_s2 = random.randint(1, 2500)
                #print(sys.getsizeof(nonce_s1 ))
                #print(sys.getsizeof(nonce_s2 ))
                nonce_s1 = 1 #305 #random.getrandbits(8)
                nonce_s2 = 1 #981 # random.getrandbits(8)
                #print("origin start:")
                #print(caps_r)
                #print(caps_h_rotated)
                #print(nonce_s1)
                #print(nonce_s2)
                #print( horizontal_trajectory[0][0])
                #print( horizontal_trajectory[0][1])
                #print("origin end:")
                caps_origin_x = horizontal_trajectory[0][0] - caps_r + (nonce_s1 * caps_h_rotated)
                caps_origin_y = horizontal_trajectory[0][1] - caps_r + (nonce_s2 * 2 * caps_r)
                #caps_origin_x = trajectory[0][0] - caps_r + (nonce_s1 * caps_h_rotated)
                #caps_origin_y = trajectory[0][1] - caps_r + (nonce_s2 * 2 * caps_r)
                origin_dict[key][traj]=([caps_origin_x, caps_origin_y])
        # generating a rototranslation of the points in+ trajectory
                shifted_trajectory = []
                test_shifted = []
                #
                for x, y in horizontal_trajectory:
                #for x, y in trajectory:
                    new_x = x - caps_origin_x
                    new_y = y - caps_origin_y
                    shifted_trajectory.append([new_x, new_y])
                    test_shifted.append([x,y, new_x, new_y])
                    #print([x,y])
                    #print([new_x,new_y])
                #print("shifted points distance:",{sd.distance(shifted_trajectory[0],shifted_trajectory[-1])})
                res = "\n".join("{} {}".format(a, b) for a,b in zip(test, test_shifted))
                #print(res)
        # (4) MAP CAPSULE TO A UNIQUE IDENTIFIER
        # unique identifier vector
                x_1, y_1 = shifted_trajectory[0]
                #id_x = 0
                #id_y = 0
                #if A_max_caps:
                #    id_x, id_y = util.get_capsule_identifier_last(x_1, y_1, caps_r, caps_h_rotated)
                #else:
                id_x, id_y = util.get_capsule_identifier(x_1, y_1, caps_r, caps_h_rotated)
                #print(type(id_x))
                capsule_id = util.get_hashed_capsule_identifier(id_x, id_y)
                capsule_id_dict[key][traj]=(capsule_id)
                #print("initator ids:")
                #print([trajectory[0][0],trajectory[0][1]])
                #print([horizontal_trajectory[0][0],horizontal_trajectory[0][1]])
                #print([x_1,y_1])
                #print([id_x,id_y])
                #print([caps_origin_x,caps_origin_y])
        # (5) SET d
                d = capsule_id

        # (6) EXTRACTS A NONCE, AND COMPUTES ENCRYPTED CHALLENGE
        # extract a nonce
                nonce = random.randint(1, n)
                traj_nonces_dict[key][traj]=(nonce)
        # compute encrypted challenges
        #encr_challenge_vector[caps_i]=(pow(nonce, d, n))
                encr_challenge_dict[key][traj]=(pow(nonce, d, n))
                if one_caps: break
        if i-1==1: 
            tls.send(conn, json.dumps({
                "code": codes.SPACE_CAPS_INFO,
                "challenges": encr_challenge_dict,
                "origins": origin_dict,
                "angles": caps_angle_dict,
                "radiuss": caps_r_dict,
                "lengths": caps_h_dict,
                "max_caps_A": A_max_caps,
                "proofs": wproofs_i
            }))
        else:
            tls.send(conn, json.dumps({
                "code": codes.SPACE_CAPS_INFO,
                "challenges": encr_challenge_dict,
                "origins": origin_dict,
                "angles": caps_angle_dict,
                "radiuss": caps_r_dict,
                "lengths": caps_h_dict,
                "max_caps_A": A_max_caps,
                "proofs": wproofdict
            }))
        #end = time.perf_counter()
        #space_exectuion += (end - start)*1000
    #print(caps_r_dict)
    #table = defaultdict(list)
    # (12) CHECK IF CAPSULES IDENTIFIERS ARE THE SAME
    # receive capsule info
    #start = time.perf_counter()
    msg = tls.receive(conn)
    #responder_payload = sys.getsizeof(msg)
    msg = json.loads(msg)
    
    #print(responder_payload)
    code = msg["code"]
    # boolean indicating max caps for responder
    B_max_caps = msg["max_caps_B"]
    # check message code

    if code == codes.STOP_ITERATION:
        
        if len(colliding_coords_vector) == 0:
            return [], responder_payload

        # TRUNCATED MODE end condition
        elif not FULL_MODE and (A_max_caps or B_max_caps):
            #colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            #return cart_to_gps(colliding_coords_vector), responder_payload
            return colliding_coords_vector, responder_payload

        # FULL MODE end condition
        elif FULL_MODE and A_max_caps and B_max_caps:
            #colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            #return cart_to_gps(colliding_coords_vector), responder_payload
            return colliding_coords_vector, responder_payload
    elif code == codes.SPACE_ENCR_RESPONSES:
        
    
        colliding_coords_vector = []

        outer = 0

        in_dict = {}
        if i > 1:
         
         for key, values in trajectories_dict.items():
           if len(msg["responses2"][str(key)]) !=0:
            w_proofs_dict[key]={}
            one_caps = False
            for value in range(len(values)):
                if isinstance(values[value][0],np.float64) :#np.int64

                     one_caps = True
                w_proofs_dict[key][value]=[]
                nonce = traj_nonces_dict[key][value]
                w = hashlib.sha256(str(nonce % n).encode('utf-8')).hexdigest()
                for j in range(len(msg["responses2"][str(key)][str(value)])):
                    encr_responses = msg["responses2"][str(key)][str(value)][j]

                    for l in range(len(encr_responses)):
                        if encr_responses[l] == w:
                            colliding = trajectories_dict[key][value]
                            if one_caps:
                                colliding = trajectories_dict[key]
                            colliding_coords_vector.append(colliding)
                            in_dict[colliding[0][2]]=[(2*key+value),colliding[0]]

                    #array2d[caps_i][j]=1
                        # (13) COMPUTE ENCRYPTED PROOF OF PROXIMITY
            # compute ecnrypted proof
                w_proof = hashlib.sha256((str(nonce % n) + str(capsule_id_dict[key][value])).encode('utf-8')).hexdigest()
            # store encrypted proof
                #current ids of round i and not i-1
                w_proofs_dict[key][value].append(w_proof)
                if one_caps: break
        else:       
         for caps_i in range(len(trajectories)):
                outer = outer+1
            #if test[caps_i]:
            # extract vector with encrypted responses
                encr_responses = msg["responses2"][caps_i]
            # compute check value w
                nonce = traj_nonces_vector[caps_i]
                w = hashlib.sha256(str(nonce % n).encode('utf-8')).hexdigest()
                #next_caps = True
            # for each B-capsule
                for j in range(len(encr_responses)):
                    #innter = inner + 1
                 
                    encr_rsp = encr_responses[j]
                    for k in range(len(encr_rsp)):
                #Matrix[caps_i]=0
                        if encr_rsp[k] == w:

                            colliding_coords_vector.append(trajectories[caps_i])


                
            # (13) COMPUTE ENCRYPTED PROOF OF PROXIMITY
            # compute ecnrypted proof
                w_proof = hashlib.sha256((str(nonce % n) + str(capsule_id_vector[caps_i])).encode('utf-8')).hexdigest()
            # store encrypted proof
                w_proofs[caps_i]=w_proof
  
        # (14) SEND TO B THE ENCRYPTED PROOFS OF PROXIMITY
        # tls.send(conn, json.dumps({
        #     "code": codes.SPACE_PROOF,
        #     "proofs": w_proofs,
        # }))

        # no collisions -> return no collisions
        proofs = None
        #dt[i]['bandwidth_response'] = responder_payload
        if i == 1: 
            proofs= w_proofs
        else: 
            proofs= w_proofs_dict
        if len(colliding_coords_vector) == 0:
            tls.send(conn, json.dumps({
                "code": codes.SPACE_CAPS_INFO,
                "proofs": proofs,
                "end": True,
            }))
            #end = time.perf_counter()
            #space_exectuion += (end - start)*1000
            #dt[i]['time']=space_exectuion    
            return [], reindexing_initator(colliding_coords_vector,in_dict), False

            # TRUNCATED MODE end condition
        elif not FULL_MODE and (A_max_caps or B_max_caps):
            #colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
           
            tls.send(conn, json.dumps({
                "code": codes.SPACE_CAPS_INFO,
                "proofs": proofs,
                "end": True,
            }))
            #print(caps_r_dict)
            #end = time.perf_counter()
            #space_exectuion += (end - start)*1000
            #dt[i]['time']=space_exectuion    
            #return cart_to_gps(colliding_coords_vector), responder_payload
            return colliding_coords_vector,reindexing_initator(colliding_coords_vector,in_dict), (FULL_MODE and (A_max_caps and B_max_caps))

            # FULL MODE end condition
        elif FULL_MODE and A_max_caps and B_max_caps:
            #colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            tls.send(conn, json.dumps({
                "code": codes.SPACE_CAPS_INFO,
                "proofs": proofs,
                "end": True
            }))
            #print(in_dict)
            #end = time.perf_counter()
            #space_exectuion += (end - start)*1000
            #dt[i]['time']=space_exectuion    
            #return cart_to_gps(colliding_coords_vector), responder_payload
            return colliding_coords_vector,reindexing_initator(colliding_coords_vector,in_dict),  (FULL_MODE and A_max_caps and B_max_caps)

        # no end condition -> next iteration
        # here condition
        else:
            # remove duplicates
                colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            # split colliding coordinates sets
                trajectories_dict = {}
                temp =[]
                counter = 0 
                trajectories = []
                trajectories_dict_reindext ={}
                
                for sub_traj in colliding_coords_vector:
                #TODO check here for splitting
                    if len(sub_traj) > 2:
                        temp = util.split_in_sections(sub_traj, 2, 1)
                        trajectories += temp
                        trajectories_dict[counter]=[temp[0], temp[1]]
                        if len(in_dict)> 0 and in_dict.get(temp[0][0][2]) is not None:
                            index = in_dict[temp[0][0][2]][0]
                            traj_array = in_dict[temp[0][0][2]][1]
                            if (temp[0][0]== traj_array).all():
                                trajectories_dict_reindext[index]=[temp[0], temp[1]]
                        else:
                            #while trajectories_dict_reindext.get(counter) is not None:
                            #    counter+=1
                            
                            trajectories_dict_reindext[counter]=[temp[0], temp[1]] 

                    else:
                        trajectories.append(sub_traj)
                        trajectories_dict[counter]=sub_traj
                        if len(in_dict)> 0 and in_dict.get(sub_traj[0][2]) is not None:
                            index = in_dict[sub_traj[0][2]][0]
                            traj_array = in_dict[sub_traj[0][2]][1]
                            if (sub_traj[0]== traj_array).all():
                                trajectories_dict_reindext[index]=sub_traj
                    counter+=1
                #end = time.perf_counter()
                #space_exectuion += (end - start)*1000
                #dt[i]['time']=space_exectuion    
            # next iteration
                return space_matching_initiator_updated(conn=conn, trajectories=trajectories, n=n, i=i+1, wproofs_i= w_proofs, colliding_coords_vector = colliding_coords_vector, trajectories_dict=trajectories_dict_reindext, wproofdict = w_proofs_dict)


def space_matching_responder_updated(conn: SSLSocket, trajectories: np.array, response_vector=[], response_vector_table=[], capsule_to_coords=[], capsule_id_vector=[], n= int(), phi= int(), i=1, table=[[]], table_dict = {},response_vector_dict={},A_max_caps = False, B_max_caps = False, traj_dict={}, initiatior_payload = 0, indexing_prev ={}):
    """
    Runs the space matching protocol for the responder (B).
    """
    # stores mappings from capsule ID to coordinates
    #capsule_to_coords = []
    # stores colliding coordinates
    colliding_coords_vector = []
    # stores capsule id's
    #capsule_id_vector = []
    temp_traj_dict = {}
    # stores received challenges
    challenge_vector = []
    # stores send responses and hashed responses
    hashed_response_vector = []
    #res = []
    indexing_dict = {}
    indexing_dict_2 = {}
    indexing_dict_3 = {}
    indexing_dict_4 = indexing_prev
    space_exectuion = 0

    # receive capsule info from initiator
    #dt[i]={}
    #dt[i]['role']='responder'
    #dt[i]['domain']='space'
    #start = time.perf_counter()
    msg = tls.receive(conn)
    #initiatior_payload = sys.getsizeof(msg)
    #print(sys.getsizeof(msg))
    msg = json.loads(msg)
    
    #print(initiatior_payload)
    #dt[i]['bandwidth']=initiatior_payload
    code = msg["code"]
    # check message code
    if code != codes.SPACE_CAPS_INFO:
        tls.wrong_code_received(codes.SPACE_CAPS_INFO, code)

    # boolean indicating max capsules reached
    
    
    
    #Matrix=[True]
   
    #future_table = defaultdict(list) 
    #array2d = []


    if i > 1:
 
        array_vector= [[[] for x in range(2**(i-2))] for y in range(2**(i-2))]
        Matrix = [False] * 2**(i-1) 
    # check message code

        outer = 0
        inner = 0
   
        next_caps = False
        check = False
        count = 0
        #colliding_coords_vector_2 =[]
        colliding_vector_table = [[]]*2**(i-2)
        in_dict = {}
        if i-1> 1:
         ind =0
         #b_index = 0

         for key, values in msg["proofs"].items():
            
            for value in range(len(values)):
                
                key_int = int(key)
                ind = 2 * key_int + value
                #new_index = key_int+key_int+value
                indexing_dict[ind]=[]
                indexing_dict_2[ind]=[]
                indexing_dict_3[ind]=[]
                w_proof = msg["proofs"][key][str(value)][0]
                #print(value)
                next_caps = True
                count = 0
                #b_index = 0
                #for j in range(len(table_dict[key_int])):
                index_col=0 
                for o in range(len(table_dict[key_int][value])) :
                    
                    for k in range(len(table_dict[key_int][value][o])):
                        #print(len(table_dict[key_int][value]))
                        w_check = hashlib.sha256((str(response_vector_dict[key_int][value][o][k]) + str(table_dict[key_int][value][o][k])).encode('utf-8')).hexdigest()
                        if w_check == w_proof:
                            temp_col = capsule_to_coords[key_int].get(table_dict[key_int][value][o][k], {}).get(value,[])
                            if next_caps:
                                colliding_coords_vector += temp_col # capsule_to_coords[key_int].get(table_dict[key_int][value][o][k], [])
                                next_caps = False
                            
                            if B_max_caps:
                  
                                 
                                for key_internal in traj_dict.keys():
                                            traj_first = None
                                            traj_last = None
                                            traj_next = None
                                            traj_next_last = None
                                            two = False
                                            three = False
                                            after_three = False
                                            #before_two = False
                                            if isinstance(traj_dict.get(key_internal)[0][0],np.ndarray) :

                                                traj_first = traj_dict.get(key_internal)[0][0]
                                                traj_last =traj_dict.get(key_internal)[0][-1]
                                                traj_next = traj_dict.get(key_internal)[1][0]
                                                traj_next_last = traj_dict.get(key_internal)[1][-1]
                                                if ( key_internal -1 in traj_dict and len(traj_dict.get(key_internal-1)) == 2 and len(traj_dict.get(key_internal-1)[0]) == 3) or ( key_internal + 1 in traj_dict and len(traj_dict.get(key_internal+1)) == 2 and len(traj_dict.get(key_internal+1)[0]) == 3):
                                                    three = True

                                            elif isinstance(traj_dict.get(key_internal)[0][0],np.float64): #int 64
                                                traj_first = traj_dict.get(key_internal)[0]
                                                traj_last = traj_dict.get(key_internal)[-1]
                                                traj_next = traj_dict.get(key_internal)[1]
                                                traj_next_last = traj_last
                                                two = True
                                                if ( key_internal -1 in traj_dict and isinstance(traj_dict.get(key_internal-1)[0][0],np.ndarray)):
                                                    after_three = True

                                            for el in range(len(temp_col)):

                                                
                  
                            
                             
                                                if (traj_first == temp_col[el][0]).all() and (traj_last== temp_col[el][-1]).all():

                                                        if three:
                                                            if B_max_caps and A_max_caps:
                                                                in_dict[traj_first[2]] =[(2*key_internal+0),traj_first]
                                                                indexing_dict_2[ind].append(2*key_internal)
                                                            else:
                                                                in_dict[traj_first[2]] =[(2*key_internal+0),traj_first]
                                                                in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                                indexing_dict_2[ind].append(2*key_internal)
                                                            #if before_two:
                                                                indexing_dict_2[ind].append(2*key_internal+1)
                                                        elif two and after_three:
                                                            in_dict[traj_first[2]] =[(2*key_internal+1),traj_first] 
                                                            indexing_dict_2[ind].append(2*key_internal+1)

                                                        elif two:
                                                            in_dict[traj_first[2]] =[(2*key_internal+0),traj_first] 
                                                            indexing_dict_2[ind].append(2*key_internal)
                                                        else:
                                                            in_dict[traj_first[2]] =[(2*key_internal+0),traj_first]
                                                            indexing_dict_2[ind].append(2*key_internal+0)
                                                            in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                            indexing_dict_2[ind].append(2*key_internal+1)
                                              
                                                        
                                                        indexing_dict_2[ind] = list(set(indexing_dict_2[ind]))                                     
                                                        
                                                        #break
                                          
                                                if (traj_next == temp_col[el][0]).all() and (traj_next_last== temp_col[el][-1]).all():
                                   
                                                        if three:
                                                            if B_max_caps and A_max_caps:
                                                                in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                                indexing_dict_2[ind].append(2*key_internal+1)
                                                            else:    
                                                                in_dict[traj_first[2]] =[(2*key_internal+0),traj_first]
                                                                in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                                indexing_dict_2[ind].append(2*key_internal)
                                                                indexing_dict_2[ind].append(2*key_internal+1)
                                                        elif two:
                                                            in_dict[traj_first[2]] =[(2*key_internal+0),traj_first]  
                                                            indexing_dict_2[ind].append(2*key_internal)
                                                        else:
                                                            in_dict[traj_first[2]]=[(2*key_internal+0),traj_first]
                                                            indexing_dict_2[ind].append(2*key_internal+0)
                                                            in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                            indexing_dict_2[ind].append(2*key_internal+1)
                                                

                                                        
                                                        indexing_dict_2[ind] = list(set(indexing_dict_2[ind])) 
                                                        break
                                            
                                temp =[]
                                temp += capsule_to_coords[key_int].get(table_dict[key_int][value][o][k], {}).get(value,[])
                                indexing_dict[ind].extend(list(set(get_index(trajectories,temp)) - set(indexing_dict[ind])))

                            else:


                                        for key_internal in traj_dict.keys():
                                            traj_first = None
                                            traj_last = None
                                            traj_next = None
                                            traj_next_last = None
                                            if isinstance(traj_dict.get(key_internal)[0][0],np.ndarray) :

                                                traj_first = traj_dict.get(key_internal)[0][0]
                                                traj_last =traj_dict.get(key_internal)[0][-1]
                                                traj_next = traj_dict.get(key_internal)[1][0]
                                                traj_next_last = traj_dict.get(key_internal)[1][-1]
                                            elif isinstance(traj_dict.get(key_internal)[0][0],np.float64):#int64
                                                traj_first = traj_dict.get(key_internal)[0]
                                                traj_last = traj_dict.get(key_internal)[-1]
                                                traj_next = traj_dict.get(key_internal)[1]
                                                traj_next_last = traj_last

                                            for el in range(len(temp_col)):

                                                
                  
                            
                             
                                                if (traj_first == temp_col[el][0]).all() and (traj_last== temp_col[el][-1]).all():
                                                    
                                                        in_dict[traj_first[2]] =[(2*key_internal+0),traj_first]
                                                        indexing_dict_2[ind].append(2*key_internal+0)

                                                        
                                                    
                                                        in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                        indexing_dict_2[ind].append(2*key_internal+1)

                                                                                                    
                                                        count += 1
                                                        break
                                                    #in_dict[traj_dict[key_internal][1][0][2]].append(traj_dict[key_internal][1][0])
                                                elif (traj_next == temp_col[el][0]).all() and (traj_next_last== temp_col[el][-1]).all():
                                                        
                                                    #in_dict[traj_dict[key_internal][0][0][2]] = []
                                                        in_dict[traj_first[2]]=[(2*key_internal+0),traj_first]
                                                        indexing_dict_2[ind].append(2*key_internal+0)

                                                        in_dict[traj_next[2]]=[(2*key_internal+1),traj_next]
                                                        indexing_dict_2[ind].append(2*key_internal+1)

                                                        count += 1
                                                        break
                                                # else:
                                                #      indexing_dict_3[ind].append(2*key_int+o)
                                                #      break
                                                
                                        actual_index = len(table_dict)*2
                                        if len(table_dict)*2 < int(math.ceil(len(trajectories)/2)):
                                            actual_index = int(math.ceil(len(trajectories)/2))#len(trajectories) #len(trable_dict)*2
                                        if o <= actual_index:  actual_index = o
                                
                                        if len(indexing_dict[ind])> 1 and actual_index == indexing_dict[ind][-1]:
                                            actual_index += 1
                                        indexing_dict[ind].append(key_int+key_int+actual_index)
                                        indexing_dict_2[ind].append(2*key_int+o)
                            
                     
                            
                            index_col +=1
                            
                            break

                indexing_dict_2[ind] = list(set(indexing_dict_2[ind])) 
                ind += 1
                    
                

        if i-1 == 1:
         for caps_i in range(len(msg["proofs"])):
        # extract proof of proximity

            #print(colliding_vector_table)    
            outer = outer +1       
            #if table[caps_i]:
            w_proof = msg["proofs"][caps_i]
            if w_proof != "":
                next_caps = True
                #indexing_dict[caps_i]=[]
                indexing_dict_2[caps_i]=[]
                for j in range(len(table[caps_i])):
                    inner = inner +1
                    check = False
                # get capsule identifier
                    caps_ids = table[caps_i][j]
                    #print(len(caps_ids))
                    if len(caps_ids) !=0:
                      for k in range(len(caps_ids)):
                # generate w check
                        w_check = hashlib.sha256((str(response_vector_table[caps_i][j][k]) + str(table[caps_i][j][k])).encode('utf-8')).hexdigest()
                    #print(w_check)
                 # check collission between capsule identifiers
                        if w_check == w_proof:
                # get colliding coordinates
                            if next_caps:
                                colliding_coords_vector += capsule_to_coords[caps_i].get(table[caps_i][j][k], [])
                                colliding_vector_table[caps_i]=capsule_to_coords[caps_i].get(table[caps_i][j][k], [])
                                next_caps = False
                # only one match possible so break from loop

                            array_vector[caps_i][j].append(capsule_to_coords[caps_i].get(table[caps_i][j][k], []))

                            indexing_dict_2[caps_i].append(j)

                            check = True

                            break
                     
                        
                        Matrix[caps_i+caps_i]=False
                        Matrix[caps_i+caps_i+1] = False
  
                    if not check:
                        #future_table[caps_i].append(0)
                        colliding_vector_table[caps_i].append([])
                    



        if len(colliding_coords_vector) == 0:
            # tls.send(conn, json.dumps({
            #     "code": codes.STOP_ITERATION,
            #     "max_caps_B": B_max_caps
            # }))
            #end = time.perf_counter()
            #space_exectuion += (end - start)*1000
            #dt[i]['time']=space_exectuion   
            return [], reindexing_responder(colliding_coords_vector,in_dict), indexing_dict_2, FULL_MODE

        # TRUNCATED MODE end condition
        elif not FULL_MODE and (A_max_caps or B_max_caps):
           
            # tls.send(conn, json.dumps({
            #     "code": codes.STOP_ITERATION,
            #     "max_caps_B": B_max_caps
            # }))
            #colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            #return cart_to_gps(colliding_coords_vector), initiatior_payload
            #end = time.perf_counter()
            #space_exectuion += (end - start)*1000
            #dt[i]['time']=space_exectuion   
            return colliding_coords_vector, reindexing_responder(colliding_coords_vector,in_dict), indexing_dict_2, FULL_MODE

        # FULL MODE end condition
        elif FULL_MODE and A_max_caps and B_max_caps:
            # if not msg["end"]:
            #     tls.send(conn, json.dumps({
            #         "code": codes.STOP_ITERATION,
            #         "max_caps_B": B_max_caps
            #     }))
            #colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            #return cart_to_gps(colliding_coords_vector), initiatior_payload
            #print(in_dict)
            #print(indexing_dict_2)
            #end = time.perf_counter()
            #space_exectuion += (end - start)*1000
            #dt[i]['time']=space_exectuion   
            return colliding_coords_vector, reindexing_responder(colliding_coords_vector,in_dict), indexing_dict_2, FULL_MODE
            #else: 
            #return cartesian_to_gps(colliding_coords_vector)

        # no end condition -> next iteration
        else:
        # (18) TAKE THE CORRESPONDING SET OF COLLIDING COORDINATES
        #      - FOR EACH, EXECUTE (8), (9), (10) AND (11)
        # remove duplicates
            colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
        # split colliding coordinates sets
            trajectories = []
            trajectories_dict = {}
            trajectories_dict_reindext ={}
            temp =[]
            counter = 0 
            for sub_traj in colliding_coords_vector:
                if len(sub_traj) > 2:
                    temp = util.split_in_sections(sub_traj, 2, 1)
                    trajectories += temp
                    trajectories_dict[counter]=[temp[0], temp[1]]
                    mapped = temp[0][0][2]#map_to_string(temp[0][0])#temp[0][0][2]
                    if len(in_dict)> 0 and in_dict.get(mapped) is not None:
                        index = in_dict[mapped][0]
                        traj_array = in_dict[mapped][1]
                        if (temp[0][0]== traj_array).all():
                            trajectories_dict_reindext[index]=[temp[0], temp[1]]
                    else:
                        while trajectories_dict_reindext.get(counter) is not None:
                            counter+=1
                            
                        
                        trajectories_dict_reindext[counter]=[temp[0], temp[1]] 
                else:
                    trajectories.append(sub_traj)
                    trajectories_dict[counter]=sub_traj

                    if len(in_dict)> 0 and in_dict.get(sub_traj[0][2]) is not None:
                        index = in_dict[sub_traj[0][2]][0]
                        traj_array = in_dict[sub_traj[0][2]][1]
                        if (sub_traj[0]== traj_array).all():
                            trajectories_dict_reindext[index]=sub_traj
                counter+=1
            temp_traj_dict = trajectories_dict_reindext
      
    
    capsule_id_vector = []
    response_vector = []
    response_vector_table = []
    hashed_response_vector_table = []
    response_vector_dict = {}
    hashed_response_vector_dict = {}
    capsule_to_coords = []

    A_max_caps = msg["max_caps_A"]
    B_max_caps = max(map(len, trajectories)) == 2
    # for each A-capsule
    outer_new =0
    inner_new =0
 
    list_of_ids = []
    list_of_ids_dict = {}
     
 
    check = False


    if i > 1:
     capsule_to_coords ={}
     #print("round:")
     #print(i)
     #dt[i]['indexing']=indexing_dict_2
     #dt[i]['capsulepoints']={}
     for key, values in indexing_dict_2.items():
        #capsule_to_coords[key]=[]
        #if key <= int(list(msg["challenges"].keys())[-1]):
            response_vector_dict[key] ={}
            hashed_response_vector_dict[key]={}
            list_of_ids_dict[key]={}
            
            capsule_to_coords_sub = {}
            capsule_to_coords_sub_test = {}
            list_of_ids_dict[key][0]=[]
            response_vector_dict[key][0]=[]
            hashed_response_vector_dict[key][0]=[]
            list_of_ids_dict[key][1]=[]
            response_vector_dict[key][1]=[]
            hashed_response_vector_dict[key][1]=[]
            for value in values:
                key_string =str(key)
                if value in trajectories_dict_reindext and key_string in msg["challenges"]:
                    
                    #print(len(msg["challenges"][key_string]))
                    for k in range(len(msg["challenges"][key_string])):
                        encr_rsps = []
                        responses = []
                        already_exists_dict = {}
                        capsule_to_coords_sub_sub = {}


                

                        encr_challenge = msg["challenges"][key_string][str(k)]
                        challenge_vector.append(encr_challenge)
                        caps_origin_x, caps_origin_y = msg["origins"][key_string][str(k)]
                        caps_angle = msg["angles"][key_string][str(k)]
                        caps_r = msg["radiuss"][key_string][str(k)]
                        caps_h = msg["lengths"][key_string][str(k)]

                        #list_ids = []
                        for l in range(len(trajectories_dict_reindext[value])):

                            capsule_ids = []
                            response_list=[]
                            encr_list = []
                            trajectory = None
                            traj_temp = None
                            #np.int64
                            if isinstance(trajectories_dict_reindext[value][0][0],np.float64) :
                                trajectory = np.delete(trajectories_dict_reindext[value], 2, 1)
                                traj_temp = trajectories_dict_reindext[value]
                            elif isinstance(trajectories_dict_reindext[value][0][0],np.ndarray):
                                trajectory = np.delete(trajectories_dict_reindext[value][l], 2, 1)
                                traj_temp = trajectories_dict_reindext[value][l]
                
                            #print("trajectory points distance:",{sd.distance(trajectory[0],trajectory[-1])})
                        # (8) MAP TRAJECTORY INTO SPACE TESSELLATION INSTRUCTED BY A
                    # generating a rototranslation of the points in trajectory
                            shifted_trajectory = []
                            shifted_trajectory_new = []
                            #dt[i]['capsulepoints'][value+l]=len(trajectory)
                            for x, y in trajectory:
                                #print("B traj:")
                                #print([x,y])
                                #x = float("{:.3f}".format(x))
                                #y = float("{:.3f}".format(y))
                                #new_x = x * float("{:.3f}".format(math.cos(-caps_angle) ))- y * float("{:.3f}".format(math.sin(-caps_angle)))
                                #new_y = x * float("{:.3f}".format(math.sin(-caps_angle))) + y * float("{:.3f}".format(math.cos(-caps_angle)))
                   
                                new_x = (x * math.cos(-caps_angle) - y * math.sin(-caps_angle)) 
                                new_y = (x * math.sin(-caps_angle) + y * math.cos(-caps_angle)) 
                                #new_x = x
                                #new_y = y
                                #print([new_x,new_y])
                                new_x = new_x - caps_origin_x
                                new_y = new_y - caps_origin_y
                                shifted_trajectory.append([new_x, new_y, x, y])
                                shifted_trajectory_new.append([new_x, new_y])

                                #print([new_x,new_y])
                                #print(caps_angle)
                                #print("B traj end:")
                            #print("shifted points distance:",{sd.distance(shifted_trajectory_new[0],shifted_trajectory_new[-1])})
                    # get B capsule
                            h, r, angle,angle_np = sd.compute_ref_capsule(
                                np.array(shifted_trajectory)[:, 0:2],
                                COLLISION_THRESHOLD,
                                MAX_GPS_DEVIATION
                            )
                        # identify capsule identifiers
                            capsule_ids_traj = list()
                            #if B_max_caps:
                            #    capsule_ids_traj = util.get_colliding_caps_ids_last(
                            #np.array(shifted_trajectory)[:, 0:2], r, angle, caps_r, caps_h
                            #)
                            #else:
                            #capsule_ids_traj = util.get_colliding_caps_ids(
                            #np.array(shifted_trajectory)[:, 0:2], r, angle, caps_r, caps_h
                            #)

                            capsule_ids_traj = util.get_colliding_caps_ids_new(
                            np.array(shifted_trajectory)[:, 0:2], h, r, angle, caps_r, caps_h
                            )
                        # store capsule id's
                            capsule_ids += capsule_ids_traj
                        #list_of_ids_dict[key].append(capsule_ids_traj)

                            for caps_id in capsule_ids_traj:
                                current_mapping_test = capsule_to_coords_sub_test.get(caps_id,{})
                                current_mapping_test_sub = current_mapping_test.get(k,[])
                                current_mapping_test_sub.append(np.array([[x, y, t] for x, y, t in traj_temp]))
                                current_mapping_test.update({k:current_mapping_test_sub})
                                capsule_to_coords_sub_test.update({caps_id:current_mapping_test})

                                current_mapping = capsule_to_coords_sub.get(caps_id, [])
                                current_mapping.append(np.array([[x, y, t] for x, y, t in traj_temp]))
                                capsule_to_coords_sub.update({caps_id: current_mapping})
                                if caps_id not in already_exists_dict:
                                    already_exists_dict.update({caps_id: ['False']})
                        
                                else:
                                    already_exists_dict[caps_id][0]= 'True'

                        # (9) FOR EACH CAPSULE THAT THIS TRAJECTORY COLLIDES WITH, COMPUTE PARAMETER E
                            e = []
                            for m in range(len(capsule_ids_traj)):
                                if already_exists_dict[capsule_ids_traj[m]][0] == 'False':
                                    d = capsule_ids_traj[m]
                        #e.append(pow(d, -1, phi))
                                    e = pow(d, -1, phi)

                    # (10) FOR EACH CAPSULE THAT THIS TRAJECTORY COLLIDES WITH, COMPUTE ENCRYPTED RESPONSE
                    #for k in range(len(capsule_ids_traj)):
                    #if not already_exists_dict[capsule_ids_traj[k]]:
                                    c = encr_challenge
                                    response_number = pow(c, e, n)
                                    responses.append(response_number)
                                    hash = hashlib.sha256(str(response_number).encode('utf-8')).hexdigest()
                                    encr_rsps.append(hash)
                                    response_list.append(response_number)
                                    encr_list.append(hash)
                                    already_exists_dict[capsule_ids_traj[m]].extend([response_number, hash])
                                else:
                                    responses.append(already_exists_dict[capsule_ids_traj[m]][1])
                                    encr_rsps.append(already_exists_dict[capsule_ids_traj[m]][2])
                                    response_list.append(already_exists_dict[capsule_ids_traj[m]][1])
                                    encr_list.append(already_exists_dict[capsule_ids_traj[m]][2])
                        # print(encr_rsps)
                        # [res.append(x) for x in encr_rsps if x not in res]
                        # encr_rsps = res
                            list_of_ids_dict[key][k].append(capsule_ids)
                            response_vector_dict[key][k].append(response_list)
                            hashed_response_vector_dict[key][k].append(encr_list)
                    
                # store responses
                        response_vector.append(responses)
                # store hashed responses
                        hashed_response_vector.append(encr_rsps)
                # store capsule ids
                        capsule_id_vector.append(capsule_ids)
            # store capsule to coordinates mapping
                #capsule_to_coords[key]=capsule_to_coords_sub
                capsule_to_coords[key]=capsule_to_coords_sub_test


    else:
     for caps_i in range(len(msg["challenges"])):
        # sub-trajectory storage
            outer_new = outer_new + 1
        
            encr_rsps = []
            responses = []
            capsule_ids = []
            capsule_to_coords_sub = {}
            already_exists_dict = {}

        # extract capsule info


   
        # for each B-capsulelen(trajectories)
            
            response_vector_table.append([])
            hashed_response_vector_table.append([])
            list_of_ids.append([])
            for traj_i in range(len(trajectories)):
              #if (i > 1 and array2d[math.trunc(caps_i/2)][math.trunc(traj_i/B_divisor)] == 1) or i == 1:
                encr_challenge = msg["challenges"][caps_i]
                challenge_vector.append(encr_challenge)
                caps_origin_x, caps_origin_y = msg["origins"][caps_i]
                caps_angle = msg["angles"][caps_i]
                caps_r = msg["radiuss"][caps_i]
                caps_h = msg["lengths"][caps_i]

                response_list=[]
                encr_list = []

                inner_new = inner_new + 1
                
                trajectory = np.delete(trajectories[traj_i], 2, 1)
                #print("trajectory points distance:",{sd.distance(trajectory[0],trajectory[-1])})
                # (8) MAP TRAJECTORY INTO SPACE TESSELLATION INSTRUCTED BY A
                # generating a rototranslation of the points in trajectory
                shifted_trajectory = []
                shifted_trajectory_new = []
                for x, y in trajectory:
                    #x = float("{:.2f}".format(x))
                    #y = float("{:.2f}".format(y))
                    #new_x = x * float("{:.3f}".format(math.cos(-caps_angle) ))- y * float("{:.3f}".format(math.sin(-caps_angle))) - caps_origin_x
                    #new_y = x * float("{:.3f}".format(math.sin(-caps_angle))) + y * float("{:.3f}".format(math.cos(-caps_angle))) - caps_origin_y
                    
                    new_x = (x * math.cos(-caps_angle) - y * math.sin(-caps_angle)) - caps_origin_x
                    new_y = (x * math.sin(-caps_angle) + y * math.cos(-caps_angle)) - caps_origin_y
                    #new_x = x - caps_origin_x
                    #new_y = y - caps_origin_y
                    
                    #new_x = x * np.round(math.cos(-caps_angle),4)- y * np.round(math.sin(-caps_angle),4)
                    #new_y = x * np.round(math.sin(-caps_angle),4) + y * np.round(math.cos(-caps_angle),4)
                    shifted_trajectory.append([new_x, new_y, x, y])
                    shifted_trajectory_new.append([new_x, new_y])
                    #print("B traj:")
                    #print([x,y])
                    #print([new_x,new_y])
                    #print("B traj end:")
                #print("shifted points distance:",{sd.distance(shifted_trajectory_new[0],shifted_trajectory_new[-1])})
                # get B capsule
                h, r, angle, angle_np = sd.compute_ref_capsule(
                    np.array(shifted_trajectory)[:, 0:2],
                    COLLISION_THRESHOLD,
                    MAX_GPS_DEVIATION
                )
                # identify capsule identifiers
                #capsule_ids_traj = util.get_colliding_caps_ids(
                #np.array(shifted_trajectory)[:, 0:2], r, angle, caps_r, caps_h
                #)
                capsule_ids_traj = util.get_colliding_caps_ids_new(
                np.array(shifted_trajectory)[:, 0:2], h, r, angle, caps_r, caps_h
                )

                # store capsule id's
                capsule_ids += capsule_ids_traj

                list_of_ids[caps_i].append(capsule_ids_traj)

                for caps_id in capsule_ids_traj:
                    current_mapping = capsule_to_coords_sub.get(caps_id, [])
                    current_mapping.append(np.array([[x, y, t] for x, y, t in trajectories[traj_i]]))
                    capsule_to_coords_sub.update({caps_id: current_mapping})
                    if caps_id not in already_exists_dict:
                        already_exists_dict.update({caps_id: ['False']})
                        
                    else:
                        already_exists_dict[caps_id][0]= 'True'

                # (9) FOR EACH CAPSULE THAT THIS TRAJECTORY COLLIDES WITH, COMPUTE PARAMETER E
                e = []
                for k in range(len(capsule_ids_traj)):
                 if already_exists_dict[capsule_ids_traj[k]][0] == 'False':
                    d = capsule_ids_traj[k]
                    
                    e = pow(d, -1, phi)

                # (10) FOR EACH CAPSULE THAT THIS TRAJECTORY COLLIDES WITH, COMPUTE ENCRYPTED RESPONSE

                    c = encr_challenge
                    response_number = pow(c, e, n)
                    responses.append(response_number)
                    hash = hashlib.sha256(str(response_number).encode('utf-8')).hexdigest()
                    encr_rsps.append(hash)
                    response_list.append(response_number)
                    encr_list.append(hash)
                    already_exists_dict[capsule_ids_traj[k]].extend([response_number, hash])
                 else:
                     responses.append(already_exists_dict[capsule_ids_traj[k]][1])
                     encr_rsps.append(already_exists_dict[capsule_ids_traj[k]][2])
                     response_list.append(already_exists_dict[capsule_ids_traj[k]][1])
                     encr_list.append(already_exists_dict[capsule_ids_traj[k]][2])

                response_vector_table[caps_i].append(response_list)
                hashed_response_vector_table[caps_i].append(encr_list)
            # store responses
            response_vector.append(responses)
            # store hashed responses
            hashed_response_vector.append(encr_rsps)
            # store capsule ids
            capsule_id_vector.append(capsule_ids)
            # store capsule to coordinates mapping
            capsule_to_coords.append(capsule_to_coords_sub)
            

        # (11) SEND: VECTOR WITH ENCRYPTED RESPONSES
 
    
    if i==1:
        tls.send(conn, json.dumps({
            "code": codes.SPACE_ENCR_RESPONSES,
            "responses2": hashed_response_vector_table,
            #"responses": hashed_response_vector,
            "max_caps_B": B_max_caps
        }))
    else:
        tls.send(conn, json.dumps({
            "code": codes.SPACE_ENCR_RESPONSES,
            "responses2": hashed_response_vector_dict,
            #"responses": hashed_response_vector,
            "max_caps_B": B_max_caps
        }))
    #print(temp_traj_dict)

    #end = time.perf_counter()
    #space_exectuion += (end - start)*1000
    #dt[i]['time']=space_exectuion   
       

    return space_matching_responder_updated(conn=conn, trajectories=trajectories, response_vector=response_vector, response_vector_table= response_vector_table ,capsule_id_vector=capsule_id_vector, capsule_to_coords=capsule_to_coords, n=n, phi=phi, i=i+1, table = list_of_ids,table_dict= list_of_ids_dict, response_vector_dict= response_vector_dict,A_max_caps = A_max_caps, B_max_caps = B_max_caps, traj_dict = temp_traj_dict, indexing_prev=indexing_dict_2)

def get_index(a,b):
    index_list = []
    for el in b:
        index = 0
        for n in a:
            if el[0][2]== n [0][2]:
                index_list.append(index)
            index += 1
    return index_list

def map_to_string(lst):
    id_str = ''.join(map(str, lst))
    return id_str

def reindexing_responder(colliding_coords_vector,in_dict):
            if len(colliding_coords_vector)==0 or len(in_dict)==0:
                return {}
            colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
        # split colliding coordinates sets
            trajectories = []
            trajectories_dict = {}
            trajectories_dict_reindext ={}
            temp =[]
            counter = 0 
            for sub_traj in colliding_coords_vector:
                if len(sub_traj) > 2:
                    temp = util.split_in_sections(sub_traj, 2, 1)
                    trajectories += temp
                    trajectories_dict[counter]=[temp[0], temp[1]]
                    mapped = temp[0][0][2]#map_to_string(temp[0][0])#temp[0][0][2]
                    if len(in_dict)> 0 and in_dict.get(mapped) is not None:
                        index = in_dict[mapped][0]
                        traj_array = in_dict[mapped][1]
                        if (temp[0][0]== traj_array).all():
                            trajectories_dict_reindext[index]=[temp[0], temp[1]]
                    else:
                        while trajectories_dict_reindext.get(counter) is not None:
                            counter+=1
                            
                        
                        trajectories_dict_reindext[counter]=[temp[0], temp[1]] 
                else:
                    trajectories.append(sub_traj)
                    trajectories_dict[counter]=sub_traj

                    if len(in_dict)> 0 and in_dict.get(sub_traj[0][2]) is not None:
                        index = in_dict[sub_traj[0][2]][0]
                        traj_array = in_dict[sub_traj[0][2]][1]
                        if (sub_traj[0]== traj_array).all():
                            trajectories_dict_reindext[index]=sub_traj
                counter+=1
            return trajectories_dict_reindext

def reindexing_initator(colliding_coords_vector,in_dict):

    colliding_coords_vector = util.remove_duplicates(colliding_coords_vector)
            # split colliding coordinates sets
    trajectories_dict = {}
    temp =[]
    counter = 0 
    trajectories = []
    trajectories_dict_reindext ={}
                
    for sub_traj in colliding_coords_vector:
                #TODO check here for splitting
        if len(sub_traj) > 2:
            temp = util.split_in_sections(sub_traj, 2, 1)
            trajectories += temp
            trajectories_dict[counter]=[temp[0], temp[1]]
            if len(in_dict)> 0 and in_dict.get(temp[0][0][2]) is not None:
                index = in_dict[temp[0][0][2]][0]
                traj_array = in_dict[temp[0][0][2]][1]
                if (temp[0][0]== traj_array).all():
                                trajectories_dict_reindext[index]=[temp[0], temp[1]]
            else:
                            #while trajectories_dict_reindext.get(counter) is not None:
                            #    counter+=1
                            
                trajectories_dict_reindext[counter]=[temp[0], temp[1]] 

        else:
            trajectories.append(sub_traj)
            trajectories_dict[counter]=sub_traj
            if len(in_dict)> 0 and in_dict.get(sub_traj[0][2]) is not None:
                index = in_dict[sub_traj[0][2]][0]
                traj_array = in_dict[sub_traj[0][2]][1]
                if (sub_traj[0]== traj_array).all():
                    trajectories_dict_reindext[index]=sub_traj
    return trajectories_dict_reindext


                    
