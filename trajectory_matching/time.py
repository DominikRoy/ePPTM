import hashlib
import json
import random
import sys
import network.codes as codes
import network.tls as tls
import trajectory_matching.util as util
import time
import numpy as np

GUARD_TIME = 1

def time_matching_initiator(conn, collisions, reindexed_collisions ,n,mode):
    """
    Runs the time matching protocol for the initiator (A).
    """
    # stores colliding coordinates
    colliding_coords_vector = []
    colliding_coords_dict = {}
    # stores capsule id's
    capsule_id_vector = []
    capsule_id_dict = {}
    # stores nonces
    nonce_vector = []
    nonce_dict = {}

    # stores send challenges
    challenge_vector = []
    challenge_dict = {}

    # capsule storage before sending
    origin_vector = []
    origin_dict = {}
    time_division_vector = []
    time_division_dict  = {}
    # proof storage before sending
    proof_vector = []
    proof_dict= {}
    #dt={}
    #dt['role']='initiator'
    #dt['domain']='time'
    #time_exectuion = 0
    #start = time.perf_counter()
    for key in reindexed_collisions:
        time_division_dict[key] = []
        origin_dict[key] = []
        capsule_id_dict[key] = []
        challenge_dict[key] = []
        nonce_dict[key] = []
        collision_points = reindexed_collisions[key]
        t_start,t_end=0,0
        if mode:
            t_start, t_end = collision_points[0][2], collision_points[-1][2]
        else:
            if isinstance(collision_points[0][0],np.ndarray):
                t_start, t_end = collision_points[0][0][2], collision_points[1][-1][2]
            elif isinstance(collision_points[0][0],np.float64):
                t_start, t_end = collision_points[0][2], collision_points[-1][2]
        # compute time division
        time_division = int(t_end) - int(t_start) + GUARD_TIME
        time_division_dict[key].append(time_division)
        r = random.getrandbits(32)
        origin_timestamp = int(t_start) - (time_division * r) #random.randint(1, 2500)
        #r = random.getrandbits(32)
        #rigin_timestamp_end = int(t_end) - (time_division * r)
        origin_dict[key].append(origin_timestamp)
        #origin_dict[key].append(origin_timestamp_end)

                # shift timestamps
        shifted_collision = []
        #for x, y, t in collision_points:
            #for origin_timestamp in origin_dict[key]:
        time_division_dict[key].append(time_division)
                #r = random.getrandbits(32)
                #origin_timestamp = int(t) - (time_division * r) #random.randint(1, 2500)
                #origin_dict[key].append(origin_timestamp)
            #origin_timestamp = int(t_start) + time_division * random.randint(1, 2500)
            #origin_dict[key].append(origin_timestamp)
        # compute random origin timestamp
           
        shifted = int(t_start) - origin_timestamp 
        shifted_collision.append(shifted)
            #shifted_collision.append(int(t) - origin_timestamp)
        # get capsule id
        id = util.get_hashed_time_capsule_id(util.get_time_capsule_id_floor(shifted, time_division))
            #id = util.get_hashed_time_capsule_id(util.get_time_capsule_id(shifted, time_division))
        capsule_id_dict[key].append(id)
        # set d
        d = id
        # generate nonce
        nonce = random.randint(1, n)
        nonce_dict[key].append(nonce)
        # compute c
        c = pow(nonce, d, n)
        challenge_dict[key].append(c)

        time_division_dict[key].append(time_division)
        shifted = int(t_end) - origin_timestamp 
        shifted_collision.append(shifted)
        id = util.get_hashed_time_capsule_id(util.get_time_capsule_id(shifted, time_division))
            #id = util.get_hashed_time_capsule_id(util.get_time_capsule_id(shifted, time_division))
        capsule_id_dict[key].append(id)
        # set d
        d = id
        # generate nonce
        nonce = random.randint(1, n)
        nonce_dict[key].append(nonce)
        # compute c
        c = pow(nonce, d, n)
        challenge_dict[key].append(c)
    # send capsule info to responder
    tls.send(conn, json.dumps({
        "code": codes.TIME_CAPS_INFO,
        "challenges": challenge_dict,
        "origins": origin_dict,
        "time_divisions": time_division_dict
    }))

    # receive responses from responder
    msg = json.loads(tls.receive(conn))
    #responder_payload = sys.getsizeof(msg)
    code = msg["code"]
    # check message code
    if code != codes.TIME_ENCR_RESPONSES:
        tls.wrong_code_received(codes.TIME_ENCR_RESPONSES, code)

    # check for collisions
    for key in reindexed_collisions:
        key_string = str(key)
        encr_responses = msg["responses"][key_string]
        proof_dict[key] = []
        for i in range(len(nonce_dict[key])):   
            nonce = nonce_dict[key][i]
        
        #for i in range(len(reindexed_collisions[key])):
             # compute w
            w = hashlib.sha256(str(nonce % n).encode('utf-8')).hexdigest()
            for encr_rsps in encr_responses:
                if encr_rsps == w:
                # get colliding coordinates
                    colliding_coords_vector.append(reindexed_collisions[key])
                    break
        # compute proofs of proximity
            w_proof = hashlib.sha256(
            (str(nonce % n) + str(capsule_id_dict[key][i])).encode('utf-8')
            ).hexdigest()
            proof_dict[key].append(w_proof)
    # send proofs
    tls.send(conn, json.dumps({
        "code": codes.TIME_PROOF,
        "proofs": proof_dict
    }))  

    #end = time.perf_counter()
    #time_exectuion += (end - start)*1000
    #dt['time']=time_exectuion
    #dt['bandwidth']=responder_payload
    return colliding_coords_vector
    # for col_i in range(len(collisions)):
    #     collision_points = collisions[col_i]
    #     t_start, t_end = collision_points[0][2], collision_points[-1][2]

    #     # compute time division
    #     time_division = int(t_end) - int(t_start) + GUARD_TIME
    #     time_division_vector.append(time_division)
    #     # compute random origin timestamp
    #     origin_timestamp = int(t_start) + time_division * random.randint(1, 2500)
    #     origin_vector.append(origin_timestamp)
    #     # shift timestamps
    #     shifted_collision = []
    #     for x, y, t in collision_points:
    #         shifted_collision.append(origin_timestamp - int(t))
    #     # get capsule id
    #     id = util.get_hashed_time_capsule_id(util.get_time_capsule_id(shifted_collision[0], time_division))
    #     capsule_id_vector.append(id)
    #     # set d
    #     d = id
    #     # generate nonce
    #     nonce = random.randint(1, n)
    #     nonce_vector.append(nonce)
    #     # compute c
    #     c = pow(nonce, d, n)
    #     challenge_vector.append(c)

    # # send capsule info to responder
    # tls.send(conn, json.dumps({
    #     "code": codes.TIME_CAPS_INFO,
    #     "challenges": challenge_vector,
    #     "origins": origin_vector,
    #     "time_divisions": time_division_vector
    # }))

    # # receive responses from responder
    # msg = json.loads(tls.receive(conn))
    # responder_payload = sys.getsizeof(msg)
    # code = msg["code"]
    # # check message code
    # if code != codes.TIME_ENCR_RESPONSES:
    #     tls.wrong_code_received(codes.TIME_ENCR_RESPONSES, code)

    # # check for collisions
    # for col_i in range(len(collisions)):
    #     encr_responses = msg["responses"][col_i]
    #     nonce = nonce_vector[col_i]
    #     # compute w
    #     w = hashlib.sha256(str(nonce % n).encode('utf-8')).hexdigest()
    #     # check collission between capsule identifiers
    #     for encr_rsps in encr_responses:
    #         if encr_rsps == w:
    #             # get colliding coordinates
    #             colliding_coords_vector.append(collisions[col_i])
    #             break
    #     # compute proofs of proximity
    #     w_proof = hashlib.sha256(
    #         (str(nonce % n) + str(capsule_id_vector[col_i])).encode('utf-8')
    #     ).hexdigest()
    #     proof_vector.append(w_proof)
    # # send proofs
    # tls.send(conn, json.dumps({
    #     "code": codes.TIME_PROOF,
    #     "proofs": proof_vector
    # }))

    # return colliding_coords_vector, responder_payload


def time_matching_responder(conn, collisions, reindexed_collisions, indexing,n, phi,mode):
    # stores colliding coordinates
    colliding_coords_vector = []
    colliding_coords_dict = {}
    # stores capsule id's
    capsule_ids_vector = []
    capsule_ids_dict = {}

    # stores received challenges
    challenge_vector = []
    challenge_dict = {}
    # stores send responses and hashed responses
    response_vector, hashed_response_vector = [], []

    response_dict = {}
    response_dict_2 = {}
    hashed_response_dict = {}
    #dt = {}
    #time_exectuion=0
    #dt['role']='responder'
    #dt['domain']='time'
    #start = time.perf_counter()
    # receive time info from initiator
    msg = json.loads(tls.receive(conn))
    #initiatior_payload = sys.getsizeof(msg)
    code = msg["code"]
    # check message code
    if code != codes.TIME_CAPS_INFO:
        tls.wrong_code_received(codes.TIME_CAPS_INFO, code)
    for key in indexing:
        challenge_dict[key] = {}
        capsule_ids_dict[key] = {}
        response_dict[key] = {}
        response_dict_2[key] = []
        hashed_response_dict[key]={}
        for value in indexing[key]:
            if value in reindexed_collisions:
                response_dict[key][value]=[]
                hashed_response_dict[key][value]=[]
                #capsule_ids_dict[key] [value] =[] 
                collision_points = reindexed_collisions [value]
            # iteration storage
                encr_rsps = []
                responses = []

        # extract timestamp info
                key_string = str(key)
                for i in range(len(msg["challenges"][key_string])):
                    encr_rsps = []
                    responses = []
                    encr_challenge = msg["challenges"][key_string][i]
                    challenge_dict[key][value]=(encr_challenge)
                    origins = msg["origins"][key_string]
                    time_division = msg["time_divisions"][key_string][i]

        # rescale timestamps
                    rescaled_timestamps = []
                    #for x, y, t in collision_points:
                    #    for origin in origins:
                    if mode:
                        rescaled_timestamps.append(int(collision_points[0][2]) - origins[0])

        # get all colliding A-capsules
                        timestamp_ids = util.get_hashed_time_capsule_ids_reversed_floor(rescaled_timestamps, time_division)
                        rescaled_timestamps = []
                        rescaled_timestamps.append(int(collision_points[-1][2]) - origins[0])
                        timestamp_ids += util.get_hashed_time_capsule_ids_reversed(rescaled_timestamps, time_division)
                        capsule_ids_dict[key][value]=(timestamp_ids)
                    else:
                        if isinstance(collision_points[0][0],np.ndarray):
                           
                            rescaled_timestamps.append(int(collision_points[0][0][2]) - origins[0])

        # get all colliding A-capsules
                            timestamp_ids = util.get_hashed_time_capsule_ids_reversed_floor(rescaled_timestamps, time_division)
                            rescaled_timestamps = []
                            rescaled_timestamps.append(int(collision_points[1][-1][2]) - origins[0])
                            timestamp_ids += util.get_hashed_time_capsule_ids_reversed(rescaled_timestamps, time_division)
                            capsule_ids_dict[key][value]=(timestamp_ids)
                        elif isinstance(collision_points[0][0],np.float64):
                            rescaled_timestamps.append(int(collision_points[0][2]) - origins[0])

        # get all colliding A-capsules
                            timestamp_ids = util.get_hashed_time_capsule_ids_reversed_floor(rescaled_timestamps, time_division)
                            rescaled_timestamps = []
                            rescaled_timestamps.append(int(collision_points[-1][2]) - origins[0])
                            timestamp_ids += util.get_hashed_time_capsule_ids_reversed(rescaled_timestamps, time_division)
                            capsule_ids_dict[key][value]=(timestamp_ids)
                                        
                    

        # compute e
                    e = []
                    for k in range(len(timestamp_ids)):
                        e.append(pow(timestamp_ids[k], -1, phi))

        # compute responses
                    for k in range(len(timestamp_ids)):
                        c = encr_challenge
                        response_number = pow(c, e[k], n)
                        responses.append(response_number)
                        encr_rsps.append(hashlib.sha256(str(response_number).encode('utf-8')).hexdigest())

            # store responses before sending
                    response_dict[key][value].append(responses)
                    response_dict_2[key].extend(encr_rsps)
                    hashed_response_dict[key][value].append(encr_rsps)



                # send responses to initiator
    tls.send(conn, json.dumps({
        "code": codes.TIME_ENCR_RESPONSES,
        "responses": response_dict_2
    }))

     # receive time proofs from initiator
    msg = json.loads(tls.receive(conn))
    #initiatior_payload += sys.getsizeof(msg)
    code = msg["code"]
    # check message code
    if code != codes.TIME_PROOF:
        tls.wrong_code_received(codes.TIME_PROOF, code)

    # check for collisions
    for key in indexing:
     if len(indexing[key]) != 0:
        key_string = str(key)

        for i in range(len(msg["proofs"][key_string])):
         
            w_proof = msg["proofs"][key_string][i]
        
            for value in indexing[key]: # change lloop
             if value in response_dict[key]:
                for j in range(len(response_dict[key][value])):
                    #caps_id = capsule_ids_dict[key][value][j]
                    for k in range(len(response_dict[key][value][j])):
                        caps_id = capsule_ids_dict[key][value][k]
                        w_check = hashlib.sha256(
                        (str(response_dict[key][value][j][k]) + str(caps_id)).encode('utf-8')
                        ).hexdigest()
            # check collission between capsule identifiers
               
                        if w_check == w_proof:
                # get colliding coordinates
                            colliding_coords_vector.append(reindexed_collisions[value])
                # only one match possible so break from loop
                            break
    #end = time.perf_counter()
    #time_exectuion += (end - start)*1000
    #dt['time']=time_exectuion
    #dt['bandwidth']=initiatior_payload
    return colliding_coords_vector#, dt

    # for col_id in range(len(collisions)):
    #     collision_points = collisions[col_id]

    #     # iteration storage
    #     encr_rsps = []
    #     responses = []

    #     # extract timestamp info
    #     encr_challenge = msg["challenges"][col_id]
    #     challenge_vector.append(encr_challenge)
    #     origin = msg["origins"][col_id]
    #     time_division = msg["time_divisions"][col_id]

    #     # rescale timestamps
    #     rescaled_timestamps = []
    #     for x, y, t in collision_points:
    #         rescaled_timestamps.append(origin - int(t))

    #     # get all colliding A-capsules
    #     timestamp_ids = util.get_hashed_time_capsule_ids_reversed(rescaled_timestamps, time_division)
    #     capsule_ids_vector.append(timestamp_ids)

    #     # compute e
    #     e = []
    #     for k in range(len(timestamp_ids)):
    #         e.append(pow(timestamp_ids[k], -1, phi))

    #     # compute responses
    #     for k in range(len(timestamp_ids)):
    #         c = encr_challenge
    #         response_number = pow(c, e[k], n)
    #         responses.append(response_number)
    #         encr_rsps.append(hashlib.sha256(str(response_number).encode('utf-8')).hexdigest())

    #     # store responses before sending
    #     response_vector.append(responses)
    #     hashed_response_vector.append(encr_rsps)

    # # send responses to initiator
    # tls.send(conn, json.dumps({
    #     "code": codes.TIME_ENCR_RESPONSES,
    #     "responses": hashed_response_vector
    # }))

    # # receive time proofs from initiator
    # msg = json.loads(tls.receive(conn))
    # initiatior_payload += sys.getsizeof(msg)
    # code = msg["code"]
    # # check message code
    # if code != codes.TIME_PROOF:
    #     tls.wrong_code_received(codes.TIME_PROOF, code)

    # # check for collisions
    # for col_i in range(len(collisions)):
    #     # extract proof of proximity
    #     w_proof = msg["proofs"][col_i]
    #     for j in range(len(response_vector[col_i])):
    #         # get capsule identifier
    #         caps_id = capsule_ids_vector[col_i][j]
    #         # generate w check
    #         w_check = hashlib.sha256(
    #             (str(response_vector[col_i][j]) + str(caps_id)).encode('utf-8')
    #         ).hexdigest()
    #         # check collission between capsule identifiers
    #         if w_check == w_proof:
    #             # get colliding coordinates
    #             colliding_coords_vector.append(collisions[col_i])
    #             # only one match possible so break from loop
    #             break

    # return colliding_coords_vector, initiatior_payload
