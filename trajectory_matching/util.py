import hashlib
import json
import math

import numpy as np
import trajectory_matching.tesselation as sd


def get_capsule_identifier(x, y, r, h):
    """
    Mapping locations into equal sized sets of unique values, representing unique space capsule identifiers.
    """
    # get unique values in x and y dimension
    id_x = int(math.floor(x / h))
    id_y = int(math.floor(y / (2 * r)))
    #id_x = int(round(x / h))
    #id_y = int(round(y / (2 * r)))
    #print("capsule identifier")
    #print([x,y])
    #print(h)
    #print(r)
    #print("capsule identifier end")
    # concatenate hashes and return created capsule identifier
    return id_x, id_y


def get_capsule_identifier_last(x, y, r, h):
    """
    Mapping locations into equal sized sets of unique values, representing unique space capsule identifiers.
    """
    # get unique values in x and y dimension
    id_x = int(math.floor(x / h))
    id_y = int(math.floor(y / (2 * r)))
    #id_x = int(round(x / h))
    #id_y = int(round(y / (2 * r)))
    #print("capsule identifier")
    #print([x,y])
    #print(h)
    #print(r)
    #print("capsule identifier end")
    # concatenate hashes and return created capsule identifier
    return id_x, id_y


def get_hashed_capsule_identifier(id_x, id_y):
    """
    Hash space capsule identifier.
    """
    # hash the unique values
    hash_x = str(int(hashlib.sha256(str(id_x).encode('utf-8')).hexdigest(), 16) % (10 ** 6))
    hash_y = str(int(hashlib.sha256(str(id_y).encode('utf-8')).hexdigest(), 16) % (10 ** 6))
    # concatenate hashes and return created capsule identifier
    return 2 * abs(int(hash_x + hash_y)) + 1


def get_colliding_caps_ids(b_trajectory, b_r, b_theta, a_r, a_h):
    """
    Given the trajectory, the radius r and the angle theta from a space capsule B and given the radius r and length
    from A's space capsules, return the space capsule id's from all intersections between A's space capsules and the
    given B space capsule.
    """
    # list to return with all colliding A-capsules id's
    # intersect_caps_ids = []
    # determine start and end point of B
    # b_A, b_B = b_trajectory[0], b_trajectory[-1]
    # calculate the 4 corners of the rectangle around the B capsule and create a polygon
    # sqrt_v = math.sqrt((b_r ** 2) + (b_r ** 2))
    # var1 = math.cos(b_theta + 0.25 * math.pi) * sqrt_v
    # var2 = math.sin(b_theta + 0.25 * math.pi) * sqrt_v
    # corners = [
    #     [b_A[0] - var1, b_A[1] - var2],
    #     [b_A[0] - var2, b_A[1] + var1],
    #     [b_B[0] + var2, b_B[1] - var1],
    #     [b_B[0] + var1, b_B[1] + var2],
    # ]

    # corners2 = [
    #     [b_A[0] - var1, b_A[1]],
    #     [b_A[0], b_A[1] + var2],
    #     [b_A[0], b_A[1] - var2],
    #     [b_B[0] + var1, b_B[1]],
    #     [b_B[0], b_B[1] + var2],
    #     [b_B[0], b_B[1] - var2],
    # ]
    # get capsule ids for each corner
    capsules = []
    #print("capsule")
    for x, y in b_trajectory:
        id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
        #print("possible colliding:")
        #print([x,y])
        #print([id_x,id_y])

        id_hashed = get_hashed_capsule_identifier(id_x,id_y)
        capsules.append(id_hashed)
    capsules = list(set(capsules))

    #COMPUTE the six points

    #COMPUTE the distance

    #gamma = min()

    #Compute points to add
    #beta = math.pi/2-b_theta
    #x_add = a_r * math.sin(beta)
    #y_add = a_r * math.sin(b_theta)
    
    
    # capsules2 = []   
    # for x_coord, y_coord in corners2:
    #     x ,y = get_capsule_identifier (x_coord, y_coord, a_r, a_h)
    #     id = get_hashed_capsule_identifier(x,y)
    #     capsules.append(id)
    # capsules = list(set(capsules))
    # caps_ids = [get_capsule_identifier(x_coord, y_coord, a_r, a_h) for x_coord, y_coord in corners2]
    # for x,y in caps_ids:
    #     id = get_hashed_capsule_identifier(x,y)
    #     capsules2.append(id)
    # # determine all A-capsules id's that can possibly collide with the B-capsule
    # id_x_min, id_x_max = min([x for x, y in caps_ids]), max([x for x, y in caps_ids])
    # id_y_min, id_y_max = min([y for x, y in caps_ids]), max([y for x, y in caps_ids])
    # capsule_ids = [[x, y] for x in range(id_x_min, id_x_max + 1) for y in range(id_y_min, id_y_max + 1)]
    # # for each possible colliding A-capsule, check whether it collides with the B-capsule
    # for x, y in capsule_ids:
    #     # calculate start and end point of A-capsule
    #     a_A = [x * a_h + a_r, y * 2 * a_r + a_r]
    #     a_B = [x * a_h + a_h - a_r, y * 2 * a_r + a_r]
    #     # check if A-capsule intersects with B-capsule
    #     if sd.capsules_intersect(a_A, a_B, a_r, b_A, b_B, b_r):
    #         # create hashed capsule id of this A-capsule and add to the colliding capsules id's list
    #         intersect_caps_ids.append(get_hashed_capsule_identifier(x, y))
    # return all intersecting capsule id's
    #return intersect_caps_ids
    return capsules 

def get_colliding_caps_ids_new(b_trajectory,b_h, b_r, b_theta, a_r, a_h):

    beta = math.pi/2-b_theta
    x_add = b_r * math.sin(beta)
    y_add = b_r * math.sin(b_theta)
    shifts = np.array([x_add,y_add])

    #x_add = 1 * math.sin(beta)
    #y_add = 1 * math.sin(b_theta)
    #shifts = np.array([x_add,y_add])
    #COMPUTE the six points

    b_A, b_B = b_trajectory[0], b_trajectory[-1]

    #print("start point:")
    #print([b_A, b_B])
    #print("end point")
    
    b_A_tip = np.subtract(b_A,shifts) #[b_A.x - x_add, b_A.y-y_add]
    b_B_base = np.add(b_B,shifts) #[b_B.x + x_add, b_B.y+y_add]
    direction = np.subtract(b_B,b_A)

    #s_x_tip =  b_A_tip[0]-b_A.x
    #s_y_tip =  b_A_tip[1]-b_A.y
    negative_theta = -math.pi/2
    positive_theta = math.pi/2
    s_tip = np.subtract(b_A_tip,b_A)
    n_rot = np.matrix(np.array([[math.cos(negative_theta), -math.sin(negative_theta)],[math.sin(negative_theta) ,math.cos(negative_theta)]])).H
    p_rot = np.matrix(np.array([[math.cos(positive_theta), -math.sin(positive_theta)],[math.sin(positive_theta) ,math.cos(positive_theta)]])).H
    s_rot_tip_n = np.matmul(s_tip,n_rot).A[0]
    s_rot_tip_p = np.matmul(s_tip,p_rot).A[0]
    v1 = np.add(s_rot_tip_n,b_A)
    v2 = np.add(s_rot_tip_p,b_A)
    #print(sd.distance(v1,b_A))
    #print(sd.distance(v2,b_A))
    #print(sd.distance(b_A_tip,b_A))


    s_base = np.subtract(b_B_base,b_B)
    s_rot_base_n = np.matmul(s_base,n_rot).A[0]
    s_rot_base_p = np.matmul(s_base,p_rot).A[0]
    v3 = np.add(s_rot_base_n,b_B)
    v4 = np.add(s_rot_base_p,b_B)
    #print(sd.distance(v3,b_B))
    #print(sd.distance(v4,b_B))
    #print(sd.distance(b_B_base,b_B))
    #COMPUTE the distance
    capsules = []
    #gamma = min(a_h,2*a_r)
    gamma = 3 
    six_points = [b_A_tip,v1,v2,b_B_base,v3,v4]
    boundery_points = six_points
    for x, y in six_points:
        id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
        #print("possible colliding:")
        #print([x,y])
        #print([id_x,id_y])
        if computerect(id_x,id_y,x,y,a_r,a_h):
            id_hashed = get_hashed_capsule_identifier(id_x,id_y)
            capsules.append(id_hashed)
    capsules = list(set(capsules))
    mv_p1 = b_A_tip
    mv_p2 = v1
    mv_p3 = v2
    if direction[0]>0:
        mv_p1 = b_A_tip
        mv_p2 = v1
        mv_p3 = v2
    else:
        mv_p1 = b_B_base
        mv_p2 = v4
        mv_p3 = v3 # changed swapped
        gamma = -gamma
    sign = 0
    if gamma>0:
        sign = 1#gamma
    else:
        sign = -1#gamma
    sign = sign*0.5
    gamma_add = np.array([sign*x_add,sign*y_add])
    #print([mv_p1[0]+sign*x_add,mv_p1[1]+sign*y_add])
    mv_p1_new = np.add(mv_p1,gamma_add)
    mv_p2_new = np.add(mv_p2,gamma_add)
    mv_p3_new = np.add(mv_p3,gamma_add)
    condition = False
    if direction[0]>0:
         
        condition = mv_p1_new[0] < b_B_base[0] and mv_p2_new[0] < v4[0] and mv_p3_new[0] < v3[0]
    else:
        condition = mv_p1_new[0] < b_A_tip[0] and mv_p2_new[0] < v1[0] and mv_p3_new[0] < v2[0]  

    l = [mv_p1_new,mv_p2_new,mv_p3_new]
    if condition:
        for x, y in l:
            id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
            #print("possible colliding:")
            #print([x,y])
            #print([id_x,id_y])
            if computerect(id_x,id_y,x,y,a_r,a_h):
                id_hashed = get_hashed_capsule_identifier(id_x,id_y)
                capsules.append(id_hashed)
    #if sd.distance(b_A,b_B)+ 2*b_r > gamma and len(capsules)!=1:
    capsules = list(set(capsules))
    counter = 0    
    while(condition):

        mv_p1_new = np.add(mv_p1_new,gamma_add)
        mv_p2_new = np.add(mv_p2_new,gamma_add)
        mv_p3_new = np.add(mv_p3_new,gamma_add)

        # l = [mv_p1_new,mv_p2_new,mv_p3_new]
        # for x, y in l:
        #     id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
        #     print("possible colliding lines:")
        #     print([x,y])
        #     print([id_x,id_y])

        #     if computerect(id_x,id_y,x,y,a_r,a_h):
        #         id_hashed = get_hashed_capsule_identifier(id_x,id_y)
        #         capsules.append(id_hashed)
        if direction[0]>0:
            #print(mv_p1_new[0] < b_B_base[0])
            #print( mv_p2_new[0] < v4[0])
            #print(mv_p3_new[0] < v3[0])
            condition = mv_p1_new[0] < b_B_base[0] and mv_p2_new[0] < v4[0] and mv_p3_new[0] < v3[0]
        else:
            condition = mv_p1_new[0] < b_A_tip[0] and mv_p2_new[0] < v1[0] and mv_p3_new[0] < v2[0]  
        if(condition):
            l = [mv_p1_new,mv_p2_new,mv_p3_new]
            for x, y in l:
                id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
                #print("possible colliding lines:")
                #print([x,y])
                #print([id_x,id_y])

                if computerect(id_x,id_y,x,y,a_r,a_h):
                    id_hashed = get_hashed_capsule_identifier(id_x,id_y)
                    capsules.append(id_hashed) 
        capsules = list(set(capsules))   
        counter += 1
    #print("counter:",counter) 

    i = -math.pi/2
    while  i <= math.pi/2:
            rot = np.matrix(np.array([[math.cos(i), -math.sin(i)],[math.sin(i) ,math.cos(i)]])).H
            
            s_base_rot = np.matmul(s_base,rot).A[0]
            s_tip_rot = np.matmul(s_tip,rot).A[0]
            v_temp1 = np.add(s_base_rot,b_B)
            v_temp2 = np.add(s_tip_rot,b_A)
            x_temp1, y_temp1 = v_temp1
            x_temp2, y_temp2 = v_temp2
            id_x_temp1,id_y_temp1 = get_capsule_identifier(x_temp1,y_temp1, a_r, a_h)
            id_x_temp2,id_y_temp2 = get_capsule_identifier(x_temp2,y_temp2, a_r, a_h)
            #print("possible colliding:")
            #print([x,y])
            #print([id_x,id_y])

            if computerect(id_x_temp1,id_y_temp1, x_temp1,y_temp1,a_r, a_h):
                    id_hashed_temp1 = get_hashed_capsule_identifier(id_x_temp1,id_y_temp1)
                    capsules.append(id_hashed_temp1)
                
            if computerect(id_x_temp2,id_y_temp2, x_temp2,y_temp2,a_r, a_h):
                    id_hashed_temp2 = get_hashed_capsule_identifier(id_x_temp2,id_y_temp2)
                    capsules.append(id_hashed_temp2)
            capsules = list(set(capsules))
            i+=(math.pi/2)/10
    ##############################
    #    If repsonder is bigger  #
    #     than initator          #                
    ##############################     
    condition_loop = True
    a_r_const = a_r
    if b_r > a_r:
        while(condition_loop):
            beta = math.pi/2-b_theta
            x_add = a_r_const * math.sin(beta)
            y_add = a_r_const * math.sin(b_theta)
            shifts = np.array([x_add,y_add])

    #x_add = 1 * math.sin(beta)
    #y_add = 1 * math.sin(b_theta)
    #shifts = np.array([x_add,y_add])
    #COMPUTE the six points

            b_A, b_B = b_trajectory[0], b_trajectory[-1]

            #print("start point:")
            #print([b_A, b_B])
            #print("end point")
    
            b_A_tip = np.subtract(b_A,shifts) #[b_A.x - x_add, b_A.y-y_add]
            b_B_base = np.add(b_B,shifts) #[b_B.x + x_add, b_B.y+y_add]
            direction = np.subtract(b_B,b_A)

    #s_x_tip =  b_A_tip[0]-b_A.x
    #s_y_tip =  b_A_tip[1]-b_A.y
            negative_theta = -math.pi/2
            positive_theta = math.pi/2
            s_tip = np.subtract(b_A_tip,b_A)
            n_rot = np.matrix(np.array([[math.cos(negative_theta), -math.sin(negative_theta)],[math.sin(negative_theta) ,math.cos(negative_theta)]])).H
            p_rot = np.matrix(np.array([[math.cos(positive_theta), -math.sin(positive_theta)],[math.sin(positive_theta) ,math.cos(positive_theta)]])).H
            s_rot_tip_n = np.matmul(s_tip,n_rot).A[0]
            s_rot_tip_p = np.matmul(s_tip,p_rot).A[0]
            v1 = np.add(s_rot_tip_n,b_A)
            v2 = np.add(s_rot_tip_p,b_A)
            #print(sd.distance(v1,b_A))
            #print(sd.distance(v2,b_A))
            #print(sd.distance(b_A_tip,b_A))


            s_base = np.subtract(b_B_base,b_B)
            s_rot_base_n = np.matmul(s_base,n_rot).A[0]
            s_rot_base_p = np.matmul(s_base,p_rot).A[0]
            v3 = np.add(s_rot_base_n,b_B)
            v4 = np.add(s_rot_base_p,b_B)
            #print(sd.distance(v3,b_B))
            #print(sd.distance(v4,b_B))
            #print(sd.distance(b_B_base,b_B))
            gamma = 3 
            six_points = [b_A_tip,v1,v2,b_B_base,v3,v4]
            for x, y in six_points:
                id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
                #print("possible colliding:")
                #print([x,y])
                #print([id_x,id_y])
                #if computerect(id_x,id_y,x,y,a_r,a_h):
                id_hashed = get_hashed_capsule_identifier(id_x,id_y)
                capsules.append(id_hashed)
            capsules = list(set(capsules))
            mv_p1 = b_A_tip
            mv_p2 = v1
            mv_p3 = v2
            if direction[0]>0:
                mv_p1 = b_A_tip
                mv_p2 = v1
                mv_p3 = v2
            else:
                mv_p1 = b_B_base
                mv_p2 = v3
                mv_p3 = v4
                gamma = -gamma
            sign = 0
            if gamma>0:
                sign = 1#gamma
            else:
                sign = -1#gamma
            sign = sign
            gamma_add = np.array([sign*x_add,sign*y_add])
            #print([mv_p1[0]+sign*x_add,mv_p1[1]+sign*y_add])
            mv_p1_new = np.add(mv_p1,gamma_add)
            mv_p2_new = np.add(mv_p2,gamma_add)
            mv_p3_new = np.add(mv_p3,gamma_add)
            condition = False
            if direction[0] >0 :
                condition = mv_p1_new[0]< b_B_base[0]
            else:
                condition = mv_p1_new[0]< b_A_tip[0]

            l = [mv_p1_new,mv_p2_new,mv_p3_new]
            for x, y in l:
                id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
                #print("possible colliding:")
                #print([x,y])
                #print([id_x,id_y])
                #if computerect(id_x,id_y,x,y,a_r,a_h):
                id_hashed = get_hashed_capsule_identifier(id_x,id_y)
                capsules.append(id_hashed)
    #if sd.distance(b_A,b_B)+ 2*b_r > gamma and len(capsules)!=1:
            capsules = list(set(capsules))
            counter = 0    
            while(condition):

                mv_p1_new = np.add(mv_p1_new,gamma_add)
                mv_p2_new = np.add(mv_p2_new,gamma_add)
                mv_p3_new = np.add(mv_p3_new,gamma_add)

                l = [mv_p1_new,mv_p2_new,mv_p3_new]
                for x, y in l:
                    id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
                    #print("possible colliding lines:")
                    #print([x,y])
                    #print([id_x,id_y])
                    #if computerect(id_x,id_y,x,y,a_r,a_h):
                    id_hashed = get_hashed_capsule_identifier(id_x,id_y)
                    capsules.append(id_hashed)
                if direction[0]>0:
                    #print(mv_p1_new[0] < b_B_base[0])
                    #print( mv_p2_new[0] < v4[0])
                    #print(mv_p3_new[0] < v3[0])
                    condition = mv_p1_new[0] < b_B_base[0] and mv_p2_new[0] < v4[0] and mv_p3_new[0] < v3[0]
                else:
                    condition = mv_p1_new[0] < b_A_tip[0] and mv_p3_new[0] < v1[0] and mv_p2_new[0] < v2[0]   
                capsules = list(set(capsules))   
                counter += 1
            #print("counter:",counter) 

            i = -math.pi/2
            while  i <= math.pi/2:
                rot = np.matrix(np.array([[math.cos(i), -math.sin(i)],[math.sin(i) ,math.cos(i)]])).H
            
                s_base_rot = np.matmul(s_base,rot).A[0]
                s_tip_rot = np.matmul(s_tip,rot).A[0]
                v_temp1 = np.add(s_base_rot,b_B)
                v_temp2 = np.add(s_tip_rot,b_A)
                x_temp1, y_temp1 = v_temp1
                x_temp2, y_temp2 = v_temp2
                id_x_temp1,id_y_temp1 = get_capsule_identifier(x_temp1,y_temp1, a_r, a_h)
                id_x_temp2,id_y_temp2 = get_capsule_identifier(x_temp2,y_temp2, a_r, a_h)
                #print("possible colliding:")
                #print([x,y])
                #print([id_x,id_y])
                
                #if computerect(id_x_temp1,id_y_temp1, x_temp1,y_temp1,a_r, a_h):
                id_hashed_temp1 = get_hashed_capsule_identifier(id_x_temp1,id_y_temp1)
                capsules.append(id_hashed_temp1)
                
                #if computerect(id_x_temp2,id_y_temp2, x_temp2,y_temp2,a_r, a_h):
                id_hashed_temp2 = get_hashed_capsule_identifier(id_x_temp2,id_y_temp2)
                capsules.append(id_hashed_temp2)
                capsules = list(set(capsules))
                i+=(math.pi/2)/10
            a_r+=a_r
            if sign == -1:
                #condition_loop = (b_A_tip[0]>boundery_points[0][0])  and (v1[0]<boundery_points[1][0]) and (v2[0]>boundery_points[2][0])
                condition_loop = ((b_A_tip[0]<boundery_points[0][0])  and (v1[0]<boundery_points[1][0]) and (v2[0]>boundery_points[2][0]) and a_r < b_r) or ((b_A_tip[0]<boundery_points[0][0])  and (v1[0]>boundery_points[1][0]) and (v2[0]<boundery_points[2][0]) and a_r < b_r)
            else:
                #condition_loop = (b_A_tip[0]>boundery_points[0][0])  and (v1[0]<boundery_points[1][0]) and (v2[0]>boundery_points[2][0]) and a_r < b_r
                condition_loop = (b_A_tip[0]>boundery_points[0][0])  and (v1[0]>boundery_points[1][0]) and (v2[0]<boundery_points[2][0]) and a_r < b_r

    # if b_A_tip[0]> 0 and b_B_base[0] > 0:
    #     if b_A_tip[0]<b_B_base[0]:

    #         b_A_tip_new = b_A_tip
    #         v1_new = v1
    #         v2_new = v2
    #         gamma_add = np.array([gamma*x_add,gamma*y_add])
    #         while b_A_tip_new[0] < b_B_base[0] and  b_A_tip_new[1] < b_B_base[1]: #and v1_new [0] < v3[0] and v1_new[1] < v3[1] and v2_new[0] < v4[0] and v2_new[1] and v4[1]:
    #             l = []

    #             b_A_tip_new = np.add(b_A_tip_new,gamma_add)
    #             v1_new = np.add(v1_new,gamma_add)
    #             v2_new = np.add(v2_new,gamma_add)
    #             l = [b_A_tip_new,v1_new,v2_new]
    #             for x, y in l:
    #                 id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
    #                 print("possible colliding:")
    #                 print([x,y])
    #                 print([id_x,id_y])

    #                 id_hashed = get_hashed_capsule_identifier(id_x,id_y)
    #                 capsules.append(id_hashed)
    #     else:
    #         b_A_tip_new = b_B_base
    #         v1_new = v3
    #         v2_new = v4
    #         gamma = -gamma
    #         gamma_add = np.array([gamma*x_add,gamma*y_add])
    #         while b_A_tip_new[0] > b_A_tip[0] and  b_A_tip_new[1] > b_A_tip[1]: #and v1_new [0] < v3[0] and v1_new[1] < v3[1] and v2_new[0] < v4[0] and v2_new[1] and v4[1]:
    #             l = []

    #             b_A_tip_new = np.add(b_A_tip_new,gamma_add)
    #             v1_new = np.add(v1_new,gamma_add)
    #             v2_new = np.add(v2_new,gamma_add)
    #             l = [b_A_tip_new,v1_new,v2_new]
    #             for x, y in l:
    #                 id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
    #                 print("possible colliding:")
    #                 print([x,y])
    #                 print([id_x,id_y])

    #                 id_hashed = get_hashed_capsule_identifier(id_x,id_y)
    #                 capsules.append(id_hashed)
    
    # if b_A_tip[0]< 0 and b_B_base[0] < 0:
    #     if b_A_tip[0]<b_B_base[0]:

    #         b_A_tip_new = b_A_tip
    #         v1_new = v1
    #         v2_new = v2
    #         gamma = -gamma
    #         gamma_add = np.array([gamma*x_add,gamma*y_add])
    #         while b_A_tip_new[0] < b_B_base[0] and  b_A_tip_new[1] < b_B_base[1]: #and v1_new [0] < v3[0] and v1_new[1] < v3[1] and v2_new[0] < v4[0] and v2_new[1] and v4[1]:
    #             l = []

    #             b_A_tip_new = np.add(b_A_tip_new,gamma_add)
    #             v1_new = np.add(v1_new,gamma_add)
    #             v2_new = np.add(v2_new,gamma_add)
    #             l = [b_A_tip_new,v1_new,v2_new]
    #             for x, y in l:
    #                 id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
    #                 print("possible colliding:")
    #                 print([x,y])
    #                 print([id_x,id_y])

    #                 id_hashed = get_hashed_capsule_identifier(id_x,id_y)
    #                 capsules.append(id_hashed)
    #     else:
    #         b_A_tip_new = b_B_base
    #         v1_new = v3
    #         v2_new = v4
    #         gamma = -gamma
    #         gamma_add = np.array([gamma*x_add,gamma*y_add])
    #         while b_A_tip_new[0] < b_A_tip[0] and  b_A_tip_new[1] < b_A_tip[1]: #and v1_new [0] < v3[0] and v1_new[1] < v3[1] and v2_new[0] < v4[0] and v2_new[1] and v4[1]:
    #             l = []

    #             b_A_tip_new = np.add(b_A_tip_new,gamma_add)
    #             v1_new = np.add(v1_new,gamma_add)
    #             v2_new = np.add(v2_new,gamma_add)
    #             l = [b_A_tip_new,v1_new,v2_new]
    #             for x, y in l:
    #                 id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
    #                 print("possible colliding:")
    #                 print([x,y])
    #                 print([id_x,id_y])

    #                 id_hashed = get_hashed_capsule_identifier(id_x,id_y)
    #                 capsules.append(id_hashed)
    # for x, y in b_trajectory:
    #     id_x,id_y = get_capsule_identifier(x,y, a_r, a_h)
    #     print("possible colliding actual points:")
    #     print([x,y])
    #     print([id_x,id_y])

    #     id_hashed = get_hashed_capsule_identifier(id_x,id_y)
    #     capsules.append(id_hashed)
    capsules = list(set(capsules))



    #Compute points to add

    return capsules 

def get_colliding_caps_ids_last(b_trajectory, b_r, b_theta, a_r, a_h):
    """
    Given the trajectory, the radius r and the angle theta from a space capsule B and given the radius r and length
    from A's space capsules, return the space capsule id's from all intersections between A's space capsules and the
    given B space capsule.
    """

    capsules = []
    #print("capsule")
    for x, y in b_trajectory:
        id_x,id_y = get_capsule_identifier_last(x,y, a_r, a_h)
        #print("possible colliding:")
        #print([x,y])
        #print([id_x,id_y])

        id_hashed = get_hashed_capsule_identifier(id_x,id_y)
        capsules.append(id_hashed)
    capsules = list(set(capsules))
    

    return capsules 


def remove_duplicates(lst):
    """
    Remove duplicates from a list with items.
    """
    new_str_list = []
    new_list = []
    for lst_item in lst:
        js = json.dumps(lst_item.tolist())
        if js not in new_str_list:
            new_str_list.append(js)
    for item in new_str_list:
        new_list.append(np.array(json.loads(item)))
    return new_list


def split_in_sections(coordinates: np.array, num_groups: int, overlap: int):
    """
    Splits given coordinate list into a specified number of new lists, with a given overlap.
    """
    group_size = max(math.floor((len(coordinates) + num_groups - 1) / num_groups), 2)
    remainder = (len(coordinates) + num_groups - 1) % num_groups
    sub_traj, index, remainder_activation = [], 0, 0
    for j in range(num_groups):
        remainder_activation += 1
        if remainder != 0 and remainder_activation * remainder % num_groups == 0:
            sub_traj.append(coordinates[index:index + group_size + 1])
            index += group_size + 1 - overlap
        else:
            sub_traj.append(coordinates[index:index + group_size])
            index += group_size - overlap
    return sub_traj


def get_time_capsule_id(point, time_division):
    """
    Mapping timestamps into equal sized sets of unique values, representing unique identifiers.
    """
     #return int(math.floor(point / time_division))
    #print(int(math.floor(abs(point/time_division))))
    return int((math.ceil(abs(point / time_division))))

def get_time_capsule_id_floor(point, time_division):
    """
    Mapping timestamps into equal sized sets of unique values, representing unique identifiers.
    """
     #return int(math.floor(point / time_division))
    return int((math.floor(abs(point / time_division))))

def get_hashed_time_capsule_id(id):
    """
    Hashed time identifier.
    """
    return 2 * abs(int(hashlib.sha256(str(id).encode('utf-8')).hexdigest(), 16) % (10 ** 12)) + 1


def get_hashed_time_capsule_ids(timestamps, time_division):
    """
    Given the timestamps and the time_division of A.
    Return the timestamp id's from all intersections between the A-timestamp and the B-timestamps.
    """
    time_ids = []
    start_time, end_time = timestamps[0], timestamps[-1]
    time = start_time
    while time < end_time:
        time_ids.append(get_hashed_time_capsule_id(get_time_capsule_id(time, time_division)))
        time += time_division
    time_ids.append(get_hashed_time_capsule_id(get_time_capsule_id(end_time, time_division)))
    return time_ids

def get_hashed_time_capsule_ids_reversed(timestamps, time_division):
    """
    Given the timestamps and the time_division of A.
    Return the timestamp id's from all intersections between the A-timestamp and the B-timestamps.
    """
    time_ids = []
    for time in timestamps:
        ids = get_time_capsule_id(time, time_division)
        #print("responder:")
        #print(ids)
        time_ids.append(get_hashed_time_capsule_id(ids))
        #print(time_ids)
    return time_ids

def get_hashed_time_capsule_ids_reversed_floor(timestamps, time_division):
    """
    Given the timestamps and the time_division of A.
    Return the timestamp id's from all intersections between the A-timestamp and the B-timestamps.
    """
    time_ids = []
    for time in timestamps:
        ids = get_time_capsule_id_floor(time, time_division)
        #print("responder:")
        #print(ids)
        time_ids.append(get_hashed_time_capsule_id(ids))
        #print(time_ids)
    return time_ids
    # time_ids = []
    # start_time, end_time = timestamps[0], timestamps[-1]
    # time = start_time
    # while time > end_time:
    #     time_ids.append(get_hashed_time_capsule_id(get_time_capsule_id(time, time_division)))
    #     time -= time_division
    # time_ids.append(get_hashed_time_capsule_id(get_time_capsule_id(end_time, time_division)))
    # return time_ids
# x and y from B responder
def computerect(idx,idy,x,y,a_radius,a_h):

    corner_a_left = [idx*a_h,(idy+1)*(2*a_radius)]
    corner_b_left = [idx*a_h+a_radius, (idy+1)*(2*a_radius) ]
    corner_c_left = [idx*a_h+a_radius,idy*(2*a_radius)]
    corner_d_left = [idx*a_h,idy*(2*a_radius)]

    corner_a_right = [((idx+1)*a_h)-a_radius,(idy+1)*(2*a_radius)]
    corner_b_right = [(idx+1)*a_h,(idy+1)*(2*a_radius)]
    corner_c_right = [(idx+1)*a_h,(idy)*(2*a_radius)]
    corner_d_right = [(idx+1)*a_h-a_radius,(idy)*(2*a_radius)]

    #print(corner_a_left)
    #print(corner_a_right)
    #print(corner_b_left)
    #print(corner_b_right)
    #print(corner_c_left)
    #print(corner_c_right)
    #print(corner_d_left)
    #print(corner_d_right)


    radius_left = [idx*a_h+a_radius,((idy+1)*(2*a_radius))-a_radius]
    radius_right = [((idx+1)*a_h)-a_radius,((idy+1)*(2*a_radius))-a_radius]

    if inrect(corner_a_left[0],corner_a_left[1],corner_b_left[0],corner_b_left[1],corner_c_left[0],corner_c_left[1],corner_d_left[0],corner_d_left[1],x,y):
        return incircle(radius_left[0],radius_left[1],x,y,a_radius)
    elif inrect(corner_a_right[0],corner_a_right[1],corner_b_right[0],corner_b_right[1],corner_c_right[0],corner_c_right[1],corner_d_right[0],corner_d_right[1],x,y):
        return incircle(radius_right[0],radius_right[1],x,y,a_radius)
    else:
        return True



def incircle(x,y,xc,yc,r):
        #if inrect(x1,y1,x2,y2,x3,y3,x4,y4,x,y):
        d = math.sqrt(math.pow((x-xc), 2) + math.pow((y-yc), 2))
        #print([x,y])
        #print([xc,yc])
        #print(d,r)
        if d <= r :
            return True
        else:
            return False

def inrect(x1,y1,x2,y2,x3,y3,x4,y4,x,y):
        A = round(area(x1,y1,x2,y2,x3,y3)+area(x1,y1,x4,y4,x3,y3))
        A1 = area(x, y, x1, y1, x2, y2)
        A2 = area(x, y, x2, y2, x3, y3)
        A3 = area(x, y, x3, y3, x4, y4)
        A4 = area(x, y, x1, y1, x4, y4)
        sumarea = round(A1+A2+A3+A4)
        #print(A1+A2+A3+A4)
        #print("area")
        #print(A)
        #print(sumarea)

        return A == sumarea


def area(x1,y1,x2,y2,x3,y3):
    return abs((x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2.0)