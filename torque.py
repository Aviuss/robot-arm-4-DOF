import numpy as np
import math
import matplotlib.pyplot as plt

def returnTorque(q1, q2, q3, q4):    
    Jac_geo_L_at_elbow_1 = np.array([[0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]])

    Jac_geo_L_at_elbow_2 = np.array([[-L1*math.cos(q2)*math.sin(q1), -L1*math.cos(q1)*math.sin(q2), 0, 0],
    [ L1*math.cos(q1)*math.cos(q2), -L1*math.sin(q1)*math.sin(q2), 0, 0],
    [                  0,         -L1*math.cos(q2), 0, 0]])
 
    Jac_geo_L_at_elbow_3 = np.array(
    [[-math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)), -math.cos(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)), -L2*math.sin(q2 + q3)*math.cos(q1), 0],
    [ math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)), -math.sin(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)), -L2*math.sin(q2 + q3)*math.sin(q1), 0],
    [                                      0,          - L2*math.cos(q2 + q3) - L1*math.cos(q2),         -L2*math.cos(q2 + q3), 0]])


    Jac_geo_L_at_hand = np.array([[- math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) - L3*math.cos(q2 + q3 + q4)*math.sin(q1), - math.cos(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)) - L3*math.sin(q2 + q3 + q4)*math.cos(q1), -math.cos(q1)*(L2*math.sin(q2 + q3) + L3*math.sin(q2 + q3 + q4)), -L3*math.sin(q2 + q3 + q4)*math.cos(q1)],
        [  math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.cos(q1), - math.sin(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)) - L3*math.sin(q2 + q3 + q4)*math.sin(q1), -math.sin(q1)*(L2*math.sin(q2 + q3) + L3*math.sin(q2 + q3 + q4)), -L3*math.sin(q2 + q3 + q4)*math.sin(q1)],
        [                                                                      0,                   - L2*math.cos(q2 + q3) - L1*math.cos(q2) - L3*math.cos(q2 + q3 + q4),          - L2*math.cos(q2 + q3) - L3*math.cos(q2 + q3 + q4),         -L3*math.cos(q2 + q3 + q4)]]
    ) 

   
    Jac_geo_L_at_middle_of_l1 = np.array(
        [[-(L1*math.cos(q2)*math.sin(q1))/2, -(L1*math.cos(q1)*math.sin(q2))/2, 0, 0],
        [ (L1*math.cos(q1)*math.cos(q2))/2, -(L1*math.sin(q1)*math.sin(q2))/2, 0, 0],
        [                      0,         -(L1*math.cos(q2))/2, 0, 0]])
 
     
    Jac_geo_L_at_middle_of_l2 = np.array([[-(math.sin(q1)*(L2*math.cos(q2 + q3) + 2*L1*math.cos(q2)))/2, -(math.cos(q1)*(L2*math.sin(q2 + q3) + 2*L1*math.sin(q2)))/2, -(L2*math.sin(q2 + q3)*math.cos(q1))/2, 0],
        [ (math.cos(q1)*(L2*math.cos(q2 + q3) + 2*L1*math.cos(q2)))/2, -(math.sin(q1)*(L2*math.sin(q2 + q3) + 2*L1*math.sin(q2)))/2, -(L2*math.sin(q2 + q3)*math.sin(q1))/2, 0],
        [                                            0,            - (L2*math.cos(q2 + q3))/2 - L1*math.cos(q2),         -(L2*math.cos(q2 + q3))/2, 0]]
    )
 
     
    Jac_geo_L_at_middle_of_l3 = np.array([[- math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) - (L3*math.cos(q2 + q3 + q4)*math.sin(q1))/2, - math.cos(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)) - (L3*math.sin(q2 + q3 + q4)*math.cos(q1))/2, -(math.cos(q1)*(2*L2*math.sin(q2 + q3) + L3*math.sin(q2 + q3 + q4)))/2, -(L3*math.sin(q2 + q3 + q4)*math.cos(q1))/2],
    [  math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + (L3*math.cos(q2 + q3 + q4)*math.cos(q1))/2, - math.sin(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)) - (L3*math.sin(q2 + q3 + q4)*math.sin(q1))/2, -(math.sin(q1)*(2*L2*math.sin(q2 + q3) + L3*math.sin(q2 + q3 + q4)))/2, -(L3*math.sin(q2 + q3 + q4)*math.sin(q1))/2],
    [                                                                          0,                   - L2*math.cos(q2 + q3) - L1*math.cos(q2) - (L3*math.cos(q2 + q3 + q4))/2,            - L2*math.cos(q2 + q3) - (L3*math.cos(q2 + q3 + q4))/2,         -(L3*math.cos(q2 + q3 + q4))/2]]
    )
 

    torque = np.transpose(np.array([0,0,0,0]))

    #engineMass = 50 * 0.001; # 40 grams
    armSingularMass = 150 * 0.001; # in grams
    
    torque = torque + np.dot(np.transpose(Jac_geo_L_at_elbow_1), np.transpose(np.array([0,0,56 * 0.001 * -9.81])))
    torque = torque + np.dot(np.transpose(Jac_geo_L_at_elbow_2), np.transpose(np.array([0,0,40 * 0.001 * -9.81])))
    torque = torque + np.dot(np.transpose(Jac_geo_L_at_elbow_3), np.transpose(np.array([0,0,10 * 0.001 * -9.81])))
    torque = torque + np.dot(np.transpose(Jac_geo_L_at_hand), np.transpose(np.array([0,0,50 * 0.001 * -9.81])))


    torque = torque + np.dot(np.transpose(Jac_geo_L_at_middle_of_l1), np.transpose(np.array([0,0,armSingularMass * -9.81])))
    torque = torque + np.dot(np.transpose(Jac_geo_L_at_middle_of_l2), np.transpose(np.array([0,0,armSingularMass * -9.81])))
    torque = torque + np.dot(np.transpose(Jac_geo_L_at_middle_of_l3), np.transpose(np.array([0,0,armSingularMass * -9.81])))

    torque = torque*10.1972 # in kg cm 
    return torque


if __name__ == "__main__":
    L1 = 12 * 0.01 # in m
    L2 = 10 * 0.01 # in m
    L3 = 10 * 0.01 # in m
    firstElbowOffset = 5 * 0.01 # in m

    joint1 = 0
    #joint2 = 0 #    0 -- 180
    #joint3 = 0 # -180 --   0
    #joint4 = 0 # -180 --   0


    fig = plt.figure()
    ax = fig.add_subplot()


    step = 15
    for j2 in range(0, 180+1, step):
        for j3 in range(0, -180-1, -step):
            for j4 in range(0, -180-1, -step):            
                q1 = math.radians(joint1)
                q2 = math.radians(-j2)
                q3 = -math.pi/2 -math.radians( j3)
                q4 = -math.pi/2 -math.radians( j4)

                trq = returnTorque(q1, q2, q3, q4)
                
                                
                p_hand = np.array([
                    [        math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.cos(q1)],
                    [math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.sin(q1)],
                    [firstElbowOffset - L2*math.sin(q2 + q3) - L1*math.sin(q2) - L3*math.sin(q2 + q3 + q4)]
                ]) * 100

                if trq[1] > 9:
                    ax.scatter(p_hand[0], p_hand[2], marker='x', color="red")
                elif trq[2] > 5:
                    ax.scatter(p_hand[0], p_hand[2], marker='x', color="green")
                elif trq[3] > 1.8:
                    ax.scatter(p_hand[0], p_hand[2], marker='x', color="blue")
                #else:
                #    ax.scatter(j2, j3, j4, marker='o')

    for i in range(0, 360 + 1, 1):
        d = L1*100 + L2*100 + L3*100
        v = np.array([math.cos(math.radians(i)), math.sin(math.radians(i))])  * d
        ax.scatter(v[0], v[1]+firstElbowOffset*100, marker='o', color="black")

    ax.set_xlabel('ox')
    ax.set_ylabel('oy')
    #ax.set_zlabel('j4 Label')
    #ax.set_xlim(0, 180)
    #ax.set_ylim(-180, 0)
    #ax.set_zlim(-180, 0)
    
    ax.scatter(0,0+firstElbowOffset*100, marker='o', color="black")
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    #ax.set_zlim(-50 * 0.01, 50 * 0.01)

    plt.show()