import pandas as pd
import numpy as np
import math
import shlex
import sys

if __name__ == '__main__':
    sys.exit(main())  # next section explains the use of sys.exit.

    

    
    
def points(local_x, local_y, velocity_x, velocity_y, v_len, v_wid):
    """리뉴얼 :: 차량의 4개 꼭지점 좌표를 반환하는 함수
    각 차량 포인트 추출 : corner_1x, corner_1y, ... corner_4x, corner_4y"""

    if pd.isna(velocity_x) == False:
    
        v_x = velocity_x * 1000/3600 # 차량 속도를 초속(m/s)으로 변환
        v_y = velocity_y * 1000/3600
    
        ### Rotation Matrix R_z
        
        if v_x != 0:
        
            theta = math.atan(v_y/v_x) # 속도 벡터가 x축과 이루는 각 theta를 호도법으로 구한다. 
            # math.atan은 두 점 사이 theta의 절대각을 구하는데, 탄젠트 값을 받아 절대각을 "라디안" 값으로 반환한다.
            # 이때 -pi/2 <= theta <= pi/2가 된다.
            
        else: # v_x == 0
            theta = math.pi/2 # v_x가 0이면 탄젠트값은 무한대가 되기 때문이다
        
        # 차량의 4개 꼭지점을 속도벡터에 따라 회전시키기 위한 배열
        R_z = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)],
                        ])
        # R_z = np.array([[-math.sin(theta), -math.cos(theta)],
        #                 [math.cos(theta), -math.sin(theta)],
        #                 ])        
        
        # 회전 전 4개 모서리 점 : 각 차량의 width, len에 따라 결정됨. 1사분면부터 반시계 방향으로
        original_corner = np.array([[1/2*v_len, -1/2*v_len, -1/2*v_len, 1/2*v_len], # 4개 점 x좌표
                                    [1/2*v_wid, 1/2*v_wid, -1/2*v_wid, -1/2*v_wid]]) # 4개 점 y좌표

        # 회전 후 4개 모서리의 점
        rotated_corner = R_z @ original_corner # 4개 점의 x좌표들, y좌표들.
        # @ 연산자는 넘파이에서 matmul 또는  dot 같은 역할을 한다. 즉, 행렬-행렬간 곱 계산시 쓴다.
        # 따라서, (2x2) @ (2x4) 행렬 계산 결과는 (2x4)인 행렬이 된다. 즉, 회전된 좌표의 행렬이 된다.

        # 회전 후 (x, y)만큼 평행이동한 4개 점의 x좌표들, y좌표들은 final_corner가 된다. 
        centroid = np.array([[local_x, local_x, local_x, local_x], # 평행이동한 값들. 차량의 중심점 좌표
                             [local_y, local_y, local_y, local_y]])

        final_corner = rotated_corner + centroid

        point1 = final_corner.T[0]
        point2 = final_corner.T[1]
        point3 = final_corner.T[2]
        point4 = final_corner.T[3]
        
    else:
        point1 = None
        point2 = None
        point3 = None
        point4 = None
    
    return point1, point2, point3, point4


def NGSIM_center(local_x, local_y, velocity_x, velocity_y, v_len, v_wid):
    """리뉴얼 :: 차량의 4개 꼭지점 좌표를 반환하는 함수
    NGSIM의 경우 front_center이므로, 이를 드론자료와 달리 보정해주어야 한다.
    각 차량 포인트 추출 : corner_1x, corner_1y, ... corner_4x, corner_4y"""

    if pd.isna(velocity_x) == False:
    
        v_x = velocity_x * 1000/3600 # 차량 속도를 초속(m/s)으로 변환
        v_y = velocity_y * 1000/3600
    
        ### Rotation Matrix R_z
        
        if v_x != 0:
        
            theta = math.atan(v_y/v_x) # 속도 벡터가 x축과 이루는 각 theta를 호도법으로 구한다. 
            # math.atan은 두 점 사이 theta의 절대각을 구하는데, 탄젠트 값을 받아 절대각을 "라디안" 값으로 반환한다.
            # 이때 -pi/2 <= theta <= pi/2가 된다.
            
        else: # v_x == 0
            theta = math.pi/2 # v_x가 0이면 탄젠트값은 무한대가 되기 때문이다
        
        # 차량의 4개 꼭지점을 속도벡터에 따라 회전시키기 위한 배열
        R_z = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)],
                        ])
        # R_z = np.array([[-math.sin(theta), -math.cos(theta)],
        #                 [math.cos(theta), -math.sin(theta)],
        #                 ])        
        
        # 회전 전 4개 모서리 점 : 각 차량의 width, len에 따라 결정됨. 1사분면부터 반시계 방향으로
        original_corner = np.array([[0, -1*v_len, -1*v_len, 0], # 4개 점 x좌표
                                    [1/2*v_wid, 1/2*v_wid, -1/2*v_wid, -1/2*v_wid]]) # 4개 점 y좌표

        # 회전 후 4개 모서리의 점
        rotated_corner = R_z @ original_corner # 4개 점의 x좌표들, y좌표들.
        # @ 연산자는 넘파이에서 matmul 또는  dot 같은 역할을 한다. 즉, 행렬-행렬간 곱 계산시 쓴다.
        # 따라서, (2x2) @ (2x4) 행렬 계산 결과는 (2x4)인 행렬이 된다. 즉, 회전된 좌표의 행렬이 된다.

        # 회전 후 (x, y)만큼 평행이동한 4개 점의 x좌표들, y좌표들은 final_corner가 된다. 
        centroid = np.array([[local_x, local_x, local_x, local_x], # 평행이동한 값들. 차량의 중심점 좌표
                             [local_y, local_y, local_y, local_y]])

        final_corner = rotated_corner + centroid

        point1 = final_corner.T[0]
        point2 = final_corner.T[1]
        point3 = final_corner.T[2]
        point4 = final_corner.T[3]
        
        center_x = (point1[0] + point2[0] + point3[0] + point4[0])/4
        center_y = (point1[1] + point2[1] + point3[1] + point4[1])/4
        
    else:
        point1 = None
        point2 = None
        point3 = None
        point4 = None
        center_x = None
        center_y = None
    
    return center_x, center_y



# def NGSIM_points(local_x, local_y, velocity_x, velocity_y, v_len, v_wid):
#     """리뉴얼 :: 차량의 4개 꼭지점 좌표를 반환하는 함수
#     NGSIM의 경우 front_center이므로, 이를 드론자료와 달리 보정해주어야 한다.
#     각 차량 포인트 추출 : corner_1x, corner_1y, ... corner_4x, corner_4y"""

#     if pd.isna(velocity_x) == False:
    
#         v_x = velocity_x * 1000/3600 # 차량 속도를 초속(m/s)으로 변환
#         v_y = velocity_y * 1000/3600
    
#         ### Rotation Matrix R_z
        
#         if v_x != 0:
        
#             theta = math.atan(v_y/v_x) # 속도 벡터가 x축과 이루는 각 theta를 호도법으로 구한다. 
#             # math.atan은 두 점 사이 theta의 절대각을 구하는데, 탄젠트 값을 받아 절대각을 "라디안" 값으로 반환한다.
#             # 이때 -pi/2 <= theta <= pi/2가 된다.
            
#         else: # v_x == 0
#             theta = math.pi/2 # v_x가 0이면 탄젠트값은 무한대가 되기 때문이다
        
#         # 차량의 4개 꼭지점을 속도벡터에 따라 회전시키기 위한 배열
#         R_z = np.array([[math.cos(theta), -math.sin(theta)],
#                         [math.sin(theta), math.cos(theta)],
#                         ])
#         # R_z = np.array([[-math.sin(theta), -math.cos(theta)],
#         #                 [math.cos(theta), -math.sin(theta)],
#         #                 ])        
        
#         # 회전 전 4개 모서리 점 : 각 차량의 width, len에 따라 결정됨. 1사분면부터 반시계 방향으로
#         original_corner = np.array([[0, -1*v_len, -1*v_len, 0], # 4개 점 x좌표
#                                     [1/2*v_wid, 1/2*v_wid, -1/2*v_wid, -1/2*v_wid]]) # 4개 점 y좌표

#         # 회전 후 4개 모서리의 점
#         rotated_corner = R_z @ original_corner # 4개 점의 x좌표들, y좌표들.
#         # @ 연산자는 넘파이에서 matmul 또는  dot 같은 역할을 한다. 즉, 행렬-행렬간 곱 계산시 쓴다.
#         # 따라서, (2x2) @ (2x4) 행렬 계산 결과는 (2x4)인 행렬이 된다. 즉, 회전된 좌표의 행렬이 된다.

#         # 회전 후 (x, y)만큼 평행이동한 4개 점의 x좌표들, y좌표들은 final_corner가 된다. 
#         centroid = np.array([[local_x, local_x, local_x, local_x], # 평행이동한 값들. 차량의 중심점 좌표
#                              [local_y, local_y, local_y, local_y]])

#         final_corner = rotated_corner + centroid

#         point1 = final_corner.T[0]
#         point2 = final_corner.T[1]
#         point3 = final_corner.T[2]
#         point4 = final_corner.T[3]
        
#     else:
#         point1 = None
#         point2 = None
#         point3 = None
#         point4 = None
    
#     return point1, point2, point3, point4




def nearest_point(V_point_1, V_point_2, V_point_3, V_point_4, LV_point_1, LV_point_2, LV_point_3, LV_point_4):
    """두 차량 간 4개 점들을 비교, 가장 가까운 점들을 추출해낸다"""
    
    if type(V_point_1) is np.ndarray and type(LV_point_1) is np.ndarray:
    
        ij_list = []
        vec_list = []

        for i in range(1, 5):
            V_point = locals()[f'V_point_{i}']
            
            for j in range(1, 5):
                LV_point = locals()[f'LV_point_{j}']

                gap = V_point - LV_point
                vector = gap @ gap

                ij_list.append((i, j))
                vec_list.append(vector)

        minvec = min(vec_list)
        idx = vec_list.index(minvec) # 벡터가 최소인 경우의 (i, j) : V의 i번째 포인트, LV의 j번째 포인트가 가장 가깝다

        ij = ij_list[idx]
        
        point_n = (ij[0], ij[1]) # V, LV의 가장 가까운 점 번호

        V_point_n = locals()[f'V_point_{ij[0]}'] # 가장 가까운 점
        LV_point_n = locals()[f'LV_point_{ij[1]}'] # 가장 가까운 점
        
        ### 두번째로 가까운점 뽑기
        vec2_list = vec_list.copy()
        vec2_list.sort()
        
        minvec2 = vec2_list[1]
        idx2 = vec_list.index(minvec2)
        
        ij2 = ij_list[idx2]
        
        point_n2 = (ij2[0], ij2[1])
        
        V_point_n2 = locals()[f'V_point_{ij2[0]}'] # 두 번째로 가까운 점 - 엣지 뽑기용
        LV_point_n2 = locals()[f'LV_point_{ij2[1]}'] # 두 번째로 가까운 점 - 엣지 뽑기용

    else:
        V_point_n = None
        LV_point_n = None
        
        V_point_n2 = None
        LV_point_n2 = None
        
        point_n = None
        point_n2 = None

    return point_n, V_point_n, LV_point_n, point_n2, V_point_n2, LV_point_n2


def overlap(LV_type, point_n, point_n2, velocity_x, velocity_y, LV_velocity_x, LV_velocity_y, local_x, local_y, LV_local_x, LV_local_y, V_len, LV_len):
    """두 차량 궤적의 Overlap 여부를 알려주는 함수
    먼저 궤적이 Overlap이어야, 잠재적인 Conflict Type을 구할 수 있게 된다.
    """
    if pd.isna(velocity_x) == False and pd.isna(LV_velocity_x) == False and velocity_x != 0 and LV_velocity_x != 0 and ((velocity_y / velocity_x) - (LV_velocity_y / LV_velocity_x)) != 0: # LV가 존재하면

        t_local_x = (-(LV_velocity_y / LV_velocity_x) * LV_local_x + LV_local_y - local_y + (velocity_y / velocity_x) * local_x) / ((velocity_y / velocity_x) - (LV_velocity_y / LV_velocity_x))
        
        if (LV_type == 'LV0') and (local_x <= (LV_local_x - 1/2*LV_len)): # 같은 차로의 뒤에 있는 경우 
            return 'overlap'

        else: 
            if t_local_x > local_x: # 수렴 점이 local_x보다 큰 경우. 즉, 앞에 있는 경우
                return 'overlap'

            else:# 두 차량이 다른 차로에 있으면서, 궤적이 수렴하지 않는 경우
             return None

    else:
        return None
        

def potential_conflict_type(LV_type, overlap, point_n, point_n2, velocity_x, velocity_y, LV_velocity_x, LV_velocity_y, local_x, local_y, LV_local_x, LV_local_y, V_len, V_wid, LV_len, LV_wid):
    """V와 LV의 가장 가까운 점 번호를 바탕으로, 상충유형을 판단하기
    point_n : V와 LV의 가장 가까운 점 번호. (V, LV)
    point_n2 : V와 LV의 두번째로 가까운 점 번호. (V, LV)
    """
    
    # 만약 LV 정보가 없다면
    if pd.isna(velocity_x) == False and pd.isna(LV_velocity_x) == False and velocity_x != 0 and LV_velocity_x != 0:
    
        # 만약 차로가 같다면, 이는 Rear-end이다.
        if LV_type == 'LV0' and ((local_x + 1/2* V_len) <= (LV_local_x - 1/2 * LV_len)): #and ((abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid))):
                                 #or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (LV_local_y >= local_y)) 
                                # or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (local_y >= LV_local_y))):
            return 'rear_end', None

        # 만약 차로가 다르다면, Rear-end 외에도 다른 경우가 있을 수 있다. 이 경우, "충돌 예상 시점(tcoll)"에서의 위치관계를 바탕으로 상충유형을 구분해야만 한다.
        else: 
            #(LV_type == 'LVL') or (LV_type == 'LVR'):
            # Q. 먼저, "수렴 중인가"를 확인해야 한다. 두 차량의 궤적이 Overlap인가?
            if overlap == 'overlap':
                
                if (pd.isna(point_n) == False) and (pd.isna(point_n2) == False):
        
                    V = (point_n[0], point_n2[0]) # V의 가장 가까운 점번호, 두번쨰 가까운 점번호
                    LV = (point_n[1], point_n2[1]) # LV의 가장 가까운 점번호, 두번째 가까운 점번호
        
                    # 두 차량 속도가 이루는 벡터로부터 이루는 각을 계산 : Side-swipe와 Angled를 구분할 목적
                    V_v = (velocity_x, velocity_y)
                    LV_v = (LV_velocity_x, LV_velocity_y)
        
                    dist_V = math.sqrt(V_v[0]**2 + V_v[1]**2)
                    dist_LV = math.sqrt(LV_v[0]**2 + LV_v[1]**2)
                    
                    if dist_LV == 0 or dist_V == 0:
                        cos_theta = 1 # 영벡터와 내적을 하면 1이 된다
                        
                    else: #dist_LV != 0:
        
                        ip1 = V_v[0] * LV_v[0] + V_v[1] * LV_v[1] # 코사인의 분자
                        ip2 = dist_V * dist_LV # 코사인의 분모
        
                        cos_theta = ip1/ip2 # 두 벡터가 이루는 각 theta의 코사인 값(이때 theta는 라디안 임)
        
                    if cos_theta >= -1 and cos_theta <= 1:
                        pass
        
                    elif cos_theta < -1:
                        cos_theta = -1
        
                    elif cos_theta > 1:
                        cos_theta = 1
        
                    else:
                        pass
        
                    theta = math.acos(cos_theta) # 두 벡터가 이루는 각 theta의 라디안 값
                    
                    degX = math.degrees(theta) # theta의 육십분법 각. 도
                    
                    #if (V in [(1, 2), (2, 1), (1, 1), (2, 2)] and LV in [(3, 4), (3, 3), (4, 4), (4, 3)]) or (V in [(2, 3), (3, 2), (1, 4), (4, 1)] and LV in [(2, 3), (3, 2), (1, 4), (4, 1)]) or (V in [(4, 3), (3, 4), (3, 3), (4, 4)] and LV in [(4, 3), (3, 4), (3, 3), (4, 4)]) or (V in [(3, 4), (3, 3), (4, 4), (4, 3)] and LV in [(1, 2), (2, 1), (1, 1), (2, 2)]):
                    if (V in [(2, 3), (3, 2), (1, 4), (4, 1)]) or (LV in [(2, 3), (3, 2), (1, 4), (4, 1)]):
                        
                        if ((degX < 85) and (degX > 0)):
                            return 'side_swipe', degX #ip1, ip2, cos_theta, theta, degX, 'side_swipe'
        
                        elif degX >= 85 and (degX < 180):
                            return 'angled', degX #ip1, ip2, cos_theta,theta, degX, 'angled'
        
                        else:
                            return None, None
                            #globals()['debug'].append(['0', V, LV])
        
                    
                    ## 전방과 전방일 경우
                    elif (V in [(1, 2), (2, 1), (1, 1), (2, 2)]) and (LV in [(1, 2), (2, 1), (1, 1), (2, 2)]):
                        
                        if ((degX < 85) and (degX > 0)):
                            return 'side_swipe', degX #ip1, ip2, cos_theta, theta, degX, 'side_swipe'
        
                        elif degX >= 85 and (degX < 180):
                            return 'angled', degX #ip1, ip2, cos_theta,theta, degX, 'angled'
        
                        else:
                            return None, None
                            #globals()['debug'].append(['*', V, LV])
                            
        
                    else:
                        #globals()['debug'].append(['**', V, LV])
                        return 'rear_end', degX
        
                else:
                    #globals()['debug'].append(['***', V, LV])
                    return None, None

            else: # overlap == None. 발산하는 경우
                return None, None
                
    else:
        return None, None




def innerProduct(v1, v2):
    """두 벡터가 이루는 각의 크기를 라디안(호도법)으로 표현하기"""
    # 벡터 v1, v2의 크기 구하기
    distA = dist(v1)
    distB = dist(v2)

    # 내적 1 (x1x2 + y1y2)
    ip = v1[0] * v2[0] + v1[1] * v2[1]

    # 내적 2 (|v1|*|v2|*cos x)
    ip2 = distA * distB

    # cos x값 구하기
    cost = ip / ip2

    #print("cos x: %10.3f" % cost)
    if cost <= 1 and cost >= -1:
        # x값(라디안) 구하기 (cos 역함수)
        x = math.acos(cost)
        #print("x (radians): %10.3f" % x)
    
    elif cost > 1:
        x = 0
    
    elif cost < -1:
        x = -1 * math.pi
        
    else:
        x = None

    # x값을 x도로 변환하기
    #degX = math.degrees(x)
    #print("x (degrees): %10.3f" % degX)
    
    return x

# Edge line : LV_point_n, LV_point_n2 사이의 벡터 :: 방향에 유의

def shortest_distance(point_n, V_point_n, LV_point_n, V_point_1, V_point_2, V_point_3, V_point_4, LV_point_1, LV_point_2, LV_point_3, LV_point_4):
    """가장 가까운 꼭지점 정보를 바탕으로 최단 경로의 꼭지점 생성"""
    
    #print(point_n, V_point_n, LV_point_n) # V의 1번 점, LV의 2번 점

    if point_n:
        
        LV_point_n_num = point_n[1]
        
        LV_point_next_num_1 = LV_point_n_num%4 + 1 # 번호 1 작은 점으로
        LV_point_next_num_2 = (LV_point_n_num+1)%4 # 번호 1 큰 점으로

        if LV_point_next_num_2 == 0:
            LV_point_next_num_2 = 4
        else:
            pass

        ## 번호 1 작은 점에 대한 벡터
        LV_point_next_1 = locals()[f'LV_point_{LV_point_next_num_1}']
        LV_point_next_2 = locals()[f'LV_point_{LV_point_next_num_2}']

        edge_vector_1 = LV_point_next_1 - LV_point_n
        vector_1 = V_point_n - LV_point_n 
        # 두 벡터 사이의 각도 psi_1을 반환
        psi_1 = innerProduct(edge_vector_1, vector_1)
        #print(psi_1) # psi_1의 라디안값이 반환되었다

        edge_vector_2 = LV_point_n - LV_point_next_1
        vector_2 = V_point_n - LV_point_next_1 
        # 두 벡터 사이 각도 psi_2를 반환
        psi_2 = innerProduct(edge_vector_2, vector_2)
        #psi_2 = math.acos(cos_psi_2)

        #print(psi_1, psi_2)

        if (pd.isna(psi_1) == False) and (pd.isna(psi_2) == False) and(psi_1 > 0 and psi_1 < math.pi/2) and (psi_2 > 0 and psi_2 < math.pi/2):
            # 구하기
            #print('case 1')

            alpha = (LV_point_n[1] - LV_point_next_1[1]) / (LV_point_n[0] - LV_point_next_1[0]) # edge의 기울기
            beta_1 = LV_point_n[1] - alpha * LV_point_n[0] # LV_point_n으로부터 계산
            beta_2 = V_point_n[1] + V_point_n[0]/alpha # V_point_n으로부터 계산

            x_delta = (beta_2 - beta_1)/(alpha + (1/alpha))
            y_delta = (alpha * (beta_2 - beta_1)) / (alpha + (1/alpha))  + beta_1

            delta_point = (x_delta, y_delta)
            #print(delta_point)

            #shortest_distance = np.sqrt((V_point_n - delta_point).dot(V_point_n - delta_point))
            #print(f'shortest distance : {shortest_distance}')

        else:
            # 번호 2 작은 점에 대한 벡터
            edge_vector_3 = LV_point_next_2 - LV_point_n
            vector_3 = V_point_n - LV_point_n
            # 두 벡터 사이의 각도 psi_3을 반환
            psi_3 = innerProduct(edge_vector_3, vector_3)

            edge_vector_4 = LV_point_n - LV_point_next_2
            vector_4 = V_point_n - LV_point_next_2
            # 두 벡터 사이 각도 psi_4를 반환
            psi_4 = innerProduct(edge_vector_4, vector_4)

            #print(psi_3, psi_4)

            if (pd.isna(psi_3) == False) and (pd.isna(psi_4) == False) and (psi_3 > 0 and psi_3 < math.pi/2) and (psi_4 > 0 and psi_4 < math.pi/2):
                # 구하기
                #print('case 2')
                alpha = (LV_point_n[1] - LV_point_next_2[1]) / (LV_point_n[0] - LV_point_next_2[0]) # edge의 기울기
                beta_1 = LV_point_n[1] - alpha * LV_point_n[0]
                beta_2 = V_point_n[1] + V_point_n[0]/alpha

                x_delta = (beta_2 - beta_1)/(alpha + (1/alpha))
                y_delta = (alpha * (beta_2 - beta_1)) / (alpha + (1/alpha)) + beta_1

                delta_point = (x_delta, y_delta)
                #shortest_distance = np.sqrt((V_point_n - delta_point).dot(V_point_n - delta_point))
                #print(delta_point)
                #print(f'shortest distance : {shortest_distance}')

            else:
                #print('case 3')
                delta_point = LV_point_n
                
                #shortest_distance = np.sqrt((V_point_n - LV_point_n).dot(V_point_n - LV_point_n))
                #print(f'shortest distance : {shortest_distance}')
            
        shortest_vector = delta_point - V_point_n # V가 LV에 가까워지는 최단거리의 방향벡터
        shortest_distance = np.sqrt((shortest_vector).dot(shortest_vector))
        
    else:
        delta_point = None
        shortest_vector = None
        shortest_distance = None
        
    return delta_point, shortest_vector, shortest_distance



# |v| = (벡터의 크기) 구하기
def dist(v):
    """벡터의 크기(길이)를 구해주는 함수"""
    return math.sqrt(v[0]**2 + v[1]**2)


def cosProduct(v1, v2):
    """두 벡터 사이 각도의 코사인값을 구해주는 함수"""
    # 벡터 v1, v2의 크기 구하기
    distA = dist(v1)
    distB = dist(v2)

    # 내적 1 (x1x2 + y1y2)
    ip = v1[0] * v2[0] + v1[1] * v2[1]

    # 내적 2 (|v1|*|v2|*cos x)
    ip2 = distA * distB

    # cos x값 구하기
    cost = ip / ip2
    
    return cost # 코사인 구하기
