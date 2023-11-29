import pandas as pd
import numpy as np
import math
import shlex
import sys

if __name__ == '__main__':
    sys.exit(main())  # next section explains the use of sys.exit.



    
    

def TTC(potential_conflict_type, LV_type, local_x, LV_local_x, velocity, LV_velocity, V_length, LV_length, V_width, LV_width, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    -- potential_conflict_type : 잠재적 상충유형 - rear_end, head_on인 경우에만 고려하시오
    
    -- local_x : 대상차량 x좌표
    -- velocity : 대상차량 속도(km/h)
    -- V_length : 대상차량 길이(m)
    
    -- LV_local_x : 선행차량 x좌표
    -- LV_length : 선행차량 길이(m)
    -- LV_velocity : 선행차량 속도(km/h)
    
    * Hayward(1972)
    * Vogel(2003)
    * Saccomanno et al.(2008) : 차량 길이를 선행차량의 것으로 쓰고 있다. ...정확히는, 이 데이터에서는 두 차량 간 거리이므로, 중심점으로부터 각 차량의 거리의 1/2씩을 빼야 할 듯 보인다.
    
    * 주로 "같은 차로", "Rear-end"에 대해 수행한다".
    * 차로변경 차량의 경우 Target Lane에 대해 수행되어야 한다.
    * 그러나 Uno의 연구에 따르면 차로변경 차량은 더 빠른 경우가 많으므로 TTC는 계산되지 않는다.
    
    """
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if (pd.isna(LV_velocity) == False) and (pd.isna(LV_length) == False):
            
            if (abs(local_y - LV_local_y) <= (1/2 * V_width + 1/2 * LV_width)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):

                v = velocity * 1000/3600 # 시속(km/h)을 초속(m/s)으로 바꿈
                LV_v = LV_velocity * 1000/3600  # 시속(km/h)을 초속(m/s)으로 바꿈

                if LV_local_x >= local_x:

                    if v > LV_v:
                        delta_D = (LV_local_x - local_x) - ((1/2)*LV_length) - ((1/2)*V_length)

                        if delta_D > 0:
                            TTC = delta_D / (v - LV_v)

                        else:
                            TTC = None

                    else:
                        TTC = None

                else: # V가 LV를 추월한 경우
                    if LV_v > v:
                        delta_D = (local_x - LV_local_x) - ((1/2)*LV_length) - ((1/2)*V_length)

                        if delta_D > 0:
                            TTC = delta_D / (LV_v - v)

                        else:
                            TTC = None

                    else:
                        TTC = None
                # else:
                #     TTC = None # 같은 차로에 없으면 rear-end 충돌위험이 없다
            else:
                TTC = None
        else:
            TTC = None

    else: # rear_end가 아니면 TTC는 계산되지 않음
        TTC = None
        
    return TTC

def ITTC(potential_conflict_type, local_x, LV_local_x, velocity, LV_velocity, V_length, LV_length, V_width, LV_width, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    -- potential_conflict_type : 잠재적 상충유형 - rear_end, head_on인 경우에만 고려하시오
    -- D : 대상차량-선행차량 사이 거리(m)
    -- velocity : 대상차량 속도(km/h)
    -- V_length : 대상차량 길이(m)
    
    -- LV_length : 선행차량 길이(m)
    -- LV_velocity : 선행차량 속도(km/h)
    
    * Hayward(1972)
    * Vogel(2003)
    * Saccomanno et al.(2008) : 차량 길이를 선행차량의 것으로 쓰고 있다. ...정확히는, 이 데이터에서는 두 차량 간 거리이므로, 중심점으로부터 각 차량의 거리의 1/2씩을 빼야 할 듯 보인다.
    
    * 주로 "같은 차로", "Rear-end"에 대해 수행한다".
    * 차로변경 차량의 경우 Target Lane에 대해 수행되어야 한다.
    * 그러나 Uno의 연구에 따르면 차로변경 차량은 더 빠른 경우가 많으므로 TTC는 계산되지 않는다.
    
    """
    if (potential_conflict_type == 'rear_end') or (potential_conflict_type == 'head_on'):
        if (pd.isna(LV_velocity) == False) and (pd.isna(LV_length) == False):
            #if (abs(local_y - LV_local_y) <= (1/2 * V_width + 1/2 * LV_width)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):

            v = velocity * 1000/3600 # 시속(km/h)을 초속(m/s)으로 바꿈
            LV_v = LV_velocity * 1000/3600  # 시속(km/h)을 초속(m/s)으로 바꿈

            if LV_local_x >= local_x:

                if v > LV_v:
                    delta_D = (LV_local_x - local_x) - ((1/2)*LV_length) - ((1/2)*V_length)

                    if delta_D > 0:
                        TTC = (v - LV_v) / delta_D

                    else:
                        TTC = None

                else:
                    TTC = None

            else: # V가 LV를 추월한 경우
                if LV_v > v:
                    delta_D = (local_x - LV_local_x) - ((1/2)*LV_length) - ((1/2)*V_length)

                    if delta_D > 0:
                        TTC = (LV_v - v) / delta_D

                    else:
                        TTC = None

                else:
                    TTC = None

            # else: # rear-end 충돌 위험이 없음
            #     TTC = None
        else:
            TTC = None
            
    else:
        TTC = None
    
    return TTC

def MTTC(potential_conflict_type, LV_type, local_x, LV_local_x, velocity, LV_velocity, acc, LV_acc, V_length, LV_length, V_width, LV_width, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    -- potential_conflict_type : 잠재적 상충유형 - rear_end인 경우에만 고려하시오
    -- velocity : 속도(km/h)
    -- LV_velocity : 선행차량 속도(km/h)
    -- acc, LV_acc : 가속도(m/s^2)
    -- D : inital_relative_space_gap.초기 상대적 위치(m)
    
    -- V_length, LV_length : "차량 사이 거리" 계산에 필요한 두 차량의 길이
    차량 사이의 거리(Distance) : 차량 중심점 사이의 거리로부터 두 차량의 길이의 절반값을 각각 빼주어야 한다.
    
    * Ozbay et al.(2008) : [연구 한계 요약] 이 대리안전지수는 처음에 링크 기반 분석을 위해 제안되었다. 
    따라서 추가적 개선 없이 교차로 안전 평가와 같은 다른 목적으로 사용되어서는 안된다. 
    또한 이런 지수의 사용은 선형 충돌 분석(linear conflicts)으로 제한되어야 한다.
    이런 링크 기반 인덱스는 전체 네트워크 안전평가를 수행할 수 있도록 확장된다.
    
    """
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):

        if (pd.isna(LV_velocity) == False) and (pd.isna(LV_acc) == False):
            #if (abs(local_y - LV_local_y) <= (1/2 * V_width + 1/2 * LV_width)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):

            # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:
                D = LV_local_x - local_x

                distance = D - (1/2)*(V_length) - (1/2) * (LV_length)

                delta_acc = acc - LV_acc
                delta_velocity = (velocity - LV_velocity) * (1000/3600) # 시속(km/h)을 초속(m/s)으로 바꿈

                if delta_acc != 0:

                    saka = (delta_velocity**2 + 2 * delta_acc * distance)

                    if saka >= 0:

                        sq = saka ** (1/2)

                        t1 = ((-1 * delta_velocity) - sq) / delta_acc
                        t2 = ((-1 * delta_velocity) + sq) / delta_acc

                        if t1 > 0 and t2 > 0:
                            if t1 >= t2:
                                MTTC = t2

                            elif t1 < t2:
                                MTTC = t1

                            else:
                                MTTC = None

                        elif t1 > 0 and t2 <= 0:
                            MTTC = t1

                        elif t1 <= 0 and t2 > 0:
                            MTTC = t2

                        else:
                            MTTC = None

                    else : # sq , 0
                        MTTC = None

                elif delta_acc == 0 and delta_velocity > 0:
                    MTTC = distance / delta_velocity

                else:
                    MTTC = None

            else: # V가 LV를 추월한 경우
                D = local_x - LV_local_x

                distance = D - (1/2)*(V_length) - (1/2) * (LV_length)

                delta_acc = LV_acc - acc
                delta_velocity = (LV_velocity - velocity) * (1000/3600) # 시속(km/h)을 초속(m/s)으로 바꿈

                if delta_acc != 0:

                    saka = (delta_velocity**2 + 2 * delta_acc * distance)

                    if saka >= 0:

                        sq = saka ** (1/2)

                        t1 = ((-1 * delta_velocity) - sq) / delta_acc
                        t2 = ((-1 * delta_velocity) + sq) / delta_acc

                        if t1 > 0 and t2 > 0:
                            if t1 >= t2:
                                MTTC = t2

                            elif t1 < t2:
                                MTTC = t1

                            else:
                                MTTC = None

                        elif t1 > 0 and t2 <= 0:
                            MTTC = t1

                        elif t1 <= 0 and t2 > 0:
                            MTTC = t2

                        else:
                            MTTC = None

                    else : # sq , 0
                        MTTC = None

                elif delta_acc == 0 and delta_velocity > 0:
                    MTTC = distance / delta_velocity

                else:
                    MTTC = None

            # else: # 차로가 다르면서 두 차량 벡터가 발산할 경우 MTTC 계산할 필요 X
            #     MTTC = None
        else:
            MTTC = None
            
    else:
        MTTC = None
    
    return MTTC



def conflict_occurs(potential_conflict_type, LV_type, local_x, LV_local_x, velocity, LV_velocity, acc, LV_acc, V_width, LV_width, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """상충 가능 여부를 판정해주는 함수, Ozbay et al. 주장
    이건 방향을 고려하지 않음에 유의하시오
    
    potential_conflict_type : 잠재적 상충유형 - rear_end인 경우에만 고려하시오
    
    velocity : 대상차량 속도
    LV_velocity : 선행차량 속도
    acc : 대상차량 가속도
    LV_velocity : 선행차량 가속도
    
    * Ozbay et al.(2008) : [연구 한계 요약] 이 대리안전지수(MTTC)는 처음에 링크 기반 분석을 위해 제안되었다. 
    따라서 추가적 개선 없이 교차로 안전 평가와 같은 다른 목적으로 사용되어서는 안된다. 
    또한 이런 지수의 사용은 선형 충돌 분석(linear conflicts)으로 제한되어야 한다.
    이런 링크 기반 인덱스는 전체 네트워크 안전평가를 수행할 수 있도록 확장된다.
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if (pd.isna(LV_velocity) == False) and (pd.isna(LV_acc) == False):
            #if (abs(local_y - LV_local_y) <= (1/2 * V_width + 1/2 * LV_width)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_width + 1/2 * LV_width)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):

            if LV_local_x >= local_x:

                if velocity > LV_velocity: # 대상차량이 선행차량보다 빠르면
                    if acc >= 0: # 대상차량이 가속중이면
                        if LV_acc > 0:
                            return 'Possible Conflict'
                        elif LV_acc <= 0: 
                            return 'Conflict Occurs'
                        else:
                            pass
                    elif acc < 0: #대상차량이 감속중이면
                        return 'Possible Conflict'
                    else:
                        pass

                elif velocity <= LV_velocity : # 대상차량이 선행차량보다 느릴 시
                    if acc > 0:
                        if LV_acc >= 0:
                            return 'Possible Conflict'
                        elif LV_acc < 0:
                            return 'Conflict Occurs'
                        else:
                            pass

                    elif acc == 0:
                        if LV_acc >= 0:
                            return 'Imposibble Conflict'
                        elif LV_acc < 0:
                            return 'Conflict Occurs'
                        else:
                            pass

                    elif acc < 0:
                        if LV_acc >= 0:
                            return 'Inpossible Conflict'
                        elif LV_acc < 0:
                            return 'Possible Conflict'
                        else:
                            pass
                    else:
                        pass
                else:
                    pass

            else: # V가 LV를 추월한 경우
                if LV_velocity > velocity: # 대상차량이 선행차량보다 빠르면
                    if LV_acc >= 0: # 대상차량이 가속중이면
                        if acc > 0:
                            return 'Possible Conflict'
                        elif acc <= 0: 
                            return 'Conflict Occurs'
                        else:
                            pass
                    elif LV_acc < 0: #대상차량이 감속중이면
                        return 'Possible Conflict'
                    else:
                        pass

                elif LV_velocity <= velocity : # 대상차량이 선행차량보다 느릴 시
                    if LV_acc > 0:
                        if acc >= 0:
                            return 'Possible Conflict'
                        elif acc < 0:
                            return 'Conflict Occurs'
                        else:
                            pass

                    elif LV_acc == 0:
                        if acc >= 0:
                            return 'Imposibble Conflict'
                        elif acc < 0:
                            return 'Conflict Occurs'
                        else:
                            pass

                    elif LV_acc < 0:
                        if acc >= 0:
                            return 'Inpossible Conflict'
                        elif acc < 0:
                            return 'Possible Conflict'
                        else:
                            pass
                    else:
                        pass
                else:
                    pass

            # else: # rear-end 충돌위험이 없으니까
            #     pass

        else:
            pass
        
    else:
        pass
    
    

def CI_MTTC(local_x, LV_local_x, velocity, LV_velocity, acc, LV_acc, MTTC):
    """
    -- velocity : 대상 차량 속도(km/h
    -- acc : 대상 차량 가속도(m/s^2)
    
    -- LV_velocity : 선행 차량 속도(km/h)
    -- LV_acc : 선행 차량 가속도(m/s^2)
    
    MTTC : 해당 차량 페어 MTTC
    """
    
    if (pd.isna(LV_velocity) == False) and (pd.isna(LV_acc) == False) and (pd.isna(MTTC) == False):
        
        if LV_local_x >= local_x:
    
            VF = velocity * 1000/3600  # 시속(km/h)을 초속(m/s)으로 바꿈
            VL = LV_velocity * 1000/3600 # 시속(km/h)을 초속(m/s)으로 바꿈

            CI_MTTC = ((VF + acc * MTTC)**2 - (VL + LV_acc * MTTC)**2)/2 * (1/MTTC)
            
        else: # V가 LV를 추월했을 경우
            VF = LV_velocity * 1000/3600  # 시속(km/h)을 초속(m/s)으로 바꿈
            VL = velocity * 1000/3600 # 시속(km/h)을 초속(m/s)으로 바꿈

            CI_MTTC = ((VF + LV_acc * MTTC)**2 - (VL + acc * MTTC)**2)/2 * (1/MTTC)

    else:
        CI_MTTC = None

    return CI_MTTC



def CID_MTTC(df): # 손봐야 할 필요가 있어요!
    """
    df 데이터프레임으로부터 CID 계산 : 0에 가까울수록 안전한 것이다.
    
    CI_ijk : j링크 k 시점에 지나가는 i번째 차량의 Ci
    T : 분석 시간(s)
    N : 대상 네트워크를 지나간 전체 차량 대수
    lj : j번째 링크의 길이(m)
    L : 네트워크의 모든 링크 길이 합(m)
    """
    
    T = (df['Frame ID'].max() - df['Frame ID'].min())/30 # 분석 시간
    N = len(df['Vehicle ID'].unique())
    lj = df['local_x'].max()
    lane_num = len(df['Lane Identification'].unique())
    L = lane_num * lj
    
    CID = (df['CI_MTTC_0'].sum() * lj) / (T * N * L)
    
    return CID



def TIT(TTC, threshold, tmsec):
    """
    
    -- TTC : Time to Collision(s)
    -- Threshold : 일정한 값을 사용(예 : 3초)
    -- tmsec : Frame Gap(예 : 0.3s)
        
    * Minderhoud & Bovy(2001) 및 그 논문을 참조한 다양한 연구가 있음. 이를 확인하라
    * TIT_star 증분값을 합하여 "집계" 해야 완벽한 TIT_star가 됨에 유의하시오.
    
    """
    
    if (pd.isna(TTC) == False):
        if (TTC >= 0) and (TTC <= threshold):
            TIT = (threshold - TTC) * tmsec
        
        else:
            TIT = 0
    
    else:
        TIT = None
        
    return TIT



def TET(TTC, threshold, tmsec): # 보통 threshold = 3으로 지정한다.
    """
    TTC가 임계값 미만이면 0.1, 아니면 0을 반환
    위험에 노출된 시간(s)
    """
    
    if (pd.isna(TTC) == False) :
        if (TTC >= 0) and (TTC <= threshold):
            TET = tmsec
            
        else:
            TET = 0
        
    else:
        TET = None
        
    return TET




    


def T2(point_n, V_point_n, LV_point_n, 
       V_point_1, V_point_2, V_point_3, V_point_4, 
       LV_point_1, LV_point_2, LV_point_3, LV_point_4,
       V_velocity_x, V_velocity_y, LV_velocity_x, LV_velocity_y):
    """T2 계산해주는 함수"""

    t_list = []
    
    if type(V_point_n) is np.ndarray: # 만약 차량의 point가 존재한다면

        for (v1, v2) in [('V', 'LV'), ('LV', 'V')]:
            
            for a in [1, 2, 3, 4]: # 각 4개 점들에 대하여

                # (x_p, y_p) : 초기 포인트 좌표
                x_p = locals()[f'{v1}_point_{a}'][0]
                y_p = locals()[f'{v1}_point_{a}'][1]

                vx_p = locals()[f'{v1}_velocity_x'] * 1000/3600 # 속도를 m/s로 변환
                vy_p = locals()[f'{v1}_velocity_y'] * 1000/3600 

                vx_ln = locals()[f'{v2}_velocity_x'] * 1000/3600
                vy_ln = locals()[f'{v2}_velocity_y'] * 1000/3600

                for (i, j) in [(1, 2), (2, 3), (3, 4), (4, 1)]: #4개 점의 인접한 점들끼리의 조합으로, "직선"이 탄생한다.
                    # edge : sample row의 점들 결합
                    # (LV_point_1, LV_point_2), (LV_point_2, LV_point_3), (LV_point_3, LV_point_4), (LV_point_4, LV_point_1)

                    x_ln1 = locals()[f'{v2}_point_{i}'][0]
                    y_ln1 = locals()[f'{v2}_point_{i}'][1] # (x_ln1, y_ln1) :: 직선(ln)의 1번 점

                    x_ln2 = locals()[f'{v2}_point_{j}'][0]
                    y_ln2 = locals()[f'{v2}_point_{j}'][1] # (x_ln2, y_ln2) :: 직선(ln)의 2번 점

                    if x_ln2 != x_ln1: # 만약 선 ln이 수직이 아닌 경우(기울기 k가 존재할 경우)

                        k = (y_ln2 - y_ln1) / (x_ln2 - x_ln1)
                        t_coll = ((y_ln1 - y_p) - k * (x_ln1 - x_p)) / ((vy_p - vy_ln) - k * (vx_p - vx_ln))
                        # t_coll은 점 p와 선 ln의 예상 충돌 시간을 의미한다. 즉, T2값이 된다.

                    else: # 만약 선 ln이 수직이어서, 기울기 k가 존재하지 않는 경우
                        t_coll = (x_p - x_ln1) / (vx_p - vx_ln) #> 이 경우에는 T2 = TTC가 된다.
                        #print('Not K', t_coll)


                    if t_coll > 0: # 만약 t_coll > 0이면, T2값이 존재한다. 즉 2차원상에서 충돌 가능성이 존재하는 것이다.
                        x_p_coll = x_p + (vx_p * t_coll) # p점의 t_coll 시점에서 위치 좌표
                        y_p_coll = y_p + (vy_p * t_coll)

                        x_ln1_coll = x_ln1 + (vx_ln * t_coll) # ln 선 1번 점의 t_coll 시점에서 위치 좌표
                        y_ln1_coll = y_ln1 + (vy_ln * t_coll)

                        x_ln2_coll = x_ln2 + (vx_ln * t_coll) # ln 선 2번 점의 t_coll 시점에서 위치좌표
                        y_ln2_coll = y_ln2 + (vy_ln * t_coll)

                        if ((x_ln1_coll <= x_p_coll) and (x_p_coll <= x_ln2_coll)) or ((x_ln2_coll <= x_p_coll) and (x_p_coll <= x_ln1_coll)):
                            if ((y_ln1_coll <= y_p_coll) and (y_p_coll <= y_ln2_coll)) or ((y_ln2_coll <= y_p_coll) and (y_p_coll <= y_ln1_coll)):
                                t_list.append(t_coll)

                            else: # 점들이 충돌 예정 지역 내에 존재하지 않음
                                None
                            
                        else: # 점들이 충돌 예정 지역 내에 존재하지 않음
                            None

                    else:
                        None # t_coll이 0 미만인 음수일 경우 충돌할 일이 없음
    
    else: # 차량의 꼭지점 포인트가 존재하지 않음
        T2 = None
    
    if len(t_list) > 0: # T2 값이 1개 이상 존재한다면, 그 중 가장 작은 값을 취한다
        T2 = np.nanmin(t_list)
        
    else:
        T2 = None # 만약 양수인 T2 값이 한 개도 존재하지 않는다면, 없는 것이다.

    return T2



def pPET(potential_conflict_type, point_n, V_point_n, LV_point_n, 
       V_point_1, V_point_2, V_point_3, V_point_4, 
       LV_point_1, LV_point_2, LV_point_3, LV_point_4,
       V_velocity_x, V_velocity_y, LV_velocity_x, LV_velocity_y):
    """Time Advantage 계산해주는 함수"""
    
    if pd.isna(potential_conflict_type) == False:

        t_list = []
        tg_list = []

        if type(V_point_n) is np.ndarray: # 만약 차량의 point가 존재한다면

            for (v1, v2) in [('V', 'LV'), ('LV', 'V')]:

                for a in [1, 2, 3, 4]: # 각 4개 점들에 대하여

                    # (x_p, y_p) : 초기 포인트 좌표
                    x_p = locals()[f'{v1}_point_{a}'][0] # 대상차량의 꼭지점 x좌표
                    y_p = locals()[f'{v1}_point_{a}'][1] # 대상차량의 꼭지점 y좌표

                    vx_p = locals()[f'{v1}_velocity_x'] * 1000/3600 # 속도를 m/s로 변환
                    vy_p = locals()[f'{v1}_velocity_y'] * 1000/3600 

                    vx_ln = locals()[f'{v2}_velocity_x'] * 1000/3600
                    vy_ln = locals()[f'{v2}_velocity_y'] * 1000/3600

                    for (i, j) in [(1, 2), (2, 3), (3, 4), (4, 1)]: #4개 점의 인접한 점들끼리의 조합으로, "직선"이 탄생한다.
                        # edge : sample row의 점들 결합
                        # (LV_point_1, LV_point_2), (LV_point_2, LV_point_3), (LV_point_3, LV_point_4), (LV_point_4, LV_point_1)

                        x_ln1 = locals()[f'{v2}_point_{i}'][0]
                        y_ln1 = locals()[f'{v2}_point_{i}'][1] # (x_ln1, y_ln1) :: 직선(ln)의 1번 점

                        x_ln2 = locals()[f'{v2}_point_{j}'][0]
                        y_ln2 = locals()[f'{v2}_point_{j}'][1] # (x_ln2, y_ln2) :: 직선(ln)의 2번 점

                        if x_ln2 != x_ln1: # 만약 선 ln이 수직이 아닌 경우(기울기 k가 존재할 경우)

                            k = (y_ln2 - y_ln1) / (x_ln2 - x_ln1)
                            t_coll = ((y_ln1 - y_p) - k * (x_ln1 - x_p)) / ((vy_p - vy_ln) - k * (vx_p - vx_ln))
                            # t_coll은 점 p와 선 ln의 예상 충돌 시간을 의미한다. 즉, T2값이 된다.

                        else: # 만약 선 ln이 수직이어서, 기울기 k가 존재하지 않는 경우
                            t_coll = (x_p - x_ln1) / (vx_p - vx_ln) #> 이 경우에는 T2 = TTC가 된다.
                            

                        if t_coll > 0: # 만약 t_coll > 0이면, T2값이 존재한다. 즉 2차원상에서 충돌 가능

                            x_inter_1 = ((vy_ln/vx_ln * x_ln1) - (vy_p/vx_p * x_p) + (y_p - y_ln1))/((vy_ln/vx_ln) - (vy_p/vx_p))
                            y_inter_1 = (vy_p/vx_p) * (x_inter_1 - x_p) + y_p

                            Sp1 = ((x_p - x_inter_1)**2 + (y_p - y_inter_1)**2)**(1/2)
                            Sln1 = ((x_ln1 - x_inter_1)**2 + (y_ln1 - y_inter_1)**2)**(1/2)

                            v_p = (vx_p**2 + vy_p**2) ** (1/2)
                            v_ln = (vx_ln**2 + vy_ln**2) ** (1/2)

                            t1 = abs(Sp1/v_p - Sln1/v_ln)
                            t21 = np.nanmax([Sp1/v_p, Sln1/v_ln])# Time Gap 계산 위한 것

                            # Sln2, (xp, yp), (xln2, yln2) 사이의 교점까지 거리 및 걸리는 시간
                            x_inter_2 = ((vy_ln/vx_ln * x_ln2) - (vy_p/vx_p * x_p) + (y_p - y_ln2))/((vy_ln/vx_ln) - (vy_p/vx_p))
                            y_inter_2 = (vy_p/vx_p) * (x_inter_2 - x_p) + y_p

                            Sp2 = ((x_p - x_inter_2)**2 + (y_p - y_inter_2)**2)**(1/2)
                            Sln2 = ((x_ln2 - x_inter_2)**2 + (y_ln2 - y_inter_2)**2)**(1/2)

                            t2 = abs(Sp2/v_p - Sln2/v_ln)
                            t22 = np.nanmax([Sp2/v_p, Sln2/v_ln])

                            t_list.append(t1)
                            t_list.append(t2)
                            
                            timegap = np.nanmin([t22, t21])
                            
                            tg_list.append(timegap)

                        else:
                            pass # t_coll이 0 미만인 음수일 경우 충돌할 일이 없음


        else: # 차량의 꼭지점 포인트가 존재하지 않는 경우
            pass

        if len(t_list) > 0: # T2 값이 1개 이상 존재한다면, 그 중 가장 작은 값을 취한다
            PET = np.nanmin(t_list)

        else:
            PET = None # 만약 T2 값이 한 개도 존재하지 않는다면, 없는 것이다.
            
        if len(tg_list) > 0:
            TG = np.nanmin(tg_list)
            
        else:
            TG = None
            
    else:
        PET = None
        TG = None

    return PET, TG




def TA(df):
    veh_list = list(df['Vehicle ID'].unique())
    
    for veh in veh_list:
        veh_df = df[df['Vehicle ID'] == veh]
        initial_velocity = veh_df['velocity'].iloc[0] * 1000/3600
        mean_velocity = veh_df['velocity'].mean() * 1000/3600
        
        TA = 1.5 * initial_velocity / (1.67 * np.exp(-0.0306 * 0.5 * mean_velocity))
        
        df.loc[df['Vehicle ID'] == veh, 'TA'] = TA
        
    return df    


def TA_CS(TA, critical_speed):
    if pd.isna(TA) == False and pd.isna(critical_speed) == False and critical_speed != 0:
        TA_CS = TA/critical_speed
        
    else:
        TA_CS = None
        
    return TA_CS

# def TA(initial_velocity, mean_velocity):
#     """
#     -- initial_velocity : 초기 속도
#     -- mean_velocity : 평균 속도
#     """
    
#     TA = 1.5 * initial_velocity / (1.67 * np.exp**(-0.0306 * 0.5 * mean_velocity))
    
#     return TA


def critical_speed(g, f, PET):
    """
    g : 중력가속도
    f : 마찰계수(friction coefficient)
    PET : PET
    """
    
    critical_speed = 2 * g * f * PET
    
    return critical_speed


def SSCR_TTC_prime(conflict_type, point_n, V_point_n, LV_point_n, 
       V_point_1, V_point_2, V_point_3, V_point_4, 
       LV_point_1, LV_point_2, LV_point_3, LV_point_4,
       V_velocity_x, V_velocity_y, LV_velocity_x, LV_velocity_y):
    """SSCR에 필요한 TTC Prime 계산해주는 함수"""
    
    if (conflict_type in ['rear_end', 'side_swipe', 'angled']) == True:
        
        TTC_list = []

        if pd.isna(V_point_n[0]) == False: # 만약 차량의 point가 존재한다면

            for (v1, v2) in [('V', 'LV'), ('LV', 'V')]:

                for a in [1, 2, 3, 4]: # 각 4개 점들에 대하여

                    # (x_p, y_p) : 초기 포인트 좌표
                    x_p = locals()[f'{v1}_point_{a}'][0]
                    y_p = locals()[f'{v1}_point_{a}'][1]

                    vx_p = locals()[f'{v1}_velocity_x'] * 1000/3600 # 속도를 m/s로 변환
                    vy_p = locals()[f'{v1}_velocity_y'] * 1000/3600 

                    vx_ln = locals()[f'{v2}_velocity_x'] * 1000/3600
                    vy_ln = locals()[f'{v2}_velocity_y'] * 1000/3600

                    for (i, j) in [(1, 2), (2, 3), (3, 4), (4, 1)]: #4개 점의 인접한 점들끼리의 조합으로, "직선"이 탄생한다.
                        # edge : sample row의 점들 결합
                        # (LV_point_1, LV_point_2), (LV_point_2, LV_point_3), (LV_point_3, LV_point_4), (LV_point_4, LV_point_1)

                        x_ln1 = locals()[f'{v2}_point_{i}'][0]
                        y_ln1 = locals()[f'{v2}_point_{i}'][1] # (x_ln1, y_ln1) :: 직선(ln)의 1번 점

                        x_ln2 = locals()[f'{v2}_point_{j}'][0]
                        y_ln2 = locals()[f'{v2}_point_{j}'][1] # (x_ln2, y_ln2) :: 직선(ln)의 2번 점

                        colpoint_x_ln1 = (((vy_p/vx_p) * x_p) - ((vy_ln/vx_ln) * x_ln1) + (y_ln1 - y_p)) / ((vy_p/vx_p) - (vy_ln/vx_ln))
                        colpoint_x_ln2 = (((vy_p/vx_p) * x_p) - ((vy_ln/vx_ln) * x_ln2) + (y_ln2 - y_p)) / ((vy_p/vx_p) - (vy_ln/vx_ln))

                        t_xp_1 = (colpoint_x_ln1 - x_p)/vx_p 
                        t_xp_2 = (colpoint_x_ln2 - x_p)/vx_p 
                        t_xln1 = (colpoint_x_ln1 - x_p)/vx_ln 
                        t_xln2 = (colpoint_x_ln2 - x_p)/vx_ln
                        
                        if t_xp_1 > 0:
                            TTC_list.append(t_xp_1)
                            
                        else:
                            pass
                        
                        if t_xp_2 > 0:
                            TTC_list.append(t_xp_2)
                            
                        else:
                            pass
                        
                        if t_xln1 > 0:
                            TTC_list.append(t_xln1)
                            
                        else:
                            pass
                        
                        if t_xln2 > 0:
                            TTC_list.append(t_xln2)
                        
                        else:
                            pass

        else: # 차량의 꼭지점 포인트가 존재하지 않음
            pass
        
        if len(TTC_list) > 0: # 값이 1개 이상 존재한다면, 그 중 가장 작은 값을 취한다
            TTC_prime = np.nanmin(TTC_list)

        else:
            TTC_prime = None # 만약 PET 값이 한 개도 존재하지 않는다면, 없는 것이다.
    
    else:
        TTC_prime = None
    
    return TTC_prime



def SSCR_R(TTC):
    """
    * Behbahani et al.(2015) : SideSwipe Collision Risk
    - 2차원 PET 구하는 식을 참조해야 함
    
    -- TTC : Time-to-collision
    -- PET : Post Encroachment Time
    """
    
    if pd.isna(TTC) == False and TTC >= 0 and TTC < 5:
        SSCR_R = np.exp(-1 * TTC)
        
    else:
        SSCR_R = 0

    return SSCR_R


def SSCR_Y(TTC, PET, TTC_prime):
    """
    * Behbahani et al.(2015) : SideSwipe Collision Risk
    - 2차원 PET 구하는 식을 참조해야 함
    
    -- TTC : Time-to-collision
    -- PET : Post Encroachment Time
    """

    if pd.isna(TTC) == True and pd.isna(PET) == False:
        
        if TTC_prime >= 0 and TTC_prime < 5 and pd.isna(PET) == False:
            SSCR_Y = np.exp(-1 * TTC_prime) / np.exp(PET)
        
        else:
            SSCR_Y = 0
            
    else:
        SSCR_Y = 0

    return SSCR_Y


# def RECP():
#     return pass


#######################################################################################





# |v| = (벡터의 크기) 구하기
def dist(v):
    """벡터의 크기(길이)를 구해주는 함수"""
    return math.sqrt(v[0]**2 + v[1]**2)


def cosProduct(v1, v2):
    """두 벡터 사이 각도의 코사인값을 구해주는 함수"""
    # 벡터 v1, v2의 크기 구하기
    distA = dist(v1)
    distB = dist(v2)
    ip = v1[0] * v2[0] + v1[1] * v2[1]

    # 내적 2 (|v1|*|v2|*cos x)
    ip2 = distA * distB

    # cos x값 구하기l
    cost = ip / ip2
    
    return cost 

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



# def ACT(velocity_x, velocity_y, velocity_x_before, velocity_y_before, 
#         LV_velocity_x, LV_velocity_y, LV_velocity_x_before, LV_velocity_y_before, 
#         V_acc_x, V_acc_y, LV_acc_x, LV_acc_y, shortest_vector, shortest_distance):
#     """ACT 계산해주는 함수"""
    
#     if pd.isna(shortest_distance) == False and pd.isna(velocity_x)  == False and pd.isna(velocity_x_before) == False and pd.isna(LV_velocity_x) == False and pd.isna(LV_velocity_x_before) == False: # 최단거리가 산정되어 있을 시

#         V_vx = velocity_x * 1000/3600 #> 속도를 초속(m/s)으로 변환
#         V_vy = velocity_y * 1000/3600

#         V_vx_before = velocity_x_before * 1000/3600
#         V_vy_before = velocity_y_before * 1000/3600

#         V_v = np.array([V_vx, V_vy]) # 대상 차량의 속도 벡터
#         V_v_before = np.array([velocity_x_before, velocity_y_before]) # 대상 차량의 직전 시점 속도 벡터
#         V_acc = np.array([V_acc_x, V_acc_y])

#         V_v_prj = ((V_v).dot(shortest_vector) / (shortest_vector).dot(shortest_vector)) * shortest_vector # 대상차량 속도벡터의 최단거리로의 정사영 벡터 
#         V_acc_prj = ((V_acc).dot(shortest_vector) / (shortest_vector).dot(shortest_vector)) * shortest_vector # 대상차량 가속도 벡터의 최단거리로 정사영 벡터

#         cos_V_theta = cosProduct(V_v, shortest_vector) # 대상차량 속도벡터와 최단거리 벡터 간 이루는 각도 theta의 코사인 값
        
#         if cos_V_theta <= 1 and cos_V_theta >= -1:
#             V_theta = math.acos(cos_V_theta) # 이때, V_theta는 0~ pi 사이의 값으로 나타난다.
            
#         elif cos_V_theta < -1:
#             V_theta = math.pi # 
             
#         else: # 
#             V_theta = 0
            
#         V_theta_before = innerProduct(V_v_before, shortest_vector) # 호도법으로 나타낸 직전각
        
#         if (pd.isna(V_theta_before) == True):
#             V_theta_before = 0
            
#         else:
#             pass
            
#         if (pd.isna(V_theta) == True):
#             V_theta = 0
        
#         else:
#             pass

#         V_theta_ratio = (V_theta - V_theta_before)/(1/10) # 각속도(초당 각 변화율. rad/s) # 0.1초로 바꿔야 한다 23/11/28
        

#         # LV의 V에 대한 상대속도, 상대가속도, 각속도 구하기 : 최단거리 벡터 방향으로
        
#         LV_vx = LV_velocity_x * 1000/3600
#         LV_vy = LV_velocity_y * 1000/3600

#         LV_vx_before = LV_velocity_x_before * 1000/3600
#         LV_vy_before = LV_velocity_y_before * 1000/3600

#         LV_v = np.array([LV_vx, LV_vy])
#         LV_v_before = np.array([LV_velocity_x_before, LV_velocity_y_before])

#         LV_v_prj = ((LV_v).dot(shortest_vector) / (shortest_vector).dot(shortest_vector)) * (shortest_vector)

#         LV_acc = np.array([LV_acc_x, LV_acc_y])
#         LV_acc_prj = ((LV_acc).dot(shortest_vector) / (shortest_vector).dot(shortest_vector)) * (shortest_vector)
        
#         cos_LV_theta = cosProduct(LV_v, shortest_vector)

#         if cos_LV_theta > 1:
#             LV_theta = 0
            
#         elif cos_LV_theta < -1:
#             LV_theta = math.pi
            
#         else:
#             LV_theta = math.acos(cos_LV_theta)
            
#         LV_theta_before = innerProduct(LV_v_before, shortest_vector) # 호도법으로 나타낸 직전각
        
#         if (pd.isna(LV_theta_before) == True):
#             LV_theta_before = 0
            
#         else:
#             pass
            
#         if (pd.isna(LV_theta) == True):
#             LV_theta = 0
        
#         else:
#             pass

#         LV_theta_ratio = (LV_theta - LV_theta_before)/(0.1) # 0.1초로 바꿔야 한다 23/11/28
        
#         # ddelta/dt 구하기

#         V_rel = V_v_prj + LV_v_prj # 0.2초 후 속도의 정사영 값
#         acc_rel = (V_acc_prj + LV_acc_prj) * (1/10) # 0.1초로 바꿔야 한다 23/11/28
#         angle_rel = (V_theta_ratio + LV_theta_ratio) * shortest_distance
        
#         total_rel = V_rel + acc_rel + angle_rel
        
#         d_delta = dist(total_rel)
        
#         if d_delta != 0:
#             ACT = shortest_distance/d_delta
        
#         else:
#             ACT = None
            
#     else:
#         ACT = None
#         V_v_prj, LV_v_prj = None, None
#         V_acc_prj, LV_acc_prj = None, None
#         V_theta_ratio, LV_theta_ratio = None, None

#     return ACT, V_v_prj, LV_v_prj, V_acc_prj, LV_acc_prj, V_theta_ratio, LV_theta_ratio


def ACT(velocity_x, velocity_y, velocity_x_before, velocity_y_before, 
        LV_velocity_x, LV_velocity_y, LV_velocity_x_before, LV_velocity_y_before, 
        V_acc_x, V_acc_y, LV_acc_x, LV_acc_y, shortest_vector, shortest_distance):
    """ACT 계산해주는 함수"""
    
    if pd.isna(shortest_distance) == False and pd.isna(velocity_x)  == False and pd.isna(velocity_x_before) == False and pd.isna(LV_velocity_x) == False and pd.isna(LV_velocity_x_before) == False: # 최단거리가 산정되어 있을 시

        V_vx = velocity_x * 1000/3600 #> 속도를 초속(m/s)으로 변환
        V_vy = velocity_y * 1000/3600

        V_vx_before = velocity_x_before * 1000/3600
        V_vy_before = velocity_y_before * 1000/3600

        V_v = np.array([V_vx, V_vy]) # 대상 차량의 속도 벡터
        V_v_before = np.array([V_vx_before, V_vy_before]) # 대상 차량의 직전 시점 속도 벡터
        V_acc = np.array([V_acc_x, V_acc_y])

        V_v_prj = ((V_v).dot(shortest_vector) / (shortest_vector).dot(shortest_vector)) * shortest_vector # 대상차량 속도벡터의 최단거리로의 정사영 벡터 
        V_acc_prj = ((V_acc).dot(shortest_vector) / (shortest_vector).dot(shortest_vector)) * shortest_vector # 대상차량 가속도 벡터의 최단거리로 정사영 벡터

        cos_V_theta = cosProduct(V_v, shortest_vector) # 대상차량 속도벡터와 최단거리 벡터 간 이루는 각도 theta의 코사인 값
        
        if cos_V_theta <= 1 and cos_V_theta >= -1:
            V_theta = math.acos(cos_V_theta) # 이때, V_theta는 0~ pi 사이의 값으로 나타난다.
            
        elif cos_V_theta < -1:
            V_theta = math.pi # 
             
        else: # 
            V_theta = 0
            
        V_theta_before = innerProduct(V_v_before, shortest_vector) # 호도법으로 나타낸 직전각
        
        if (pd.isna(V_theta_before) == True):
            V_theta_before = 0
            
        else:
            pass
            
        if (pd.isna(V_theta) == True):
            V_theta = 0
        
        else:
            pass

        V_theta_ratio = (V_theta - V_theta_before)/(1/10) # 각속도(초당 각 변화율. rad/s) # 0.1초로 바꿔야 한다 23/11/28
        

        # LV의 V에 대한 상대속도, 상대가속도, 각속도 구하기 : 최단거리 벡터 방향으로
        shortest_vector_LV = -1 * shortest_vector
        
        LV_vx = LV_velocity_x * 1000/3600
        LV_vy = LV_velocity_y * 1000/3600

        LV_vx_before = LV_velocity_x_before * 1000/3600
        LV_vy_before = LV_velocity_y_before * 1000/3600

        LV_v = np.array([LV_vx, LV_vy])
        LV_v_before = np.array([LV_vx_before, LV_vy_before])

        LV_v_prj = ((LV_v).dot(shortest_vector_LV) / (shortest_vector).dot(shortest_vector_LV)) * (shortest_vector_LV)

        LV_acc = np.array([LV_acc_x, LV_acc_y])
        LV_acc_prj = ((LV_acc).dot(shortest_vector_LV) / (shortest_vector_LV).dot(shortest_vector_LV)) * (shortest_vector_LV)
        
        cos_LV_theta = cosProduct(LV_v, shortest_vector_LV)

        if cos_LV_theta > 1:
            LV_theta = 0
            
        elif cos_LV_theta < -1:
            LV_theta = math.pi
            
        else:
            LV_theta = math.acos(cos_LV_theta)
            
        LV_theta_before = innerProduct(LV_v_before, shortest_vector_LV) # 호도법으로 나타낸 직전각
        
        if (pd.isna(LV_theta_before) == True):
            LV_theta_before = 0
            
        else:
            pass
            
        if (pd.isna(LV_theta) == True):
            LV_theta = 0
        
        else:
            pass

        LV_theta_ratio = (LV_theta - LV_theta_before)/(0.1) # 0.1초로 바꿔야 한다 23/11/28
        
        # ddelta/dt 구하기

        V_instance = V_v_prj + V_acc_prj * (1/10)
        LV_instance = LV_v_prj + LV_acc_prj * (1/10)
        
        angle_rel = (V_theta_ratio + LV_theta_ratio) * shortest_distance
        
        total_rel = V_instance + LV_instance + angle_rel

        if total_rel[0] > 0 and total_rel[1] > 0:
            
            d_delta = dist(total_rel)
            ACT = shortest_distance/d_delta

        # elif total_rel[0] >= 0 and total_rel[1] < 0:
        #     d_delta = dist(total_rel)
        #     ACT = 999

        # elif total_rel[0] < 0 and total_rel[1] >= 0:
        #     d_delta = dist(total_rel)
        #     ACT = 888
        
        else:
            ACT = None
            d_delta = None
            
    else:
        ACT = None
        total_rel = None
        d_delta = None

    return ACT, total_rel, d_delta