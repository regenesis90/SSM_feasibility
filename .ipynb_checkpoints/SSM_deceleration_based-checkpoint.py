import pandas as pd
import numpy as np
import math
import shlex
import sys

if __name__ == '__main__':
    sys.exit(main())  # next section explains the use of sys.exit

###### 감속 및 거리 기반 SSM ##########

def PSD(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, V_len, LV_len, f, g, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    낮을수록 위험함
    """
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
        
        if pd.isna(LV_local_x) == False and pd.isna(LV_len) == False:

            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            v = velocity_x * 1000/3600
            LV_v = LV_velocity_x * 1000/3600

            if LV_local_x >= local_x:
                PSD = (LV_local_x - local_x - 1/2 * V_len - 1/2 * LV_len)/(v ** 2 / (2 * f * g))

            else: # V가 역전한 경우
                PSD = (local_x - LV_local_x - 1/2 * V_len - 1/2 * LV_len)/(LV_v ** 2 / (2 * f * g))

            # else: # 차로가 다를 경우
            #     PSD = None # y방향으로 발산할 경우 계산할 이유가 없다

        else:
            PSD = None
    
    else:
        PSD = None
    
    return PSD



def DSS(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, V_len, LV_len, g, f, delta_t, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    velocity : 대상차량 속도(m/s)
    LV_velocity : 선행차량 속도(m/s)
    mu : friction coefficient. 마찰계수. 보통 0.35를 쓴다
    g : 중력가속도(m/s^2). 9.81을 쓴다
    D : 선행차량과 후행차량 간 거리(m)
    delta_t : 반응시간(초). 1초 정도를 잡는다.
    
    * Okamura et al.(2011) 참조. 이사람이 TIDSS 제안하면서 DSS 계산에대해 설명했다. DSS는 일본교통학회에서 2005년에 제안한 방법론으로, 감속도-거리 기반이다.
    
    """
    
    #g = 9.81
    mu = f
    #delta_t = 1
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if pd.isna(LV_local_x) == False and pd.isna(LV_len) == False:

            distance = abs(local_x - LV_local_x) - (1/2)*LV_len - (1/2)*V_len
            v = velocity_x * 1000/3600
            LV_v = LV_velocity_x * 1000/3600

            if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

                if LV_local_x >= local_x: # 대상차량에서 선행차량을 빼줘야 한다.
                    DSS = ((v**2/(2*mu*g)) + distance) - (LV_v * delta_t + LV_v**2/(2*mu*g))

                else: # V가 역전한 경우
                    DSS = ((LV_v**2/(2*mu*g)) + distance) - (v * delta_t + v**2/(2*mu*g))

            else: # 두 차량이 발산한다면, 굳이 계산할 필요가 없다
                DSS = None

        else:
            DSS = None
    
    else:
        DSS = None
    
    return DSS # 자세히 보기!! 저 velocity, LV_velocity좀 이상해. 원전 확인해



def TIDSS(DSS, tmsec):
    """
    DSS가 임계값(0)보다 작은 부분만 적분값해서 반환함
    """
    
    if (pd.isna(DSS) == False):
        if DSS < 0:
            TIDSS = -1 * DSS * tmsec
        else:
            TIDSS = 0
        
    else:
        TIDSS = None
        
    return TIDSS



def PICUD(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, D_x, delta_t, deceleration_rate, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    PICUD(m)
    
    * Uno et al.(2002) : PICUD란, 선행차량이 긴급 제동을 걸었다고 가정했을 때, 두 대의 연속된 차량이 충돌할 가능성을 평가하는 지표다.
    PICUD는 두 차량이 완전히 정지했을 때 고려되는 두 차량 사이의 거리로 정의된다.
    -- (The distance of vehicle in front travels after emergency brake) + (distance between the vehicles) - (The distance the target vehicle travels after emergency brake) < 0 일 시 충돌(collision)이 발생함
    -- 차량 감속도 : 3.3m/s^2 
    -- 반응시간 : 1.0
    
    * [선행 연구] PICUD : 앞차가 긴급 제동을 걸었다는 가정 하에, 두 대의 차량이 연속적으로 충돌할 가능성을 평가하기 위한 새 지표로 제안되었다(Uno, N., Y. Iida, S. Yasuhara, and S. Masumi. Objective Analysis of Traffic Conflict and Modeling of Vehicular Speed Adjustment at Weaving. Journal of Infrastructure Planning, Vol. 4, No. 4, 2003, pp. 989–996 -- 일본어로 된 논문이다.). 이 연구를 수행한 연구자들은 PICUD가 두 차량 사이의 동적으로 변화하는 거리의 영향을 포착하므로, 유사한 속도를 가진 연속차량의 충돌위험 평가에 TTC보다 더 적합하다고 결론지었다.
    
    * [선행 연구] Uno et al.(2003)이 제안한 PICUD는 교통상황의 변화와 충돌은 TTC보다 더 민감하게 감지할 수 있다(20. Bin, M., N. Uno, and Y. Iida. A Study of Lane-Changing Behavior Model at Weaving Section Considering Conﬂicts. Journal of the Eastern Asia Society for Transportation Studies, Vol. 5, Oct., 2003, pp. 2039–2052.)
    
    :: Params ::
    -- velocity : 대상 차량 속도(km/h) -> m/s
    -- LV_velocity : 선행 차량 속도(km/h) -> m/s
    -- D : 대상 차량과 선행 차량 사이의 거리
    -- delta_t : 운전자의 반응 시간(Driver's reaction time) :: 기본값 1s
    -- deceleration_rate : deceleration rate to stop. 0.3G = 2.95m/s^2
    
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if LV_velocity_x and D_x:

            LV_v = LV_velocity_x * 1000/3600
            v = velocity_x * 1000/3600

            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:
                PICUD = ((LV_v**2 - v**2)/(2 * deceleration_rate)) + abs(D_x) - (v * delta_t)
                # 스즈키가 쓴 식은 좀더 정제되어 있다 : 엑셀 참조. 근데 결국 같은 말이다.
            else: # V가 역전한 경우
                PICUD = ((v**2 - LV_v**2)/(2 * deceleration_rate)) + abs(D_x) - (LV_v * delta_t)

            # else: # 두 차량이 발산한다면 굳이? 그럴 필요가 없다
            #     PICUD = None

        else:
            PICUD = None
    
    else:
        PICUD = None
    
    return PICUD


def DRAC(potential_conflict_type, LV_type, local_x, LV_local_x, D_x, velocity_x, LV_velocity_x, LV_length, V_length, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    ## 이거 length가 어느차 건지 확인해야됨
    """
    velocity : 대상차량 속도(km/h) -> m/s
    LV_velocity : 선행차량 속도(km/h) -> m/s
    
    x : 대상차량 위치
    LV_x : 선행차량 위치
    LV_length : 선행차량 길이
    
    Saccomanno et al.(2008) 참조 :  Hyden은 DRAC 값이 미리 결정된 제동 편안함 임계값 초과 시 충돌위험이 문제가 된다고 지적.
    DRAC >= 3.35m/s^2를 초과하는 DRAC 값은 안전하지 않은 상태를 반영한다.
    
    * Archer(2005)가 우월함을 역설
    * Meng & Weng(2011)이 공사구간 안전성 평가 시 DRAC를 이용하였음. Saccomanno도 참조하시오.
    
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if (pd.isna(LV_velocity_x) == False) and (pd.isna(D_x) == False) and (pd.isna(LV_length) == False):

            distance = D_x - (1/2)*(LV_length) - (1/2)*(V_length)

            v = velocity_x * 1000/3600
            LV_v = LV_velocity_x * 1000/3600


            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:
                DRAC = ((v - LV_v)**2) / (2 * distance)

            else: # V가 LV를 추월한 경우
                DRAC = ((LV_v - v)**2) / (2 * distance)

            # else: # 만약 차로가 다르고 두 차량이 발산한다면 굳이 계산할 필요 없음
            #     DRAC = None

        else:
            DRAC = None
            
    else:
        DRAC = None
    
    return DRAC


def TIDRAC(DRAC, threshold_DRAC):
    """
    임계값 이하의 DRAC 값을 적용함
    """
    
    if (pd.isna(DRAC) == False):
        if DRAC >= threshold_DRAC:
            TIDRAC = DRAC - threshold_DRAC
        else:
            TIDRAC = 0
    else:
        TIDRAC = None
        
    return TIDRAC


def MDRAC(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, TTC, R, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    속도 : km/h -> m/s로
    R : 인지반응시간. 2.5초로 잡은 논문이 있다.
    
    MDRAC의 임계값은 3.4m/s^2이다.
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if (pd.isna(LV_velocity_x) == False) and (pd.isna(TTC) == False):

            v = velocity_x * 1000/3600
            LV_v = LV_velocity_x * 1000/3600

            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:        
                if v > LV_v and TTC > R:
                    MDRAC = (v - LV_v)/(2 * (TTC - R))

                else:
                    MDRAC = None

            else: # V가 LV를 추월한 경우
                if LV_v > v and TTC > R:
                    MDRAC = (LV_v - v)/(2 * (TTC - R))

                else:
                    MDRAC = None

            # else: # 차로가 다른데 두 차량이 발산한다면 계산할 이유가 없다
            #     MDRAC = None

        else:
            MDRAC = None
            
    else:
        MDRAC = None
        
    return MDRAC


def DCIA(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, acc_x, LV_acc_x, reaction_time, D, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    -- v_10 : 차량 1(LV)의 t0 시점에서 속도
    -- d_20 : 차량 1(LV)의 t0 시점에서 가속도
    -- v_20 : 차량 2(FV)의 t0 시점에서의 속도
    -- d_20 : 차량 2(FV)의 t0 시점에서의 가속도
    -- R : reaction time
    -- D : t0 시점에서 두 차량 사이의 갭
    """
    
    R = reaction_time
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

        if LV_local_x >= local_x:

            v_10 = LV_velocity_x * 1000/3600
            d_10 = LV_acc_x
            v_20 = velocity_x * 1000/3600
            d_20 = acc_x

            if (v_10 - d_10 * R - v_20 + d_20 * R) != 0:
                T = (v_20 * R - v_10 * R - 2 * D)/(v_10 - d_10 * R - v_20 + d_20 * R)

                if T != R:
                    DCIA = (d_10 * T + v_10 - d_20 * R - v_20)/(T - R)

                else:
                    DCIA = None
                    
            else:
                DCIA = None

        else: # LV가 V에 x방향으로 추월당한 경우
            v_10 = velocity_x * 1000/3600
            d_10 = acc_x
            v_20 = LV_velocity_x * 1000/3600
            d_20 = LV_acc_x
            
            if (v_10 - d_10 * R - v_20 + d_20 * R) != 0:

                T = (v_20 * R - v_10 * R - 2 * D)/(v_10 - d_10 * R - v_20 + d_20 * R)

                if T!= R:
                    DCIA = (d_10 * T + v_10 - d_20 * R - v_20)/(T - R)
                else:
                    DCIA = None
                    
            else:
                DCIA = None

        # else: # 두 차량이 차로가 다른데 서로 발산한다면 계산할 이유가 없을 것이다
        #     DCIA = None
            
    else:
        DCIA = None
    
    return DCIA


def unsafety(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, acc_x, LV_acc_x, bmax, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    delta_speed : collision time에서의 속도 차이
    speed : 대상 차량의 속도
    Rb : unsafe parameter
    
    bmax = maximum possible deceleration rate. 
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
        
        if (pd.isna(LV_velocity_x) == False) and (pd.isna(LV_acc_x) == False):
            v = velocity_x * 1000/3600
            LV_v = LV_velocity_x * 1000/3600

            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:

                delta_v = LV_v - v
                bmax = -1 * bmax

                if LV_acc_x < 0:
                    Rb = LV_acc_x / bmax

                else:
                    Rb = 0

                unsafety = delta_v * v * Rb

            else: # V가 LV를 추월한 경우
                delta_v = v - LV_v
                bmax = -1 * bmax

                if acc_x < 0:
                    Rb = acc_x / bmax

                else:
                    Rb = 0

                unsafety = delta_v * LV_v * Rb            

            # else: # 차로가 다른데 발산하기까지 한다면 구할 필요 없음
            #     unsafety = None

        else:
            unsafety = None
            
    else:
        unsafety = None
    
    return unsafety

#def UD(local_x, local_y, LV_acc)

# def MTC(local_x, velocity_x, LV_velocity_x, g):
#     """
#     속도 : km/h -> m/s로
#     ap, af : 0.7G = 0.7 * 9.81
#     """
    
#     if pd.isna(LV_velocity_x) == False:
        
#         ap = 0.7 * g
#         af = 0.7 * g
        
#         v = velocity_x * 1000/3600
#         LV_v = LV_velocity_x * 1000/3600
        
#         MTC = (-1 * local_x - (LV_v)**2/(2 * ap))/(-1 * v**2 / (2 * af))
        
#     else:
#         MTC = None
        
#     return MTC

def MTC(potential_conflict_type, LV_type, local_x, LV_local_x, D_x, V_length, LV_length, velocity, LV_velocity, delta_t, deceleration_rate, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    MTC = (D(t) + SSD_L(t))/SSD_F_(t)
    SSD_L(t) = D(t) + V_F(t)**2/d_L_max 
    SSD_F(t) = V_f(t)*RT_F + V_F(t)**2/d_F_max
    
    * 임계값 : 1(Das et al.(2022) 참조)
    
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if pd.isna(LV_velocity) == False:
            LV_d_max = deceleration_rate
            V_d_max = deceleration_rate

            v = velocity * 1000/3600
            LV_v = LV_velocity * 1000/3600

            RT_F = delta_t # 반응시간

            distance = D_x - (1/2)*V_length - (1/2)*LV_length

            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:
                SSD_L = distance + (LV_v**2 / LV_d_max)
                SSD_F = v * RT_F + (v**2/V_d_max)

                MTC = (distance + SSD_L) / SSD_F

            else: # V가 LV를 x방향으로 추월한 경우
                SSD_L = distance + (v**2 / V_d_max)
                SSD_F = LV_v * RT_F + (LV_v**2/LV_d_max)

                MTC = (distance + SSD_L) / SSD_F

            # else: # 두 차량이 다른 차로인데 발산한다면 구할 필요 없음
            #     MTC = None

        else:
            MTC = None
    
    else:
        MTC = None
    
    return MTC


def MMTC(potential_conflict_type, LV_type, local_x, LV_local_x, velocity_x, LV_velocity_x, g, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    """
    속도 : km/h -> m/s로
    ap, af : 0.7G = 0.7 * g
    """
    
    if (potential_conflict_type == 'rear_end') and (LV_type == 'LV0'):
    
        if pd.isna(LV_velocity_x) == False:

            ap = 0.7 * g
            af = 0.7 * g

            v = velocity_x * 1000/3600
            LV_v = LV_velocity_x * 1000/3600

            #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
                # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우

            if LV_local_x >= local_x:
                D = LV_local_x - local_x
                MMTC = (D - (LV_v)**2/(2 * ap) + v**2 / (2 * af))/v

            else: # V가 LV를 x방향으로 추월한 경우
                D = local_x - LV_local_x
                MMTC = (D - (v)**2/(2 * af) + LV_v**2 / (2 * ap))/LV_v

            # else: # 차로가 다른데 발산한다면 구할 필요가 없음
            #     MMTC = None

        else:
            MMTC = None
            
    else:
        MMTC = None
        
    return MMTC

# def CPI(df, MADR, tmsec, delta_t):
#     """
    
#     DRAC : Deceleration rate to avoid the crash For FV i during time interval t(m/s^2)
#     P(x) : DRAC > MADR이면 1, 그 외에는 0
#     delta_t : observation time interval(s) == 본 연구에서는 0.1초
#     b : state variable. 만약 FV가 LV에 접근중이면(gap이 줄어드는 중이면) 1, 아닌 경우에는 0
#     T_i : Total observed time for vehicle i
    
#     * Cunto and Saccomanno(2007) 등 Saccomanno 관련 저서를 참고하시오
#     "각 차량 쌍"에 대해 얻어야 하므로, 이를 얻기 위할 시 "차량 쌍별" DF를 만들어야 한다. 
#     """
#     veh_list = list(df['Vehicle ID'].unique())
    
#     for veh in veh_list:
#         veh_df = df[df['Vehicle ID'] == veh]
#         veh_df['D_lag'] = veh_df['D'].shift(1)
#         veh_df['D_gap'] = veh_df['D'] - veh_df['D_lag']
    
    
#     DRAC, b, T_i, tmsec
    
#     delta_t = tmsec
    
#     return df
    
    # MADR = Dry pavement에서 평균 8.45m/s^2로 가정함(Cunto and Saccomanno,2008)
    
    
    
def RCRI(local_x, LV_local_x, velocity_x, LV_velocity_x, acc_x, LV_acc_x, D_x, delta_t, V_wid, LV_wid, local_y, LV_local_y, velocity_y, LV_velocity_y):
    
    
    if LV_acc_x != 0 and pd.isna(LV_acc_x) == False and pd.isna(acc_x) == False and acc_x != 0 :
        
        v = velocity_x * 1000/3600
        LV_v =  LV_velocity_x * 1000/3600
        
        #if (abs(local_y - LV_local_y) <= (1/2 * V_wid + 1/2 * LV_wid)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (LV_local_y >= local_y) and (LV_velocity_y <= velocity_y)) or ((abs(local_y - LV_local_y) >= (1/2 * V_wid + 1/2 * LV_wid)) and (local_y >= LV_local_y) and (velocity_y <= LV_velocity_y)):
            # 차로가 같을 경우, 또는 차로가 다를 때 두 차량의 벡터가 서로 수렴하는 경우
        
        if LV_local_x >= local_x:

            h = D_x/v
            d_LV = LV_v * h + LV_v**2/(2 * LV_acc_x) + D_x
            d_FV = v * delta_t + v ** 2/(2 * acc_x)

            if d_LV > d_FV:
                RCRI = 0 # Safe

            else:
                RCRI = 1

        else: # V가 LV를 추월한 경우
            h = D_x/LV_v
            d_LV = v * h + v**2/(2 * acc_x) + D_x
            d_FV = LV_v * delta_t + LV_v ** 2/(2 * LV_acc_x)

        if d_LV > d_FV:
            RCRI = 0 # Safe

        else:
            RCRI = 1
                
#         else: # 차로가 다른데 발산한다면 안구해도 된다
#             RCRI = 0
            
    else:
        RCRI = 0
    
    return RCRI


def TERCRI(RCRI, tmsec):
    
    TERCRI = np.sum(RCRI) * tmsec
    
    return TERCRI
