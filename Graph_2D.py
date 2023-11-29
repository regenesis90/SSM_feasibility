import pandas as pd
import numpy as np
import warnings
import csv
import os
import math

import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
from matplotlib.colors import ListedColormap, BoundaryNorm
import matplotlib.ticker as ticker
import matplotlib.colors as colors

from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.text import Annotation
from SSM_time_based import points


def graph_2d(df, Vehicle_ID, LV_ID, SSM, threshold):
    """
    통합함수
    """
    
    scatter_size = 12
    
    ## 공통
    sample = df[(df['Vehicle ID'] == Vehicle_ID) & (df['LV_ID'] == LV_ID)]
    
    sample = sample.sort_values(by = ['frame']).drop_duplicates(['frame'])
        
    sample = sample.reset_index(drop = True)
    sample['Time'] = (sample['Time'] - sample['Time'].min()).round(2)

    fig = plt.figure(figsize = (8, 9))

    gs = fig.add_gridspec(5, 1, height_ratios = (9, 2, 1, 1, 1), hspace = 0.8)

    ax = fig.add_subplot(gs[0, 0])
    ax_Ds = fig.add_subplot(gs[1, 0], sharex = ax)
    ax_Dx = fig.add_subplot(gs[2, 0], sharex = ax)
    ax_Dy = fig.add_subplot(gs[3, 0], sharex = ax)
    ax_Vrel = fig.add_subplot(gs[4, 0], sharex = ax)

    ## 대상차량 궤적 표시
    V_x = np.array(sample['local_x'])
    V_y = np.array(sample['local_y'])
    V_time = np.array(sample['Time'])
    V_ssm = np.array(sample[SSM])

    ## 선행차량 궤적 표시
    LV_x = np.array(sample['LV_local_x'])
    LV_y = np.array(sample['LV_local_y'])
    #LV_time = np.array(sample['Time'])
    
    
    # 시간변수 미리 설정하기
    ### 가장 두 차량 사이가 가까운 시점을 뽑기 :: 이 시점이 t == 0이 됨
    nearest = sample[sample['D'] == sample['D'].min()]    
    nearest_time = nearest['Time'].iloc[0]
    nearest_V_x = nearest['local_x'].iloc[0]
    nearest_V_y = nearest['local_y'].iloc[0]
    nearest_LV_x = nearest['LV_local_x'].iloc[0]
    nearest_LV_y = nearest['LV_local_y'].iloc[0]
    
    ## 가장 두 차량 사이가 가까웠던 점을 기준으로 V_time 평준화
    
    V_time_label = np.array((sample['Time'] - nearest_time).round(2))

    
    
    Dx = np.array(sample['D_x'].abs())
    Dy = np.array(sample['D_y'].abs()) # y는 절대값이다
    Vrel = np.array(sample['velocity_x'] - sample['LV_velocity_x'])
    
    # 백그라운드 점 그래프 그리기
    LV_sdw = ax.scatter(LV_x, LV_y, c = 'lightgray', marker = '^', s = scatter_size)
    V_sdw = ax.scatter(V_x, V_y, c = 'lightgray', marker = 'o', s = scatter_size)
    
    # 상대거리, 상대속도 그리기
    #dxnorm = colors.TwoSlopeNorm(vmin = np.nanmin([-1, np.nanmin(Dx)]), vcenter = 0, vmax = np.nanmax(Dx))

    SSM_plot = ax_Ds.plot(V_x, V_ssm, linewidth = 0.5, color = 'gray')
    Dx_plot = ax_Dx.scatter(V_x, Dx, c = Dx, cmap = 'copper', vmin = 0, vmax = np.nanmax(Dx), s = scatter_size)
    
    Dy_plot = ax_Dy.scatter(V_x, Dy, c = Dy, cmap = 'copper', vmin = 0, vmax = np.nanmax(Dy), s = scatter_size)
    
    relnorm = colors.TwoSlopeNorm(vmin = np.nanmin([-1, np.nanmin(Vrel)]), vcenter = 0, vmax = np.nanmax([1, np.nanmax(Vrel)]))
    
    Vrel_plot = ax_Vrel.scatter(V_x, Vrel, c = Vrel, cmap = 'coolwarm', norm = relnorm, s = scatter_size)

    
    # 점 그래프 그리기
    
    if len(sample[SSM].unique()) > 1:
        ## 임계값 == 0이고, 임계값보다 크면 클수록 위험한 경우
        if SSM in ['TIT', 'TIT2', 'ACTIT', 'TIDRAC', 'TIDSS', 'TERCRI', 'SSCR', 'TET', 'CI_MTTC']:

            TITnorm = colors.TwoSlopeNorm(vmin = -0.5, vcenter = threshold, vmax = np.nanmax([0.5, np.nanmax(V_ssm)]))

            LV_trj= ax.scatter(LV_x, LV_y, c = V_ssm, marker = '^', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                               cmap = 'coolwarm', label = 'Leading Vehicle', norm = TITnorm)
            V_trj = ax.scatter(V_x, V_y, c = V_ssm, marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                               cmap = 'coolwarm', label = 'Following Vehicle', norm = TITnorm)

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The higher the value, the higher the risk')
            
            # 임계값보다 큰 값들을 scatter x로 표현
            danger_V_x = np.array(sample[sample[SSM] > 0]['local_x'])
            danger_V_ssm = np.array(sample[sample[SSM] > 0][SSM])
            
            ax_Ds.scatter(V_x, V_ssm, s = 5, c = V_ssm, cmap = 'coolwarm', norm = TITnorm)
            ax_Ds.scatter(danger_V_x, danger_V_ssm, marker = 'x', c = 'red', s = 20, label = 'Danger')
            ax_Ds.axhline(y = threshold, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Threshold = {threshold}')
            
            # 최대값이 가장 위험 : SSM 값이 최대일 때 수직 점선을 그려준다.
            
            idx_MD = np.where(V_ssm == np.nanmax(V_ssm))[0][0]
            V_x_MD = V_x[idx_MD]
            LV_x_MD = LV_x[idx_MD]
            V_time_MD = V_time_label[idx_MD]
            
            ax.axvline(x = V_x_MD, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Most Dangerous : t = {V_time_MD}')
            

        ## 임계값이 존재하고, 값이 0 이상이며, 임계값보다 작을수록 위험한 경우
        elif SSM in ['TTC', 'T2', 'MTTC', 'TA', 'ACT', 'PSD', 'pPET']:
            divnorm = colors.TwoSlopeNorm(vmin = 0, vcenter = threshold, vmax = 10)

            LV_trj= ax.scatter(LV_x, LV_y, c = V_ssm, 
                       marker = '^', edgecolor = 'gray', linewidth = 0.5, s = scatter_size,
                       cmap = 'RdBu', norm = divnorm,
                       label = 'Leading Vehicle')
            V_trj = ax.scatter(V_x, V_y, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'RdBu', norm = divnorm,
                       label = 'Following Vehicle')

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The lower the value, the higher the risk')
            
            # 임계값보다 작은 들을 scatter x로 표현
            danger_V_x = np.array(sample[sample[SSM] < threshold]['local_x'])
            danger_V_ssm = np.array(sample[sample[SSM] < threshold][SSM])
            
            ax_Ds.scatter(V_x, V_ssm, s = 5, c = V_ssm, cmap = 'RdBu', norm = divnorm)
            ax_Ds.scatter(danger_V_x, danger_V_ssm, marker = 'x', c = 'red', s = 20, label = 'Danger')
            ax_Ds.axhline(y = threshold, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Threshold = {threshold}')
            
            # 최소값이 가장 위험 : SSM 값이 최소일 때 수직 점선을 그려준다.
            
            idx_MD = np.where(V_ssm == np.nanmin(V_ssm))[0][0]
            V_x_MD = V_x[idx_MD]
            LV_x_MD = LV_x[idx_MD]
            V_time_MD = V_time_label[idx_MD]
            
            ax.axvline(x = V_x_MD, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Most Dangerous : t = {V_time_MD}')
            

        ## 임계값이 있고, 바닥이 없으며, 임계값보다 작을수록 위험한 경우
        elif SSM in ['PICUD', 'DSS', 'MTC', 'MMTC']:
            divnorm = colors.TwoSlopeNorm(vmin = np.nanmin([-10, np.nanmin(V_ssm)]), vcenter = threshold, vmax = np.nanmax([10, np.nanmax(V_ssm)]))

            LV_trj= ax.scatter(LV_x, LV_y, c = V_ssm, 
                       marker = '^', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'RdBu', norm = divnorm,
                       label = 'Leading Vehicle')
            V_trj = ax.scatter(V_x, V_y, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'RdBu', norm = divnorm,
                       label = 'Following Vehicle')

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The lower the value, the higher the risk')
            
            # 임계값보다 작은 SSM들을 scatter x로 표현
            danger_V_x = np.array(sample[sample[SSM] < threshold]['local_x'])
            danger_V_ssm = np.array(sample[sample[SSM] < threshold][SSM])
            
            ax_Ds.scatter(V_x, V_ssm, s = 5, c = V_ssm, cmap = 'RdBu', norm = divnorm)
            ax_Ds.scatter(danger_V_x, danger_V_ssm, marker = 'x', c = 'red', s = 20, label = 'Danger')
            ax_Ds.axhline(y = threshold, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Threshold = {threshold}')

            # 최소값이 가장 위험 : SSM 값이 최소일 때 수직 점선을 그려준다.
            
            idx_MD = np.where(V_ssm == np.nanmin(V_ssm))[0][0]
            V_x_MD = V_x[idx_MD]
            LV_x_MD = LV_x[idx_MD]
            V_time_MD = V_time_label[idx_MD]
            
            ax.axvline(x = V_x_MD, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Most Dangerous : t = {V_time_MD}')
            
        ## 임계값이 있고, 임계값보다 클수록 위험한 경우
        elif SSM in ['DRAC', 'MDRAC', 'DCIA', 'SSCR', 'RCRI', 'ITTC']:
            divnorm = colors.TwoSlopeNorm(vmin = np.nanmin(V_ssm), vcenter = threshold, vmax = threshold + 5)

            LV_trj= ax.scatter(LV_x, LV_y, c = V_ssm, 
                       marker = '^', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'coolwarm', norm = divnorm,
                       label = 'Leading Vehicle')
            V_trj = ax.scatter(V_x, V_y, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'coolwarm', norm = divnorm,
                       label = 'Following Vehicle')

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The higher the value, the higher the risk')
            
            # 최대값이 가장 위험 : SSM 값이 최대일 때 수직 점선을 그려준다.
            
            idx_MD = np.where(V_ssm == np.nanmax(V_ssm))[0][0]
            V_x_MD = V_x[idx_MD]
            LV_x_MD = LV_x[idx_MD]
            V_time_MD = V_time_label[idx_MD]
            
            ax.axvline(x = V_x_MD, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Most Dangerous : t = {V_time_MD}')

            # 임계값보다 큰 SSM들을 scatter x로 표현
            danger_V_x = np.array(sample[sample[SSM] > threshold]['local_x'])
            danger_V_ssm = np.array(sample[sample[SSM] > threshold][SSM])
            
            ax_Ds.scatter(V_x, V_ssm, s = 5, c = V_ssm, cmap = 'coolwarm', norm = divnorm)
            ax_Ds.scatter(danger_V_x, danger_V_ssm, marker = 'x', c = 'red', s = 20, label = 'Danger')
            ax_Ds.axhline(y = threshold, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Threshold = {threshold}')

        ## 그 외, 임계값과 판정기준을 알 수 없는 경우
        else:
            LV_trj= ax.scatter(LV_x, LV_y, c = V_ssm, 
                       marker = '^', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'turbo', vmin = np.nanmin(V_ssm), vmax = np.nanmax(V_ssm),
                       label = 'Leading Vehicle')
            V_trj = ax.scatter(V_x, V_y, c = V_ssm, 
                               marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                               cmap = 'turbo', vmin = np.nanmin(V_ssm), vmax = np.nanmax(V_ssm),
                               label = 'Following Vehicle')

            ax.set_title(f'[{SSM}], Threshold : ?')
            
            # 최대값이 가장 위험? : SSM 값이 최대일 때 수직 점선을 그려준다.
            
            idx_MD = np.where(V_ssm == np.nanmax(V_ssm))[0][0]
            V_x_MD = V_x[idx_MD]
            LV_x_MD = LV_x[idx_MD]
            V_time_MD = V_time_label[idx_MD]
            
            ax.axvline(x = V_x_MD, color = 'r', linestyle = '--', linewidth = 0.5, label = f'Most Dangerous : t = {V_time_MD}')
            
            # 임계값보다 작은 SSM들을 scatter x로 표현
            
            ax_Ds.scatter(V_x, V_ssm, s = 5, c = V_ssm, cmap = 'turbo', vmin = np.nanmin(V_ssm), vmax = np.nanmax(V_ssm))

    
    # 차로변경 지점을 표시
    
    V_change = sample[sample['Lane_change'] == 'Change']
    LV_change = sample[sample['LV_Lane_change'] == 'Change']
    
    if len(V_change) > 0:
        for i in range(len(V_change)):
            V_change_x = V_change['local_x'].iloc[i]
            V_change_y = V_change['local_y'].iloc[i]
            
            ax.scatter(V_change_x, V_change_y, color = 'green', s = 120, marker = 'x')
            
            if i == 0:
                ax.annotate(f'Lane Change :\n Following Vehicle', 
                            xy = (V_change_x, V_change_y), 
                            xytext = (V_change_x-20, V_change_y+30), color = 'green',
                            arrowprops = dict(facecolor = 'green', arrowstyle = '-|>', edgecolor = 'green'))
            else:
                pass
    
    else:
        pass
    
    if len(LV_change) > 0:
        for i in range(len(LV_change)):
            LV_change_x = LV_change['local_x'].iloc[i]
            LV_change_y = LV_change['local_y'].iloc[i]
            
            ax.scatter(LV_change_x, LV_change_y, color = 'blue', s = 120, marker = 'x')
            
            if i == 0:
                ax.annotate(f'Lane Change :\n Leading Vehicle', 
                            xy = (LV_change_x, LV_change_y), 
                            xytext = (LV_change_x-20, LV_change_y-30), color = 'blue',
                            arrowprops = dict(facecolor = 'blue', arrowstyle = '-|>', edgecolor = 'blue'))
            else:
                pass
    else:
        pass
    
    
    # Annotation
    
#     ### 가장 두 차량 사이가 가까운 시점을 뽑기 :: 이 시점이 t == 0이 됨
#     nearest = sample[sample['D'] == sample['D'].min()]    
#     nearest_time = nearest['Time'].iloc[0]
#     nearest_V_x = nearest['local_x'].iloc[0]
#     nearest_V_y = nearest['local_y'].iloc[0]
#     nearest_LV_x = nearest['LV_local_x'].iloc[0]
#     nearest_LV_y = nearest['LV_local_y'].iloc[0]
    
#     ## 가장 두 차량 사이가 가까웠던 점을 기준으로 V_time 평준화
    
#     V_time_label = np.array((sample['Time'] - nearest_time).round(2))

    ## Following Vehicle Annotation
    time_min = math.ceil(V_time_label[0])
    time_max = math.floor(V_time_label[-1])
    
    ## 가장 가까운 점, t == 0일때의 점 표기
    if nearest_V_y > nearest_LV_y: # 차량 V가 위에 있음
        ax.annotate(f't=0', 
                    xy = (nearest_V_x, nearest_V_y), 
                    xytext = (nearest_V_x+5, nearest_V_y+15), color = 'red',
                    arrowprops = dict(facecolor = 'red', arrowstyle = '-|>', edgecolor = 'red'))

        ax.annotate(f't=0', 
                    xy = (nearest_LV_x, nearest_LV_y), 
                    xytext = (nearest_LV_x-5, nearest_LV_y-15), color = 'red',
                    arrowprops = dict(facecolor = 'red', arrowstyle = '-|>', edgecolor = 'red'))            

    else: # 차량 V가 아래에 있음
        ax.annotate(f't=0', 
                    xy = (nearest_LV_x, nearest_LV_y), 
                    xytext = (nearest_LV_x+5, nearest_LV_y+15), color = 'red',
                    arrowprops = dict(facecolor = 'red', arrowstyle = '-|>', edgecolor = 'red'))

        ax.annotate(f't=0', 
                    xy = (nearest_V_x, nearest_V_y), 
                    xytext = (nearest_V_x-5, nearest_V_y-15), color = 'red',
                    arrowprops = dict(facecolor = 'red', arrowstyle = '-|>', edgecolor = 'red'))         
    
    ax.plot([nearest_V_x, nearest_LV_x], [nearest_V_y, nearest_LV_y], 
            c = 'red', linewidth = 1, )
    
    ## t == 0을 중심으로 한 점들 표기
    for i in range(time_min, time_max+1):
        
        # t == i일때의 인덱스 반환
        idx = np.where(V_time_label == i)[0][0]
        
        local_y = V_y[idx]
        LV_local_y = LV_y[idx]
    
        if local_y >= LV_local_y and i != 0 : # 차량 V가 위에 있으면 V는 위에, LV는 아래에 표기
            ax.annotate(f't={i}', 
                        xy = (V_x[idx], V_y[idx]), 
                        xytext = (V_x[idx]+5, V_y[idx]+15), 
                        arrowprops = dict(facecolor = 'gray', arrowstyle = '-|>'))
        
            ax.annotate(f't={i}', 
                        xy = (LV_x[idx], LV_y[idx]), 
                        xytext = (LV_x[idx]-5, LV_y[idx]-15), 
                        arrowprops = dict(facecolor = 'gray', arrowstyle = '-|>'))
            
            ax.plot([V_x[idx], LV_x[idx]], [V_y[idx], LV_y[idx]], c = 'black', linewidth = 0.5)
            
        elif local_y < LV_local_y and i != 0: # 차량 V가 아래에 있으면 서로 바꿔 표시
            ax.annotate(f't={i}', 
                        xy = (V_x[idx], V_y[idx]), 
                        xytext = (V_x[idx]-5, V_y[idx]-15), 
                        arrowprops = dict(facecolor = 'gray', arrowstyle = '-|>'))
        
            ax.annotate(f't={i}', 
                        xy = (LV_x[idx], LV_y[idx]), 
                        xytext = (LV_x[idx]+5, LV_y[idx]+15), 
                        arrowprops = dict(facecolor = 'gray', arrowstyle = '-|>'))
            
            ax.plot([V_x[idx], LV_x[idx]], [V_y[idx], LV_y[idx]], c = 'black', linewidth = 0.5)
            
        else:
            pass
        
        ### 각 점을 잇는 선 표시
        


    # 축 및 범례 설정

    ax_Dx.set_ylim(np.nanmin([0, np.nanmin(Dx)]), np.nanmax(Dx)+1)
    ax_Dy.set_ylim(np.nanmin([0, np.nanmin(Dy)]), np.nanmax(Dy)+1)
    ax_Vrel.set_ylim(np.nanmin([0, np.nanmin(Vrel)]), np.nanmax(Vrel))

    ax.legend()
    ax_Ds.legend()
    
    ax.set_xlim(0, 250)
    ax.set_ylim((np.nanmin([np.nanmin(LV_y), np.nanmin(V_y)])-3, np.nanmax([np.nanmax(LV_y), np.nanmax(V_y)])+3))
    ax.axis('equal')
    ax.set_xlabel('local_x')
    ax.set_ylabel('local_y')


    # 컬러바 설정
    
    if len(sample[SSM].unique()) > 1:
        fig.colorbar(V_trj, ax = ax) # 범례
        fig.colorbar(V_trj, ax = ax_Ds) # 범례
        
    else:
        fig.colorbar(Dy_plot, ax = ax) # 범례
        fig.colorbar(Dy_plot, ax = ax_Ds) # 범례
    
    fig.colorbar(Dx_plot, ax = ax_Dx) # 범례
    fig.colorbar(Dy_plot, ax = ax_Dy) # 범례
    fig.colorbar(Vrel_plot, ax = ax_Vrel) # 범례


    ## Super Title, Ax Title 설정
    #fig.suptitle(f'Vehicle Pair : ({Vehicle_ID}, {LV_ID})') # 총 지도
    
    ax_Dx.set_title(f'Distance_x')
    ax_Dy.set_title(f'Distance_y')
    ax_Vrel.set_title(f'Relative Speed(m/s)')
    
    #plt.show()

    
    
    
    
    
    
    
def drawing(df, Vehicle_ID, LV_ID, SSM):
    """
    도면 그려준당
    """
    sample = df[(df['Vehicle ID'] == Vehicle_ID) & (df['LV_ID'] == LV_ID)].drop_duplicates(subset = ['Time'])

    fig = plt.figure(figsize = (10, 4))
    ax = fig.add_subplot()

    # V, LV의 각 SSM별 가장 위험한 시점 뽑기

    # 시간변수 미리 설정하기
    ### 가장 두 차량 사이가 가까운 시점을 뽑기 :: 이 시점이 t == 0이 됨
    nearest = sample[sample['D'] == sample['D'].min()].iloc[0]
    nearest_time = nearest['Time']
    nearest_V_x = nearest['local_x']
    nearest_V_y = nearest['local_y']
    nearest_LV_x = nearest['LV_local_x']
    nearest_LV_y = nearest['LV_local_y']

    nearest_V_velocity_x = nearest['velocity_x']
    nearest_V_velocity_y = nearest['velocity_y']
    nearest_LV_velocity_x = nearest['LV_velocity_x']
    nearest_LV_velocity_y = nearest['LV_velocity_y']

    ## 가장 두 차량 사이가 가까웠던 점을 기준으로 V_time 평준화
    sample['Time_label'] = sample['Time'] - nearest_time

    if len(sample[SSM].unique()) > 1: # 만약 SSM 컬럼이 NaN 외의 값을 가지고 있다면


        ####### 가장 위험한 순간 ########

        # 가장 위험한 순간의 row 뽑기

        ## 임계값이 높을수록 위험한 경우 
        if SSM in ['TIT', 'TIT2', 'ACTIT', 'TIDRAC', 'TIDSS', 'TERCRI', 'SSCR', 'TET', 'CI_MTTC', 'DRAC', 'MDRAC', 'DCIA', 'SSCR', 'RCRI', 'ITTC']:
            V_time_MD = sample[sample[SSM] == np.nanmax(sample[SSM])]['Time'].iloc[0]
            MD = sample[sample['Time'] == V_time_MD].iloc[0]
            V_time_label_MD = round(sample[sample[SSM] == np.nanmax(sample[SSM])]['Time_label'].iloc[0], 2) # 라벨용 시간은 소숫점 아래 둘째자리까지 나타낸다.

        elif SSM in ['TTC', 'T2', 'MTTC', 'TA', 'ACT', 'PSD', 'pPET', 'PICUD', 'DSS', 'MTC', 'MMTC']:
            V_time_MD = sample[sample[SSM] == np.nanmin(sample[SSM])]['Time'].iloc[0]
            MD = sample[sample['Time'] == V_time_MD].iloc[0]
            V_time_label_MD = round(sample[sample[SSM] == np.nanmin(sample[SSM])]['Time_label'].iloc[0], 2) # 라벨용 시간은 소숫점 아래 둘째자리까지 나타낸다.

        else:
            V_time_MD = sample[sample[SSM] == np.nanmax(sample[SSM])]['Time'].iloc[0]
            MD = sample[sample['Time'] == V_time_MD].iloc[0]
            V_time_label_MD = round(sample[sample[SSM] == np.nanmax(sample[SSM])]['Time_label'].iloc[0], 2) # 라벨용 시간은 소숫점 아래 둘째자리까지 나타낸다.


        SSM_MD = round(MD[SSM], 3)


        ## 차량 꼭지점 뽑기
        local_x = MD['local_x']
        local_y = MD['local_y']
        LV_local_x = MD['LV_local_x']
        LV_local_y = MD['LV_local_y']

        V_velocity_x = MD['velocity_x'] * 1000/3600
        V_velocity_y = MD['velocity_y'] * 1000/3600
        LV_velocity_x = MD['LV_velocity_x'] * 1000/3600
        LV_velocity_y = MD['LV_velocity_y'] * 1000/3600

        V_len = MD['V_len']
        V_wid = MD['V_wid']
        LV_len = MD['LV_len']
        LV_wid = MD['LV_wid']

        V_point1, V_point2, V_point3, V_point4 = points(local_x, local_y, V_velocity_x, V_velocity_y, V_len, V_wid)
        LV_point1, LV_point2, LV_point3, LV_point4 = points(LV_local_x, LV_local_y, LV_velocity_x, LV_velocity_y, LV_len, LV_wid)

        ## 차량 V, LV 그리기 : 선분 4개
        for veh in ['V', 'LV']:
            for line in [(1, 2), (2, 3), (3, 4), (4, 1)]:
                i = line[0]
                j = line[1]

                ax.plot([locals()[f'V_point{i}'][0], locals()[f'V_point{j}'][0]], 
                        [locals()[f'V_point{i}'][1], locals()[f'V_point{j}'][1]],
                       color = 'black')
                ax.plot([locals()[f'LV_point{i}'][0], locals()[f'LV_point{j}'][0]], 
                        [locals()[f'LV_point{i}'][1], locals()[f'LV_point{j}'][1]],
                       color = 'black')

        ## 차량 V, 차량 LV의 중앙지점 찍기 : 점 2개
        ax.scatter(local_x, local_y, color = 'black', label = 'Following Vehicle', s = 10)
        ax.scatter(LV_local_x, LV_local_y, color = 'black', label = 'Leading Vehicle', marker = '^', s = 10)

        # 차량 V, 차량 LV의 속도벡터 그리기 : 화살표로



        ax.annotate('',#f'Velocity = ({round(V_velocity_x, 2)}, {round(V_velocity_y, 2)}) m/s', 
                    xytext = (local_x, local_y), xy = ((local_x+V_velocity_x), (local_y+V_velocity_y)), color = 'black',
                    arrowprops = dict(arrowstyle = '->', facecolor = 'black', edgecolor = 'black', linewidth = 1),
                    ha = 'center', va = 'baseline')

        ax.annotate('',#f'Velocity = ({round(V_velocity_x, 2)}, {round(V_velocity_y, 2)}) m/s', 
                    xytext = (LV_local_x, LV_local_y), xy = ((LV_local_x+LV_velocity_x), (LV_local_y+LV_velocity_y)), color = 'black',
                    arrowprops = dict(arrowstyle = '->', facecolor = 'black', edgecolor = 'black', linewidth = 1),
                    ha = 'center', va = 'baseline')


        ## 차량 V, 차량 LV의 궤적을 점선으로 나타내기
        ax.plot(sample['local_x'], sample['local_y'], color = 'black', linestyle = '--', linewidth = 0.5, label = 'Vehicle trajectory')
        ax.plot(sample['LV_local_x'], sample['LV_local_y'], color = 'black', linestyle = '--', linewidth = 0.5)

        ## 텍스트 붙이기
        ax.text((local_x+LV_local_x)/2, np.max([local_y, LV_local_y])+6, 
                f'{SSM}:\nMost dangerous', fontsize = 10, 
                va = 'baseline', ha = 'center')

        ax.text((local_x+LV_local_x)/2, np.max([local_y, LV_local_y])-10, 
                f't={V_time_label_MD}', fontsize = 10, 
                va = 'baseline', ha = 'center')


        ####### 가장 가까운 순간 순간 :: Nearest ########

        if pd.isna(nearest_V_velocity_x) == True:
            nearest_V_velocity_x = sample.loc[sample[sample['D'] == sample['D'].min()].index[0]+1]['velocity_x'] * 1000/3600
            nearest_V_velocity_y = sample.loc[sample[sample['D'] == sample['D'].min()].index[0]+1]['velocity_y'] * 1000/3600
        else:
            pass

        if pd.isna(nearest_LV_velocity_x) == True:
            nearest_LV_velocity_x = sample.loc[sample[sample['D'] == sample['D'].min()].index[0]+1]['LV_velocity_x'] * 1000/3600
            nearest_LV_velocity_y = sample.loc[sample[sample['D'] == sample['D'].min()].index[0]+1]['LV_velocity_y'] * 1000/3600
        else:
            pass

        near_V_point1, near_V_point2, near_V_point3, near_V_point4 = points(nearest_V_x, nearest_V_y, nearest_V_velocity_x, nearest_V_velocity_y, V_len, V_wid)
        near_LV_point1, near_LV_point2, near_LV_point3, near_LV_point4 = points(nearest_LV_x, nearest_LV_y, nearest_LV_velocity_x, nearest_LV_velocity_y, LV_len, LV_wid)

        ## 차량 V, LV 그리기 : 선분 4개
        for veh in ['V', 'LV']:
            for line in [(1, 2), (2, 3), (3, 4), (4, 1)]:
                i = line[0]
                j = line[1]

                ax.plot([locals()[f'near_V_point{i}'][0], locals()[f'near_V_point{j}'][0]], 
                        [locals()[f'near_V_point{i}'][1], locals()[f'near_V_point{j}'][1]],
                       color = 'black')
                ax.plot([locals()[f'near_LV_point{i}'][0], locals()[f'near_LV_point{j}'][0]], 
                        [locals()[f'near_LV_point{i}'][1], locals()[f'near_LV_point{j}'][1]],
                       color = 'black')

        ## 차량 중앙지점 찍기
        ax.scatter(nearest_V_x, nearest_V_y, color = 'black', s = 10)
        ax.scatter(nearest_LV_x, nearest_LV_y, color = 'black', marker = '^', s = 10)

        ## 텍스트 붙이기
        ax.text((nearest_V_x+nearest_LV_x)/2, np.max([nearest_V_y, nearest_LV_y])+6, 
                f'Nearest', fontsize = 10, 
                va = 'baseline', ha = 'center')

        ax.text((nearest_V_x+nearest_LV_x)/2, np.max([nearest_V_y, nearest_LV_y])-10, 
                f't=0', fontsize = 10, 
                va = 'baseline', ha = 'center')


        # 범례 및 스케일 지정
        #ax.legend()

        ax.set_xlim(0, 250)
        ax.set_ylim((np.nanmin([np.nanmin(LV_local_y), np.nanmin(local_y)])-5, np.nanmax([np.nanmax(LV_local_y), np.nanmax(local_y)])+20))
        ax.axis('equal')
        ax.set_xlabel('local_x')
        ax.set_ylabel('local_y')

        #plt.show()

        return V_time_label_MD
    
    else:
        return None
    
    
    
    

        
        
        
        
def spacing_2d(df, Vehicle_ID, LV_ID, SSM, threshold):
    """
    relative speed, D에 따른 2차원 Scatter
    """
    
    scatter_size = 12
    
    ## 공통
    sample = df[(df['Vehicle ID'] == Vehicle_ID) & (df['LV_ID'] == LV_ID)]
    
    sample = sample.sort_values(by = ['frame']).drop_duplicates(['frame'])
        
    sample = sample.reset_index(drop = True)
    sample['Time'] = (sample['Time'] - sample['Time'].min()).round(2)

    fig = plt.figure(figsize = (8, 9))

    gs = fig.add_gridspec(1, 1)

    ax = fig.add_subplot(gs[0, 0])

    ## 상대속도(x축 방향) 표시 : 벡터의 정사영을 반영해야 함
    V_time = np.array(sample['Time'])
    V_ssm = np.array(sample[SSM])
    
    
    # 시간변수 미리 설정하기
    ### 가장 두 차량 사이가 가까운 시점을 뽑기 :: 이 시점이 t == 0이 됨
    nearest = sample[sample['D'] == sample['D'].min()]    
    nearest_time = nearest['Time'].iloc[0]
    nearest_Dx = nearest['D_x'].iloc[0]
    nearest_Vrel = nearest['velocity_x'].iloc[0] - nearest['LV_velocity_x'].iloc[0]
    
    ## 가장 두 차량 사이가 가까웠던 점을 기준으로 V_time 평준화
    
    V_time_label = np.array((sample['Time'] - nearest_time).round(2))
    
    Dx = np.array(sample['D_x'].abs())
    Dy = np.array(sample['D_y'].abs()) # y는 절대값이다
    Vrel = np.array(sample['velocity_x'] - sample['LV_velocity_x'])
    
    # 상대거리, 상대속도 그리기
    #dxnorm = colors.TwoSlopeNorm(vmin = np.nanmin([-1, np.nanmin(Dx)]), vcenter = 0, vmax = np.nanmax(Dx))
    
    # 점 그래프 그리기
    
    if len(sample[SSM].unique()) > 1:
        ## 임계값 == 0이고, 임계값보다 크면 클수록 위험한 경우
        if SSM in ['TIT', 'TIT2', 'ACTIT', 'TIDRAC', 'TIDSS', 'TERCRI', 'SSCR', 'TET', 'CI_MTTC']:

            TITnorm = colors.TwoSlopeNorm(vmin = -0.5, vcenter = threshold, vmax = np.nanmax([0.5, np.nanmax(V_ssm)]))

            trj= ax.scatter(Dx, Vrel, c = V_ssm, marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                               cmap = 'coolwarm', label = 'Leading Vehicle', norm = TITnorm)

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The higher the value, the higher the risk')
                        

        ## 임계값이 존재하고, 값이 0 이상이며, 임계값보다 작을수록 위험한 경우
        elif SSM in ['TTC', 'T2', 'MTTC', 'TA', 'ACT', 'PSD', 'pPET']:
            divnorm = colors.TwoSlopeNorm(vmin = 0, vcenter = threshold, vmax = 10)

            trj= ax.scatter(Dx, Vrel, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5, s = scatter_size,
                       cmap = 'RdBu', norm = divnorm,
                       label = 'Leading Vehicle')

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The lower the value, the higher the risk')


        ## 임계값이 있고, 바닥이 없으며, 임계값보다 작을수록 위험한 경우
        elif SSM in ['PICUD', 'DSS', 'MTC', 'MMTC']:
            divnorm = colors.TwoSlopeNorm(vmin = np.nanmin([-10, np.nanmin(V_ssm)]), vcenter = threshold, vmax = np.nanmax([10, np.nanmax(V_ssm)]))

            trj = ax.scatter(Dx, Vrel, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'RdBu', norm = divnorm,
                       label = 'Leading Vehicle')

            ax.set_title(f'[{SSM}], Threshold : {threshold}, The lower the value, the higher the risk')
                        
        ## 임계값이 있고, 임계값보다 클수록 위험한 경우
        elif SSM in ['DRAC', 'MDRAC', 'DCIA', 'SSCR', 'RCRI', 'ITTC']:
            divnorm = colors.TwoSlopeNorm(vmin = np.nanmin(V_ssm), vcenter = threshold, vmax = threshold + 5)

            trj= ax.scatter(Dx, Vrel, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'coolwarm', norm = divnorm,
                       label = 'Leading Vehicle')

        ## 그 외, 임계값과 판정기준을 알 수 없는 경우
        else:
            trj= ax.scatter(Dx, Vrel, c = V_ssm, 
                       marker = 'o', edgecolor = 'gray', linewidth = 0.5,  s = scatter_size,
                       cmap = 'turbo', vmin = np.nanmin(V_ssm), vmax = np.nanmax(V_ssm),
                       label = 'Leading Vehicle')
    
    # 차로변경 지점을 표시
    
#     V_change = sample[sample['Lane_change'] == 'Change']
#     LV_change = sample[sample['LV_Lane_change'] == 'Change']
    
#     if len(V_change) > 0:
#         for i in range(len(V_change)):
#             V_change_Vrel = (V_change['velocity_x'].iloc[i] - V_change['LV_velocity_x'].iloc[i]) * 1000/3600
#             V_change_D = V_change['D_x'].iloc[i]
            
#             ax.scatter(V_change_D, V_change_Vrel, color = 'green', s = 120, marker = 'x')
            
#             if i == 0:
#                 ax.annotate(f'Lane Change :\n Following Vehicle', 
#                             xy = (V_change_D, V_change_Vrel), 
#                             xytext = (V_change_D-5, V_change_Vrel+5), color = 'green',
#                             arrowprops = dict(facecolor = 'green', arrowstyle = '-|>', edgecolor = 'green'))
#             else:
#                 pass
    
#     else:
#         pass
    
#     if len(LV_change) > 0:
#         for i in range(len(LV_change)):
#             LV_change_Vrel = (LV_change['velocity_x'].iloc[i] - LV_change['LV_velocity_x'].iloc[i]) * 1000/3600
#             LV_change_D = LV_change['D_x'].iloc[i]
            
#             ax.scatter(LV_change_D, LV_change_Vrel, color = 'blue', s = 120, marker = 'x')
            
#             if i == 0:
#                 ax.annotate(f'Lane Change :\n Leading Vehicle', 
#                             xy = (LV_change_D, LV_change_Vrel), 
#                             xytext = (LV_change_D-5, LV_change_Vrel-5), color = 'blue',
#                             arrowprops = dict(facecolor = 'blue', arrowstyle = '-|>', edgecolor = 'blue'))
#             else:
#                 pass
#     else:
#         pass
    
    
    # Annotation
    
#     ### 가장 두 차량 사이가 가까운 시점을 뽑기 :: 이 시점이 t == 0이 됨
#     nearest = sample[sample['D'] == sample['D'].min()]    
#     nearest_time = nearest['Time'].iloc[0]
#     nearest_V_x = nearest['local_x'].iloc[0]
#     nearest_V_y = nearest['local_y'].iloc[0]
#     nearest_LV_x = nearest['LV_local_x'].iloc[0]
#     nearest_LV_y = nearest['LV_local_y'].iloc[0]
    
#     ## 가장 두 차량 사이가 가까웠던 점을 기준으로 V_time 평준화
    
#     V_time_label = np.array((sample['Time'] - nearest_time).round(2))

    ## Following Vehicle Annotation
    time_min = math.ceil(V_time_label[0])
    time_max = math.floor(V_time_label[-1])
    
    ## 가장 가까운 점, t == 0일때의 점 표기
    ax.annotate(f't=0', 
                xy = (nearest_Dx, nearest_Vrel), 
                xytext = (nearest_Dx+1, nearest_Vrel+1), color = 'red',
                arrowprops = dict(facecolor = 'red', arrowstyle = '-|>', edgecolor = 'red'))

    
    ## t == 0을 중심으로 한 점들 표기
    for i in range(time_min, time_max+1):
        
        # t == i일때의 인덱스 반환
        idx = np.where(V_time_label == i)[0][0]
    
        ax.annotate(f't={i}', 
                    xy = (Dx[idx], Vrel[idx]), 
                    xytext = (Dx[idx]+1, Vrel[idx]+1), 
                    arrowprops = dict(facecolor = 'gray', arrowstyle = '-|>'))
        


    # 축 및 범례 설정

    ax.legend()
    
    #ax.set_xlim(0, 250)
    #ax.set_ylim((np.nanmin([np.nanmin(LV_y), np.nanmin(V_y)])-3, np.nanmax([np.nanmax(LV_y), np.nanmax(V_y)])+3))
    #ax.axis('equal')
    ax.set_xlabel('Spacing(m)')
    ax.set_ylabel('Relative speed(m/s)')


    # 컬러바 설정
    
    if len(sample[SSM].unique()) > 1:
        fig.colorbar(trj, ax = ax) # 범례

    else:
        pass
    
    
    #plt.show()