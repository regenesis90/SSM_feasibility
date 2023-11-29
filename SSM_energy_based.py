import pandas as pd
import numpy as np
import math
import shlex
import sys

if __name__ == '__main__':
    sys.exit(main())  # next section explains the use of sys.exit

###### 에너지 기반 SSM ##########

def CIF(velocity, TTC):
    """
    velocity : 속도(km/h) ---> 함수내에서 m/s로 바꿀것임
    TTC : Time to Conflict(s)
    """
    
    if pd.isna(TTC) == False:
        
        if TTC > 0:
            v = velocity * 1000/3600
            CIF = v**2 / TTC
        
        else:
            CIF = None
        
    else: # TTC = 0이거나 없으면
        CIF = None

    return CIF