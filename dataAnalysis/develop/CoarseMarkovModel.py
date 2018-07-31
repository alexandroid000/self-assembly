
# coding: utf-8

# In[1]:

import numpy as np
import pandas as pd


# In[2]:

#import the data

FLOAT_ERROR_TOLERANCE = 0.00000000001 #See IEEE 754 for why a floating point is never perfect
df = pd.read_csv('../data/time-series.csv',index_col=False)
#for index, row in df.iterrows():
#    df.at[index,'Time'] = row['Time'].replace(' ', '.')
to_drop = ['Time','ID','ResetID','checkCorrectness']
df.drop(to_drop, inplace=True, axis=1)
df = df.apply(pd.to_numeric)

# clean up data
df['Yaw'] += np.pi

#df.columns = ['X','Y','Yaw']
df.to_csv('../data/clean-time-series.csv', index = False)
print(df.head())


# In[6]:

CORNER = 0.52 #This can be obtained from the .sdf file of the weazelball enclosure in gazebo
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi
EPSILON = 0.1

def classify(x,y,yaw):
    wall_collisions = 0
    disc_yaw = (int(yaw / RESOLUTION_OF_S1) * RESOLUTION_OF_S1)
    if ((abs(x+CORNER) < EPSILON) 
      | (abs(x-CORNER) < EPSILON) 
      | (abs(y+CORNER) < EPSILON) 
      | (abs(y-CORNER) < EPSILON)):
        wall_collisions += 1
    if wall_collisions == 0:
        return ("FREE",disc_yaw)
    elif wall_collisions == 1:
        return ("WALL", disc_yaw)
    else:
        return ("CORNER", disc_yaw)
    

classify(0,0,1)


# In[7]:

df_discretized = df.copy()

df_discretized['StateLabel'] = df_discretized.apply(lambda dat: classify(dat['X'], dat['Y'], dat['Yaw']),axis=1)
df_discretized.to_csv('../data/classified-time-series.csv', index = False)

print(df_discretized.head())


# In[ ]:



