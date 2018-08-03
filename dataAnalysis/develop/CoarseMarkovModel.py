
# coding: utf-8

# In[9]:

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
get_ipython().magic('matplotlib inline')
import seaborn
seaborn.set(style='ticks')


# In[5]:

#import the data

FLOAT_ERROR_TOLERANCE = 0.00000000001 #See IEEE 754 for why a floating point is never perfect
df = pd.read_csv('../data/wall-collision.csv',index_col=False)
#for index, row in df.iterrows():
#    df.at[index,'Time'] = row['Time'].replace(' ', '.')
to_drop = ['Time','ID','ResetID','checkCorrectness']
df.drop(to_drop, inplace=True, axis=1)
df = df.apply(pd.to_numeric)

# clean up data
#df['Yaw'] += np.pi

#df.columns = ['X','Y','Yaw']
df.to_csv('../data/clean-wall-collision.csv', index = False)
print(df.head())


# In[38]:

RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi

def classify(yaw,numwalls):
    disc_yaw = (int(yaw / RESOLUTION_OF_S1) * RESOLUTION_OF_S1)
    if int(numwalls) == 0:
        return "FREE"
    elif int(numwalls) == 1:
        return "WALL"
    else:
        return "CORNER"


# In[39]:

df_discretized = pd.read_csv('../data/clean-wall-collision.csv', index_col=False)

df_discretized['StateLabel'] = df_discretized.apply(lambda dat: classify(dat['Yaw'], dat['NumberOfWalls']),axis=1)
df_discretized.to_csv('../data/classified-time-series.csv', index = False)

print(df_discretized.head())


# In[48]:

states = ['Free','WALL','CORNER']

free = df_discretized.loc[df_discretized['StateLabel'] == 'FREE']
plt.xlim(-0.1,0.6)
plt.ylim(-0.1,0.6)
plt.scatter(free['X'],free['Y'],c='red',marker='o',edgecolors=None)
plt.show()

wall = df_discretized.loc[df_discretized['StateLabel'] == 'WALL']
plt.xlim(-0.1,0.6)
plt.ylim(-0.1,0.6)
plt.scatter(wall['X'],wall['Y'],c='blue',marker='o',edgecolors=None)
plt.show()

corner = df_discretized.loc[df_discretized['StateLabel'] == 'CORNER']
plt.xlim(-0.1,0.6)
plt.ylim(-0.1,0.6)
plt.scatter(corner['X'],corner['Y'],c='green',marker='o',edgecolors=None)
plt.show()


# In[ ]:




# In[ ]:



