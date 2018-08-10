
# coding: utf-8

# In[1]:


#The point of this script is to verify the correctness of a data file


# In[1]:


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mapping import Mapping

get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figures/2018-07-22-jw-weaselball-starting_location_"
data_prefix = "../data/2018-07-22-jw-weaselball-starting_location_"


# In[2]:


FLOAT_ERROR_TOLERANCE = 0.00000000001 #See IEEE 754 for why a floating point is never perfect
df_strings = ['../data/08-10-2018_09-24-59.csv']
frames = []
for csv in df_strings:
    temp = pd.read_csv(csv,index_col=False )
    frames.append(temp)
df = pd.concat(frames,ignore_index=True)
print(df.shape)
df = df.drop(columns=['Time'])
df = df.apply(pd.to_numeric)
df.head(10)


# In[3]:


df.tail(10)


# In[4]:


#Verify that the balls hit the wall at least once
walls_column = df['NumberOfWalls']
walls = {}
for i in walls_column:
    if i in walls:
        walls[i] += 1
    else:
        walls[i] = 1
walls

