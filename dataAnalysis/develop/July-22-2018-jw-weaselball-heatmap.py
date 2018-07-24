
# coding: utf-8

# In[66]:


#!/usr/bin/env python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figures/2018-07-22-jw-weaselball-heatmap_"
data_prefix = "../data/2018-07-22-jw-weaselball-heatmap_"


# In[67]:


df = pd.read_csv('../data/2018-07-22-jw-weaselball_analysis_translation_matrix_out.csv')
print(df.head())


# In[68]:


NUMBER_OF_SQUARES = 100 #This should be a square number to create equal sized squares.
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi

X_MAX = NUMBER_OF_SQUARES ** (1/2.0)
Y_MAX = NUMBER_OF_SQUARES ** (1/2.0)
YAW_MAX = (2 * np.pi) / RESOLUTION_OF_S1

def map1Dto3D(element):
    z = int(element / (X_MAX * Y_MAX));
    element -= int(z * X_MAX * Y_MAX)
    y = int(element / X_MAX)
    x = int(element % X_MAX)
    return ( x, y, z );
           
def map3Dto1D(x,y,yaw):
    return int((yaw*X_MAX*Y_MAX) + (y*X_MAX) + x)


# In[69]:


print(map1Dto3D(map3Dto1D(5,5,2)))


# In[70]:


df_regularized = df.copy()
for index, row in df_regularized.iterrows():
    multiplyer = 1.0/row.min() 
    df_regularized.iloc[index] *= multiplyer
print(df_regularized.head())


# In[71]:


#Create heatmap of where ball is
size_of_rotation_series = int((2*np.pi)/RESOLUTION_OF_S1)
heatmap_rotation = pd.Series(0, index=range(size_of_rotation_series + 1))
heatmap = pd.DataFrame(0, index=range(10), columns=range(10))
heatmap_index = range(10)

for index, row in df_regularized.iterrows():
    coordinates_in_3d = map1Dto3D(index)
    total_states = row.sum()
    heatmap.at[coordinates_in_3d[0],coordinates_in_3d[1]] += total_states
    
    heatmap_rotation.at[coordinates_in_3d[2]] += total_states

print(heatmap.head())
print(heatmap_rotation.head())


# In[72]:


#I am not sure if this is neccessary. This tries to take out the +1 given to all events.
#Get the position heatmap have the lowest at 0
import sys
heatmap_without_n = heatmap.copy()

minimum = sys.maxint
for index, row in heatmap.iterrows():
    minimum = min(minimum, row.min())
    
print(minimum)
for index, row in heatmap.iterrows():
    heatmap_without_n.iloc[index] -= minimum
print(heatmap_without_n.head())


# In[73]:


heatmap_rotation_without_n = heatmap_rotation.copy()
minimum = heatmap_rotation_without_n.min()
heatmap_rotation_without_n -= minimum
print(heatmap_rotation_without_n.head())


# In[74]:


#Create graphic heatmap of position with n taken out
#Take the log of everything so it can be graphed, add 1 to get rid of log(0)
for index, row in heatmap_without_n.iterrows():
    heatmap_without_n.iloc[index] += 1
    heatmap_without_n.iloc[index] = np.log(heatmap_without_n.iloc[index])
heatmap_position = sns.heatmap(heatmap_without_n, annot=False).set_title('Log Frequence of Weaselball Structure in (x,y) with n taken out')
plot = heatmap_position.get_figure()
plot.savefig(fig_prefix + "position_heatmap_without_n.png")
print(heatmap_without_n)


# In[75]:


#Create graphic heatmap of position without n taken out
#Take the log of everything so it can be graphed, add 1 to get rid of log(0)
for index, row in heatmap.iterrows():
    heatmap.iloc[index] += 1
    heatmap.iloc[index] = np.log(heatmap.iloc[index])
heatmap_position = sns.heatmap(heatmap, annot=False).set_title('Log Frequence of Weaselball Structure in (x,y)')
plot = heatmap_position.get_figure()
plot.savefig(fig_prefix + "position_heatmap.png")
print(heatmap)


# In[76]:


#Create graphic heatmap of rotation with n taken out
#Take the log of everything to it can be graphed better, add 1 to get rid of log(0)
heatmap_rotation_without_n += 1
heatmap_rotation_without_n = np.log(heatmap_rotation_without_n)
plt.figure(figsize=(20,10))
plt.title('Log Rotation Frequence of Weaselball Structure with n taken out')
plt.xlabel('n * RESOLUTION_OF_S1')
plt.ylabel('log(frequency)')
heatmap_rotation_without_n.plot.bar()
plt.savefig(fig_prefix + "rotation_heatmap_without_n.png")
plt.show()



# In[77]:


#Create graphic heatmap of rotation without n taken out
#Take the log of everything to it can be graphed better, add 1 to get rid of log(0)
heatmap_rotation += 1
heatmap_rotation = np.log(heatmap_rotation)
plt.figure(figsize=(20,10))
plt.title('Log Rotation Frequence of Weaselball Structure without n taken out')
plt.xlabel('n * RESOLUTION_OF_S1')
plt.ylabel('log(frequency)')
heatmap_rotation.plot.bar()
plt.savefig(fig_prefix + "rotation_heatmap.png")
plt.show()

