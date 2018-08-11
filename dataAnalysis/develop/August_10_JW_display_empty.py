
# coding: utf-8

# # The purpose of this journal is to prove that the markov transition matrices are very empty so we should consider using a different data structure
# 
# This experiment will be done by comparing the time to make and do operations on different data structures. Namely I will be focusing on the time to make the data structures (as this is a large bottleneck in 2018-07-22-jw-weaselball_analysis.ipynb), as well as the time it takes to do a cross product on 2 of the same data structure matrices.

# ## Import modules and define a few magic numbers

# In[1]:


import pandas as pd
import numpy as np
from mapping import Mapping
get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figuires/2018-07-22-jw-weaselball_analysis"
data_prefix = "../data/2018-07-22-jw-weaselball_analysis_"


# ## Get the data we want and clean it

# ### First get the data

# In[2]:


FLOAT_ERROR_TOLERANCE = 0.00000000001 #See IEEE 754 for why a floating point is never perfect
df_strings = ['../data/08-04-2018_23-37-09.csv']
frames = []
for csv in df_strings:
    temp = pd.read_csv(csv,index_col=False )
    frames.append(temp)
df = pd.concat(frames,ignore_index=True)
print(df.shape)
df = df.drop(columns=['Time'])
df = df.dropna() #Get rid of any rows with NA in it.
df = df.apply(pd.to_numeric)
df.head(10)


# ### Sample the data

# In[3]:


SAMPLING_RATE = 250 #Keep 1 row for every SAMPLING_RATE
df_sampled = df.iloc[::SAMPLING_RATE,:]
print("Size of new DF is {}".format(df_sampled.shape))
df_sampled.head(10)


# In[4]:


#Clean up the data
df_clean = df_sampled.copy()

#Replace the row indexes with a range from 0..n because originally on import of multiple data files it will start from 1 for each file
df_clean.index = range(df_clean.shape[0])
#When the gazebo run it may collect some data of the robots when they aren't moving for the first few 50 or so samples.
#TODO
df_clean.head()


# ### Shift the data

# In[5]:


#Clean up the data

#Shift Yaw to go from 0 to 2pi, so just add PI since it currently goes for -pi to pi

df_clean['Yaw'] += np.pi
if(df_clean['Yaw'].max() > 2 * np.pi or df_clean['Yaw'].min() < 0):
    print("[ERROR] Cleaning Yaw failed. Make sure range is from 0 and 2 * pi")
    print("Yaw = ({} - {})".format(df_clean['Yaw'].min(), df_clean['Yaw'].max()))


df_clean.head()


# In[6]:


#Clean up the data

#Shift X and Y over so that way it could be made easier to use. Currently the world reference is at the center of the "play area"
LENGTH_OF_BOX = 1.127 #This can be obtained from the .sdf file of the weazelball enclosure in gazebo


df_clean['X'] += LENGTH_OF_BOX / 2
df_clean['Y'] += LENGTH_OF_BOX / 2
if (df_clean['X'].max() > LENGTH_OF_BOX or df_clean['Y'].max() > LENGTH_OF_BOX or df_clean['X'].min() < 0 or df_clean['Y'].min() < 0):
    print("[ERROR] Cleaning X/Y failed, Make sure the points are between 0 and LENGTH_OF_BOX")
    print("Y = ({} - {}) X = ({} to {})".format(df_clean['Y'].min(), df_clean['Y'].max(), df_clean['X'].min(), df_clean['X'].max()))
df_clean.head()


# ### Discretize the data

# In[7]:


#Discretize the data
NUMBER_OF_SQUARES = 100 #This should be a square number to create equal sized squares.
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi

df_discretized = df_clean.copy()
    
mappingBoxConstant = (NUMBER_OF_SQUARES ** (1/2.0)) / (LENGTH_OF_BOX)
for index, row in df_clean.iterrows():
    df_discretized.at[index, 'X'] = int(row['X'] * mappingBoxConstant)
    df_discretized.at[index, 'Y'] = int(row['Y'] * mappingBoxConstant)
    df_discretized.at[index, 'Yaw'] = int(row['Yaw'] / RESOLUTION_OF_S1) * RESOLUTION_OF_S1
        
df_discretized.describe()


# ### Initialize a few variables/objects used in the experiment

# In[8]:




#The formula for mapping a 3D array to 1D is
#(z * xMax * yMax) + (y * xMax) + x;
#https://stackoverflow.com/questions/7367770/how-to-flatten-or-index-3d-array-in-1d-array
X_MAX = NUMBER_OF_SQUARES ** (1/2.0)
Y_MAX = NUMBER_OF_SQUARES ** (1/2.0)
YAW_MAX = (2 * np.pi) / RESOLUTION_OF_S1
mapping = Mapping(X_MAX, Y_MAX, YAW_MAX)#Fill in the logical areas that the system can reach (For now I am assuming it can go up/down 2 yaw states or the surronding (x,y) blocks)
d = {} #d is used to hold all the transitions and keep a counter of how often each occur

TRANSLATION_MATRIX_INITIAL_VALUE = 1
#Create the matrix representing the Markov Chain
#I am assuming we are discretizing the space into equal sized boxes
#The transition matrix A is of size 
#( # of states of X * # of states of Y * # of states of Yaw)
#( # of states of X and Y = mappingBoxConstant * LENGTH_OF_BOX )
#( # of states of Yaw = int(2*pi / RESOLUTION_OF_S1)+1)
number_of_x_states = mappingBoxConstant * LENGTH_OF_BOX
number_of_y_states = mappingBoxConstant * LENGTH_OF_BOX
number_of_s1_states = int(2*np.pi / RESOLUTION_OF_S1) + 1
n = int(number_of_x_states * number_of_y_states * number_of_s1_states) #n is the size of the transition matrix
n


# ### Create artificial data if needed

# In[9]:


#HUERISTIC: Add a +1 to any logical possible state the structure would likely end up in.
#This lowers the amount of artifiical data in the matrix (most of which isnt needed)
for index in range(n):
    if(TRANSLATION_MATRIX_INITIAL_VALUE == 0):
        break
    (x,y,yaw) = mapping.map1Dto3D(index)
    #Generate all possible (x,y,yaw) permutations
    #I assume we can only move with 1 around (x,y) including diagnols and +- 2 yaw states
    VARIATION_OF_X = 1
    VARIATION_OF_Y = 1
    VARIATION_OF_YAW = 1
    possible_spots = []
    for x_p in range(-VARIATION_OF_X,VARIATION_OF_X+1):
        for y_p in range(-VARIATION_OF_Y,VARIATION_OF_Y+1):
            for yaw_p in range(-VARIATION_OF_YAW,VARIATION_OF_YAW+1):
                if(mapping.checkValid3DMap(x+x_p, y+y_p, yaw+yaw_p)):
                    possible_spots.append( (x+x_p, y+y_p, yaw+yaw_p) )
    for pose in possible_spots:
        key = (x,y,yaw, pose[0],pose[1],pose[2])
        if key in d:
            d[key] += 1
        else:
            d[key] = 1


# ### Fill in the dictionary with actual data from the experiment

# In[10]:


#Our keys to the dictionary will look like (x_t, y_t, yaw_t, x_t+1, y_t+1, yaw_t+1)
#Go through all but last row since t+1 isnt defined there...
skipCount = 0
try:
    for index, row in df_discretized.iterrows():
        if(index == df_discretized.index[-1]):
            break
        if(df_discretized.at[index, 'ResetID'] != df_discretized.at[index + 1, 'ResetID']):
            skipCount += 1
            continue
        #Need to round here because Yaw data has floating point error
        key = (df_discretized.at[index, 'X'], df_discretized.at[index, 'Y'], round(df_discretized.at[index, 'Yaw'], 6),df_discretized.at[index+1, 'X'], df_discretized.at[index+1, 'Y'], round(df_discretized.at[index+1, 'Yaw'], 6) )

        if key in d:
            d[key] += 1
        else:
            d[key] = 1
except Exception as e:
    print e
    
print "[DEBUG] Skipped {} events".format(skipCount)


# ## Test data structures
# Now that the data has been imported into a dictionary, the different data sturctures can be compared.

# ### Pandas dataframe

# In[11]:


import time
pandas_start_time = time.time()


# #### Fill in matrix with dictionary data

# In[12]:


translation_matrix = pd.DataFrame(0, index=range(n), columns=range(n))

for key, value in d.iteritems():
    #we need to map yaw to an int state

    element_t = mapping.map3Dto1D(key[0], key[1], key[2])
    element_t_plus_1 = mapping.map3Dto1D(key[3], key[4], key[5])
    translation_matrix.at[element_t, element_t_plus_1] = value + translation_matrix.at[element_t, element_t_plus_1]


# #### Make the rows have a magnitude of 1

# In[13]:


for index, row in translation_matrix.iterrows():
    totalActionsInThisState = row.sum()
    if totalActionsInThisState == 0:
        continue
    translation_matrix.iloc[index] /= totalActionsInThisState


# In[14]:


pandas_creation_time = time.time() - pandas_start_time


# ### SciPy Sparse matrix

# In[15]:


from scipy import sparse
from sklearn.preprocessing import normalize
scipy_start_time = time.time()


# #### Fill in matrix with dictionary data

# In[ ]:


sparse_matrix = sparse.dok_matrix((n, n), dtype=np.float32)


# In[20]:


for key, value in d.iteritems():
    element_t = mapping.map3Dto1D(key[0], key[1], key[2])
    element_t_plus_1 = mapping.map3Dto1D(key[3], key[4], key[5])
    sparse_matrix[element_t, element_t_plus_1] = value + sparse_matrix[element_t, element_t_plus_1]
    
#Convert matrix to csr since csr can do multiplication a lot faster
sparse_matrix = sparse_matrix.transpose().tocsr()


# In[21]:


#### Make the rows have a magnitude of 1
sparse_matrix_normalized = normalize(sparse_matrix, norm='l1', axis=1)


# In[ ]:


scipy_creation_time = time.time() - scipy_start_time


# ## Dot product test

# ### Pandas Dot Product Test

# In[ ]:


pandas_dot_start_time = time.time()
foo = translation_matrix.dot(translation_matrix)
pandas_dot_time = time.time() - pandas_cross_start_time


# ### Scipy Sparse Matrix Dot Product Test

# In[ ]:


pandas_dot_start_time = time.time()
foo = translation_matrix.dot(translation_matrix)
pandas_dot_time = time.time() - pandas_cross_start_time

