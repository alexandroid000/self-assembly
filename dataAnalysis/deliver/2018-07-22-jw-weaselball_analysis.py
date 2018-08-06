
# coding: utf-8

# In[35]:


import pandas as pd
import numpy as np
from mapping import Mapping
get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figuires/2018-07-22-jw-weaselball_analysis"
data_prefix = "../data/2018-07-22-jw-weaselball_analysis_"


# In[36]:


FLOAT_ERROR_TOLERANCE = 0.00000000001 #See IEEE 754 for why a floating point is never perfect
df1 = pd.read_csv('../data/08-05-2018_09-50-39.csv', index_col=False)
print(df1.shape)
frames = [df1]
df = pd.concat(frames,ignore_index=True)
print(df.shape)
df = df.drop(columns=['Time'])
df = df.apply(pd.to_numeric)
df.head(10)


# In[37]:


df.tail(10)


# In[38]:


#Sample the data
SAMPLING_RATE = 250 #Keep 1 row for every SAMPLING_RATE
df_sampled = df.iloc[::SAMPLING_RATE,:]
print("Size of new DF is {}".format(df_sampled.shape))
df_sampled.head(10)


# In[39]:


#Break data into 3 parts. S = {Near 2 Walls, Near 1 Wall, Near No Walls}
#df2 = df_sampled.loc[df_sampled['ResetID'] % 3 == 2]
#df1 = df_sampled.loc[df_sampled['ResetID'] % 3 == 1]
#df0 = df_sampled.loc[df_sampled['ResetID'] % 3 == 0]
#df = df0


# In[40]:


#Clean up the data
df_clean = df_sampled.copy()

df_clean.index = range(df_clean.shape[0])
#When the gazebo run it may collect some data of the robots when they aren't moving for the first few 50 or so samples.
#TODO
df_clean.head()


# In[41]:


#Clean up the data

#Shift Yaw to go from 0 to 2pi, so just add PI since it currently goes for -pi to pi

df_clean['Yaw'] += np.pi
if(df_clean['Yaw'].max() > 2 * np.pi or df_clean['Yaw'].min() < 0):
    print("[ERROR] Cleaning Yaw failed. Make sure range is from 0 and 2 * pi")
    print("Yaw = ({} - {})".format(df_clean['Yaw'].min(), df_clean['Yaw'].max()))


df_clean.head()


# In[42]:


#Clean up the data

#Shift X and Y over so that way it could be made easier to use. Currently the world reference is at the center of the "play area"
LENGTH_OF_BOX = 1.127 #This can be obtained from the .sdf file of the weazelball enclosure in gazebo


df_clean['X'] += LENGTH_OF_BOX / 2
df_clean['Y'] += LENGTH_OF_BOX / 2
if (df_clean['X'].max() > LENGTH_OF_BOX or df_clean['Y'].max() > LENGTH_OF_BOX or df_clean['X'].min() < 0 or df_clean['Y'].min() < 0):
    print("[ERROR] Cleaning X/Y failed, Make sure the points are between 0 and LENGTH_OF_BOX")
    print("Y = ({} - {}) X = ({} to {})".format(df_clean['Y'].min(), df_clean['Y'].max(), df_clean['X'].min(), df_clean['X'].max()))
df_clean.head()


# In[43]:


#Clean up the data

#Clean the time data since gazebo prints it weird...
#TODO


# In[44]:


#Discretize the data
NUMBER_OF_SQUARES = 100 #This should be a square number to create equal sized squares.
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi

df_discretized = df_clean.copy()
    
mappingBoxConstant = (NUMBER_OF_SQUARES ** (1/2.0)) / (LENGTH_OF_BOX)
for index, row in df_clean.iterrows():
    df_discretized.at[index, 'X'] = int(row['X'] * mappingBoxConstant)
    df_discretized.at[index, 'Y'] = int(row['Y'] * mappingBoxConstant)
    df_discretized.at[index, 'Yaw'] = int(row['Yaw'] / RESOLUTION_OF_S1) * RESOLUTION_OF_S1
        
df_discretized.head()
df_discretized.describe()


# In[45]:


#Verify Discretizing suceeded by checking that number of states generated is the number of states we expeted or less (Sometimes these things dont visit all states)

if (df_discretized['X'].max() > (NUMBER_OF_SQUARES ** (1/2.0)) or df_discretized['Y'].max() > (NUMBER_OF_SQUARES ** (1/2.0)) or df_discretized['X'].min() < 0 or df_discretized['Y'].min() < 0):
    print("[ERROR] Discretizing X/Y failed, Make sure the points are between 0 and (NUMBER_OF_SQUARES ** (1/2.0)")
    print("Y = ({} - {}) X = ({} to {})".format(df_clean['Y'].min(), df_clean['Y'].max(), df_clean['X'].min(), df_clean['X'].max()))


# In[46]:


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

n = int(number_of_x_states * number_of_y_states * number_of_s1_states)
print("[DEBUG] Size of n is {}".format(n))
translation_matrix = pd.DataFrame(0, index=range(n), columns=range(n))#We use 1 here to set the whole matrix elements to 1
translation_matrix.head()


# In[47]:




#The formula for mapping a 3D array to 1D is
#(z * xMax * yMax) + (y * xMax) + x;
#https://stackoverflow.com/questions/7367770/how-to-flatten-or-index-3d-array-in-1d-array
X_MAX = NUMBER_OF_SQUARES ** (1/2.0)
Y_MAX = NUMBER_OF_SQUARES ** (1/2.0)
YAW_MAX = (2 * np.pi) / RESOLUTION_OF_S1
mapping = Mapping(X_MAX, Y_MAX, YAW_MAX)#Fill in the logical areas that the system can reach (For now I am assuming it can go up/down 2 yaw states or the surronding (x,y) blocks)


# In[48]:


#HUERISTIC: Add a +1 to any logical possible state the structure would likely end up in.
#This lowers the amount of artifiical data in the matrix (most of which isnt needed)



for index, row in translation_matrix.iterrows():
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
        translation_matrix.at[mapping.map3Dto1D(x,y,yaw), mapping.map3Dto1D(pose[0],pose[1],pose[2])] = translation_matrix.at[mapping.map3Dto1D(x,y,yaw), mapping.map3Dto1D(pose[0],pose[1],pose[2])] +TRANSLATION_MATRIX_INITIAL_VALUE
translation_matrix.head()    


# In[49]:


#Create a dictionary for storing the transition states analysis
d ={}
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
d


# In[50]:


mapping.map3Dto1D(0.0,0.0,4.0)


# In[51]:



#Fill in matrix with dictionary data


for key, value in d.iteritems():
    #we need to map yaw to an int state
    element_t = mapping.map3Dto1D(key[0], key[1], key[2]/RESOLUTION_OF_S1)
    element_t_plus_1 = mapping.map3Dto1D(key[3], key[4], key[5]/RESOLUTION_OF_S1)
    #Use the following to verify we the math above is fine
    if((mapping.checkValid1DMap(element_t)) & (mapping.checkValid1DMap(element_t_plus_1)) == 0 ):
        print "[ERROR] BAD MAPPING!"
    
    translation_matrix.at[element_t, element_t_plus_1] = value + translation_matrix.at[element_t, element_t_plus_1]
   # print("key = {}, elements = {}, {}".format(key, element_t, element_t_plus_1))


# In[52]:


#Check sum of "events" per matrix

totalEvents = 0
for index,row in translation_matrix.iterrows():
    totalEvents += row.sum()
print("Total Events is {}".format(totalEvents))
print("Size of data point df is {}".format(df_discretized.size))


# In[53]:


#Divide the whole dataframe by number of data collections to get the probabilities.


magnitudeVector = pd.Series(0, index=range(n + 1))



for index, row in translation_matrix.iterrows():
    totalActionsInThisState = row.sum()
    magnitudeVector.iloc[index] = totalActionsInThisState
    if totalActionsInThisState == 0:
        continue
    translation_matrix.iloc[index] /= totalActionsInThisState

translation_matrix.head()


# In[54]:


#validate the matrix (all rows == 1)
for index, row in translation_matrix.iterrows():
    if(abs(row.sum() - 1.0) > FLOAT_ERROR_TOLERANCE):
        print("[ERROR] Row probability not equal to one!")
        print(index)
        print(row.sum())


# In[55]:


#Make matrix into CSV
translation_matrix.to_csv(data_prefix + 'translation_matrix_out.csv', encoding='utf-8', index=False)


# In[153]:


#Make csv of the number of instances for each row
#magnitudeVector.to_csv(data_prefix + 'magnitude_vector_out.csv', encoding='utf-8', index=False)

