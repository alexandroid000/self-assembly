
# coding: utf-8

# In[78]:


import pandas as pd
import numpy as np
get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figuires/2018-07-22-jw-weaselball_analysis"
data_prefix = "../data/2018-07-22-jw-weaselball_analysis_"


# In[79]:


#import the data

FLOAT_ERROR_TOLERANCE = 0.00000000001 #See IEEE 754 for why a floating point is never perfect
df = pd.read_csv('../data/collection.csv')
for index, row in df.iterrows():
    df.at[index,'Time'] = row['Time'].replace(' ', '.')

df = df.apply(pd.to_numeric)
df.columns = ['','Time','X','Y','Yaw']
print(df.head())




# In[80]:


#Clean up the data

df_clean = df.copy()
#When the gazebo run it may collect some data of the robots when they aren't moving for the first few 50 or so samples.
#TODO


# In[81]:


#Clean up the data

#Shift Yaw to go from 0 to 2pi, so just add PI since it currently goes for -pi to pi
df_clean['Yaw'] += np.pi
if(df_clean['Yaw'].max() > 2 * np.pi or df_clean['Yaw'].min() < 0):
    print("[ERROR] Cleaning Yaw failed. Make sure range is from 0 and 2 * pi")
    print("Yaw = ({} - {})".format(df_clean['Yaw'].min(), df_clean['Yaw'].max()))


print(df_clean.head())


# In[82]:


#Clean up the data

#Shift X and Y over so that way it could be made easier to use. Currently the world reference is at the center of the "play area"
LENGTH_OF_BOX = 1.127 #This can be obtained from the .sdf file of the weazelball enclosure in gazebo


df_clean['X'] += LENGTH_OF_BOX / 2
df_clean['Y'] += LENGTH_OF_BOX / 2
if (df_clean['X'].max() > LENGTH_OF_BOX or df_clean['Y'].max() > LENGTH_OF_BOX or df_clean['X'].min() < 0 or df_clean['Y'].min() < 0):
    print("[ERROR] Cleaning X/Y failed, Make sure the points are between 0 and LENGTH_OF_BOX")
    print("Y = ({} - {}) X = ({} to {})".format(df_clean['Y'].min(), df_clean['Y'].max(), df_clean['X'].min(), df_clean['X'].max()))
print(df_clean.head())


# In[83]:


#Clean up the data

#Clean the time data since gazebo prints it weird...
#TODO


# In[84]:


#Discretize the data
NUMBER_OF_SQUARES = 100 #This should be a square number to create equal sized squares.
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi


df_discretized = df_clean.copy()
mappingBoxConstant = (NUMBER_OF_SQUARES ** (1/2.0)) / (LENGTH_OF_BOX)
for index, row in df_clean.iterrows():
    df_discretized.at[index, 'X'] = int(row['X'] * mappingBoxConstant)
    df_discretized.at[index, 'Y'] = int(row['Y'] * mappingBoxConstant)
    df_discretized.at[index, 'Yaw'] = int(row['Yaw'] / RESOLUTION_OF_S1) * RESOLUTION_OF_S1

print(df_discretized.head())


# In[85]:


#Verify Discretizing suceeded by checking that number of states generated is the number of states we expeted or less (Sometimes these things dont visit all states)
if (df_discretized['X'].max() > (NUMBER_OF_SQUARES ** (1/2.0)) or df_discretized['Y'].max() > (NUMBER_OF_SQUARES ** (1/2.0)) or df_discretized['X'].min() < 0 or df_discretized['Y'].min() < 0):
    print("[ERROR] Discretizing X/Y failed, Make sure the points are between 0 and (NUMBER_OF_SQUARES ** (1/2.0)")
    print("Y = ({} - {}) X = ({} to {})".format(df_clean['Y'].min(), df_clean['Y'].max(), df_clean['X'].min(), df_clean['X'].max()))


# In[86]:



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
translation_matrix = pd.DataFrame(TRANSLATION_MATRIX_INITIAL_VALUE, index=range(n), columns=range(n))#We use 1 here to set the whole matrix elements to 1


# In[87]:


#Create a dictionary for storing the transition states analysis
d = {}
#Our keys to the dictionary will look like (x_t, y_t, yaw_t, x_t+1, y_t+1, yaw_t+1)
#Go through all but last row since t+1 isnt defined there...
for index, row in df_discretized.iterrows():
    if(index == df_discretized.shape[0]-1):
        break
    try:
        #Need to round here because Yaw data has floating point error
        key = (df_discretized.at[index, 'X'], df_discretized.at[index, 'Y'], round(df_discretized.at[index, 'Yaw'], 6),df_discretized.at[index+1, 'X'], df_discretized.at[index+1, 'Y'], round(df_discretized.at[index+1, 'Yaw'], 6) )
    except Exception as e:
        print index
        print df_discretized.shape[0]
        print index == df_discretized.shape[0]
    if key in d:
        d[key] += 1
    else:
        d[key] = 1
#print d


# In[88]:


#Fill in matrix with data from dictionary
TRANSLATION_MATRIX_INITIAL_VALUE = 1.0
translation_matrix = pd.DataFrame(TRANSLATION_MATRIX_INITIAL_VALUE, index=range(n), columns=range(n))#We use 1 here to set the whole matrix elements to 1

#The formula for mapping a 3D array to 1D is
#(z * xMax * yMax) + (y * xMax) + x;
#https://stackoverflow.com/questions/7367770/how-to-flatten-or-index-3d-array-in-1d-array
X_MAX = NUMBER_OF_SQUARES ** (1/2.0)
Y_MAX = NUMBER_OF_SQUARES ** (1/2.0)
YAW_MAX = (2 * np.pi) / RESOLUTION_OF_S1
def map3Dto1D(x,y,yaw):
    return int((yaw*X_MAX*Y_MAX) + (y*X_MAX) + x)


#Fill in matrix with dictionary data
for key, value in d.iteritems():
    #we need to map yaw to an int state
    element_t = map3Dto1D(key[0], key[1], key[2]/RESOLUTION_OF_S1)
    element_t_plus_1 = map3Dto1D(key[3], key[4], key[5]/RESOLUTION_OF_S1)
    #Use the following to verify we the math above is fine
    if (translation_matrix.at[element_t, element_t_plus_1] != TRANSLATION_MATRIX_INITIAL_VALUE):
        print("[ERROR] Check 3D to 1D mapping!")
        print(translation_matrix.at[element_t, element_t_plus_1])
        print("{} {}".format(element_t,element_t_plus_1 ))
        break
    translation_matrix.at[element_t, element_t_plus_1] = value + translation_matrix.at[element_t, element_t_plus_1]
   # print("key = {}, elements = {}, {}".format(key, element_t, element_t_plus_1))


# In[89]:


#Divide the whole dataframe by number of data collections to get the probabilities.
print(translation_matrix.head())
for index, row in translation_matrix.iterrows():
    totalActionsInThisState = row.sum()
    translation_matrix.iloc[index] /= totalActionsInThisState
print(translation_matrix.head())


# In[90]:


#validate the matrix (all rows == 1)
for index, row in translation_matrix.iterrows():
    if index == 5:
        break
    if(abs(row.sum() - 1.0) > FLOAT_ERROR_TOLERANCE):
        print("[ERROR] Row probability not equal to one!")
        print(index)
        print(row.sum())


# In[91]:


#Make matrix into CSV
translation_matrix.to_csv(data_prefix + 'translation_matrix_out.csv', encoding='utf-8', index=False)

