
# coding: utf-8

# # The purpose of this notebook is to run a benchmark test on the different size transition matrices to find a good size to have our code run on

# In[119]:


import pandas as pd
import numpy as np
from mapping import Mapping
from scipy import sparse
from sklearn.preprocessing import normalize
from random import randint
import time
import seaborn as sns

get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figures/2018-08-14_jw-benchmark_test_"
data_prefix = "../data/2018-08-14_jw-benchmark_test_"


# ## Define the functions used in the test

# In[120]:


def crossproduct_sparse(P,Q, size):
    start_time = time.time()
    k = size #K is used for the number of states each configuration can be in. This needs to be the same for all configurations
            # And this isnt a bad assumption because all of the configurations should be discretized the same way
    if(P.get_shape()[0] != P.get_shape()[1] or Q.get_shape()[0] != Q.get_shape()[1]):
        print("[Error] the transition matrices should be square!")
        print(P.get_shape())
        print(Q.get_shape())
        return None
    
    length_P = P.get_shape()[0]
    length_Q = Q.get_shape()[0]
    
    d_M1 = {}
    for key in P.keys():
        new_key = key + ((1,),)
        d_M1[new_key] = P.get(key)

    d_M2 = {}
    for key in Q.keys():
        new_key = key + ((2,),)
        d_M2[new_key] = Q.get(key)
    print("preparing all the data took {} seconds".format(time.time()-start_time))   
    PQ = sparse.dok_matrix( (length_P*length_Q,length_P*length_Q), dtype=np.float32)
    for key_1,value_1 in d_M1.iteritems():
        for key_2,value_2 in d_M2.iteritems():
            #The new key is in the format of (x_1,y_1,yaw_1,x_2,y_2,yaw_2,...,x_n,y_n,yaw_n, [1,2,...,n])
            #Note: The (x,y,yaw) should also just be a map3dTo1D
            #By doing this format we can track back the states of different configurations
            combined_key = key_1[:-1] + key_2[:-1] + (key_1[-1] + key_2[-1],)
            x_map = y_map = 0
            for i in range(len(key_1[-1] + key_2[-1],)):
                x_map += combined_key[2 * i] * (k ** i)
                y_map += combined_key[1 + 2 * i] * (k ** i)
            PQ[x_map,y_map] = value_1 * value_2
    print("completing took {} seconds".format(time.time()-start_time))           
    return PQ


# In[121]:


def createRandomSparseMatrix(n):
    new_matrix = sparse.dok_matrix( (n,n), dtype=np.float32)
    empty_percentage = 1 - 0.993707205845
    total_non_zeroes = empty_percentage * (n*n)

    min_number_of_events = 1 #This is arbitrary
    max_number_of_events = 1000 #This is arbitrary
    non_zero_locations = []
    #Manually add along the diagnol so that way all rows have an action
    for i in range(n):
        non_zero_locations.append((i,i))
    while (len(non_zero_locations) < total_non_zeroes):
        new_x = randint(0, n-1)
        new_y = randint(0, n-1)
        if (new_x, new_y) not in non_zero_locations:
            non_zero_locations.append((new_x,new_y))
    for coordinate in non_zero_locations:
        new_matrix[coordinate[0], coordinate[1]] = randint(min_number_of_events,max_number_of_events)
    normalized_new_matrix = normalize(new_matrix, norm='l1', axis=1)
    return normalized_new_matrix


# ## Start the test

# In[122]:


max_waiting_time = 100 #In seconds
results = []
continue_flag = 1
size = 2


# In[123]:


#The test will work like this. We will keep running until there exists a test that goes over the time constraint given.
while(continue_flag == 1):
    test_matrix = createRandomSparseMatrix(size)
    sparse_test_matrix = sparse.dok_matrix(test_matrix)
    test_time_start = time.time()
    crossproduct_sparse(sparse_test_matrix,sparse_test_matrix,size)
    results.append( (size, time.time() - test_time_start) )
    if(time.time() - test_time_start > max_waiting_time):
        continue_flag = 0
    size = size * 2


# In[124]:


results


# In[125]:


df = pd.DataFrame(results, columns=['n', 'seconds'])
df


# In[126]:


df['log(seconds)'] = np.log(df['seconds'])
df


# ## Create the appropiate graphics

# In[127]:


#Create appropiate log figure
sns.set(style="whitegrid")
ax = sns.barplot(x=df['n'], y=df['log(seconds)'], data=df).set_title('Size of n x n sparse matrix vs log(time) to calculate cross product')
ax.figure.savefig(fig_prefix + "log_seconds_for_completion")


# In[128]:


#Create appropiate figure
sns.set(style="whitegrid")
ax = sns.barplot(x=df['n'], y=df['seconds'], data=df).set_title('Size of n x n sparse matrix vs time to calculate cross product')
ax.figure.savefig(fig_prefix + "seconds_for_completion")

