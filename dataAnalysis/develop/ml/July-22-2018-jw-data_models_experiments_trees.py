
# coding: utf-8

# In[1]:


#!/usr/bin/env python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.ensemble import RandomForestClassifier
get_ipython().magic(u'load_ext version_information')
get_ipython().magic(u'version_information numpy, pandas, matplotlib')
get_ipython().magic(u'version_information')
fig_prefix = "../figures/2018-07-22-jw-weaselball-heatmap_"
data_prefix = "../data/2018-07-22-jw-weaselball-heatmap_"


# In[2]:


df1 = pd.read_csv('../data/07-28-2018_22-55-56.csv', index_col=False)
print(df1.shape)
df2 = pd.read_csv('../data/07-29-2018_17-26-37.csv', index_col=False)
print(df2.shape)
df3 = pd.read_csv('../data/07-29-2018_13-13-11.csv', index_col=False)
print(df3.shape)
df4 = pd.read_csv('../data/07-29-2018_22-44-40.csv', index_col=False)
print(df4.shape)
df5 = pd.read_csv('../data/07-30-2018_03-47-59.csv', index_col=False)
print(df5.shape)
frames = [df1,df2,df3,df4,df5]
df = pd.concat(frames,ignore_index=True)
df.head()


# In[ ]:


#Clean x,y data
df_clean = df.copy()
#Clean up the data

#Shift X and Y over so that way it could be made easier to use. Currently the world reference is at the center of the "play area"
LENGTH_OF_BOX = 1.127 #This can be obtained from the .sdf file of the weazelball enclosure in gazebo

df_clean['Mount_X'] += LENGTH_OF_BOX / 2
df_clean['Mount_Y'] += LENGTH_OF_BOX / 2
df_clean.head()


# In[ ]:


#Discretize x,y
NUMBER_OF_SQUARES = 100 #This should be a square number to create equal sized squares.
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi


df_discretized = df_clean.copy()
mappingBoxConstant = (NUMBER_OF_SQUARES ** (1/2.0)) / (LENGTH_OF_BOX)
for index, row in df_clean.iterrows():
    df_discretized.at[index, 'Mount_X'] = int(row['Mount_X'] * mappingBoxConstant)
    df_discretized.at[index, 'Mount_Y'] = int(row['Mount_Y'] * mappingBoxConstant)

df_discretized.head()


# In[ ]:


#Create goal columns (x_t+1, y_t+1, z_t+1)

df_discretized['Mount_X_Next'] = df_discretized['Mount_X']
df_discretized['Mount_Y_Next'] = df_discretized['Mount_Y']
df_discretized['Mount_Yaw_Next'] = df_discretized['Mount_Yaw']

df_discretized['Mount_X_Next'].shift(-1)
df_discretized['Mount_Y_Next'].shift(-1)
df_discretized['Mount_Yaw_Next'].shift(-1)

df_discretized.drop(df_discretized.index[len(df_discretized)-1])
df_discretized.head()


# In[ ]:


#one-hot encode the ID


# In[ ]:


#Split into training and testing
df_discretized['is_train'] = np.random.uniform(0, 1, len(df)) <= .75
df_discretized.head()


# In[ ]:


#Create dataframes based on split
train, test = df_discretized[df_discretized['is_train']==True], df_discretized[df_discretized['is_train']==False]
# Show the number of observations for the test and training dataframes
print('Number of observations in the training data:', len(train))
print('Number of observations in the test data:',len(test))


# In[ ]:


#print list of information of each column (sanity check)
df_discretized.describe()


# In[ ]:


# Create a list of the feature column's names
features = df_discretized.columns[1:35]

# View features
features


# In[ ]:


#Train the classifier
clf = RandomForestClassifier(n_jobs=-1, verbose = 1)
clf.fit(train[features], train['Mount_X_Next'])


# In[ ]:


# Apply the Classifier we trained to the test data 
preds= clf.predict(test[features])


# In[ ]:


# View the predicted probabilities of the first 10 observations
clf.predict_proba(test[features])[0:10]


# In[ ]:


# Create confusion matrix
pd.crosstab(test['Mount_X_Next'], preds, rownames=['Actual Species'], colnames=['Predicted Species'])


# In[ ]:


# View a list of the features and their importance scores
list(zip(train[features], clf.feature_importances_))

