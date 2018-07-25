
# coding: utf-8

# In[25]:


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


# In[26]:


df = pd.read_csv('../data/collection.csv', index_col=False)
df.head()


# In[27]:


#Clean x,y data
df_clean = df.copy()
#Clean up the data

#Shift X and Y over so that way it could be made easier to use. Currently the world reference is at the center of the "play area"
LENGTH_OF_BOX = 1.127 #This can be obtained from the .sdf file of the weazelball enclosure in gazebo

df_clean['Mount_X'] += LENGTH_OF_BOX / 2
df_clean['Mount_Y'] += LENGTH_OF_BOX / 2
df_clean.head()


# In[28]:


#Discretize x,y
NUMBER_OF_SQUARES = 100 #This should be a square number to create equal sized squares.
RESOLUTION_OF_S1 = 0.1 #This is used to discretize the yaw angle over 0 - 2*pi


df_discretized = df_clean.copy()
mappingBoxConstant = (NUMBER_OF_SQUARES ** (1/2.0)) / (LENGTH_OF_BOX)
for index, row in df_clean.iterrows():
    df_discretized.at[index, 'Mount_X'] = int(row['Mount_X'] * mappingBoxConstant)
    df_discretized.at[index, 'Mount_Y'] = int(row['Mount_Y'] * mappingBoxConstant)

df_discretized.head()


# In[29]:


#Create goal columns (x_t+1, y_t+1, z_t+1)

df_discretized['Mount_X_Next'] = df_discretized['Mount_X']
df_discretized['Mount_Y_Next'] = df_discretized['Mount_Y']
df_discretized['Mount_Yaw_Next'] = df_discretized['Mount_Yaw']

df_discretized['Mount_X_Next'].shift(-1)
df_discretized['Mount_Y_Next'].shift(-1)
df_discretized['Mount_Yaw_Next'].shift(-1)

df_discretized.drop(df_discretized.index[len(df_discretized)-1])
df_discretized.head()


# In[23]:


#one-hot encode the ID


# In[30]:


#Split into training and testing
df_discretized['is_train'] = np.random.uniform(0, 1, len(df)) <= .75
df_discretized.head()


# In[31]:


#Create dataframes based on split
train, test = df_discretized[df_discretized['is_train']==True], df_discretized[df_discretized['is_train']==False]
# Show the number of observations for the test and training dataframes
print('Number of observations in the training data:', len(train))
print('Number of observations in the test data:',len(test))


# In[32]:


#print list of information of each column (sanity check)
df_discretized.describe()


# In[33]:


# Create a list of the feature column's names
features = df_discretized.columns[1:35]

# View features
features


# In[38]:


#Train the classifier
clf = RandomForestClassifier(n_jobs=-1, verbose = 1)
clf.fit(train[features], train['Mount_X_Next'])


# In[39]:


# Apply the Classifier we trained to the test data 
preds= clf.predict(test[features])


# In[40]:


# View the predicted probabilities of the first 10 observations
clf.predict_proba(test[features])[0:10]


# In[41]:


# Create confusion matrix
pd.crosstab(test['Mount_X_Next'], preds, rownames=['Actual Species'], colnames=['Predicted Species'])


# In[42]:


# View a list of the features and their importance scores
list(zip(train[features], clf.feature_importances_))

