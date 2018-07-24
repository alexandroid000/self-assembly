
# coding: utf-8

# In[2]:


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


# In[3]:


df = pd.read_csv('../data/collection.csv', index_col=False)
df.head()


# In[6]:


#Create goal columns (x_t+1, y_t+1, z_t+1)

df['Mount_X_Next'] = df['Mount_X']
df['Mount_Y_Next'] = df['Mount_Y']
df['Mount_Yaw_Next'] = df['Mount_Z']

df['Mount_X_Next'].shift(-1)
df['Mount_Y_Next'].shift(-1)
df['Mount_Yaw_Next'].shift(-1)

df.head()


# In[7]:


#Split into training and testing
df['is_train'] = np.random.uniform(0, 1, len(df)) <= .75
df.head()


# In[8]:


#Create dataframes based on split
train, test = df[df['is_train']==True], df[df['is_train']==False]
# Show the number of observations for the test and training dataframes
print('Number of observations in the training data:', len(train))
print('Number of observations in the test data:',len(test))


# In[9]:


#print list of dtypes of each column (sanity check)
df.dtypes


# In[10]:


# Create a list of the feature column's names
features = df.columns[1:35]

# View features
features


# In[20]:


#Train the classifier
clf = RandomForestClassifier(n_jobs=1, verbose = 1)
clf.fit(train[features], train['Mount_X_Next'])


# In[21]:


# Apply the Classifier we trained to the test data 
preds= clf.predict(test[features])


# In[22]:


# View the predicted probabilities of the first 10 observations
clf.predict_proba(test[features])[0:10]


# In[23]:


# Create confusion matrix
pd.crosstab(test['Mount_X_Next'], preds, rownames=['Actual Species'], colnames=['Predicted Species'])


# In[24]:


# View a list of the features and their importance scores
list(zip(train[features], clf.feature_importances_))

