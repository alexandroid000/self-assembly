
# coding: utf-8

# In[1]:


#The purpose of this test is to show that there are currently more samples for the diagnol than there are for the non diagnol parts of the matrix
#This means that the data we record will often have the weaselball stay in the same state
#I beleive this occurs because the simulator samples very quickly


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
fig_prefix = "../figures/2018-07-22-jw-weaselball-diagnol_"
data_prefix = "../data/2018-07-22-jw-weaselball-diagnol_"


# In[3]:


df = pd.read_csv('../data/2018-07-22-jw-weaselball_analysis_translation_matrix_out.csv')
df.head()


# In[4]:


sumDiagnol = 0
sumNotDiagnol = 0

for index, row in df.iterrows():
    for i in range(row.size):
        if(index == i):
            sumDiagnol += df.iat[index,i]
        else:
            sumNotDiagnol += df.iat[index,i]


# In[5]:


sumDiagnolAveraged = sumDiagnol/(df.shape[0])
sumNotDiagnolAveraged = sumDiagnol/(df.shape[0]**2 - df.shape[0])


# In[6]:


#This graphically shows that my hypothesis is true
#As can be seen the average "Not Diagnol" probability is orders of magnitude smaller than the average "Diagnol" probability.
data = pd.Series([sumDiagnolAveraged,sumNotDiagnolAveraged], index=['Diagnol', 'Not Diagnol'])
x = ["Diagnol","Not Diagnol"]
barplot = sns.barplot(x=x, y =np.log(data) )
plot = barplot.get_figure()
plot.savefig(fig_prefix + "diagnol_log_p.png")

