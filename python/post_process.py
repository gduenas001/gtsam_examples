import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy.stats import chi2

# read residuals form csv
r= np.genfromtxt('../results/residuals.csv', delimiter='\n')


# chi-squared
df= 27
mean, var, skew, kurt = chi2.stats(df, moments='mvsk')
print 'Expected for ' + str(df) + ' dof:'
print 'mean: ' + str(mean) + ' var: ' + str(var) +\
	  'skew: ' + str(skew) + ' kurt: ' + str(kurt)

print 'From data:'
print 'mean: ' + str(np.mean(r)) + ' var: ' + str(np.var(r))
	  # 'skew: ' + str(skew) + ' kurt: ' + str(kurt)



# create figure
fig, ax = plt.subplots(1, 1)

# plot chi-squared pdf
x= np.linspace(chi2.ppf(0.01, df), chi2.ppf(0.99, df), 100)
ax.plot(x, chi2.pdf(x, df), 'r-', lw=5, alpha=0.6, label='chi2 pdf')


# plot histogram for residuals
# num_bins= int(r.size / 10)
ax.hist(r, bins= "freedman", density=True, histtype='stepfilled', alpha=0.2)

plt.show()





