import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy.stats import chi2
from astropy.visualization import hist
import scipy.stats as scipy


# read residuals form csv
r= np.genfromtxt('../results/residuals/time20_lag3.csv', delimiter='\n')


# chi-squared
df= 42
mean, var, skew, kurt = chi2.stats(df, moments='mvsk')
print 'Expected for ' + str(df) + ' dof:'
print 'mean: ' + str(mean) + \
	  '\tvar: ' + str(var) +\
	  '\tskew: ' + str(skew) +\
	  '\tkurt: ' + str(kurt)

print 'From data:'
print 'mean: ' + str(np.mean(r)) + \
	  '\tvar: ' + str(np.var(r)) +\
	  '\tskew: ' + str(scipy.skew(r)) + \
	  '\tkurt: ' + str(scipy.kurtosis(r))


# create figure
fig, ax = plt.subplots(1, 1)

# plot chi-squared pdf
x= np.linspace(chi2.ppf(0.0001, df), chi2.ppf(0.9099, df), 300)
ax.plot(x, chi2.pdf(x, df), 'r-', lw=5, alpha=0.5, label='chi2 pdf')


# plot histogram for residuals
num_bins= int(r.size / 50)
# hist(r, bins= 'knuth', density=True, histtype='stepfilled', alpha=0.5)
hist(r, bins= num_bins, density=True, histtype='stepfilled', alpha=1)

plt.show()





