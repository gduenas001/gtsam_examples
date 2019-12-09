
import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy.stats import chi2
from astropy.visualization import hist
import scipy.stats as scipy

# input variables
sim_time= 10
lag= 10
res_types= ['odom', 'gps', 'lidar', 'sum']
dof= {}
dof['odom']= (lag - 1) * 9
dof['gps']= lag * 3
dof['lidar']= lag * 4 * 3
dof['sum']= dof['odom'] + dof['gps'] + dof['lidar'] - 15 * lag
# add prior
dof['sum'] += 15


# read residuals form csv
filename= '../results/residuals/types_time' + \
	str(sim_time) + '_lag' +  str(lag) + '.csv'
residuals= np.genfromtxt(filename, delimiter=' ')

# initialize 
res= {}
for res_type in res_types:

	if res_type == 'odom':
		res[res_type]= residuals[:,0]
	elif res_type == 'gps':
		res[res_type]= residuals[:,1]
	elif res_type == 'lidar':
		res[res_type]= residuals[:,2]
	elif res_type == 'sum':
		res[res_type]= residuals[:,3]
	else:
		raise ValueError('Not implemented for ' + res_type)

	
	# chi-squared
	mean, var, skew, kurt = chi2.stats(dof[res_type], moments='mvsk')
	print '---- ' + res_type + ' ----'
	print 'Expected for ' + str(dof[res_type]) + ' dof:'
	print 'mean: ' + str(mean) + \
		  '\tvar: ' + str(var) +\
		  '\tskew: ' + str(skew) +\
		  '\tkurt: ' + str(kurt)

	print 'From data:'
	print 'mean: ' + str(np.mean(res[res_type])) + \
		  '\tvar: ' + str(np.var(res[res_type])) +\
		  '\tskew: ' + str(scipy.skew(res[res_type])) + \
		  '\tkurt: ' + str(scipy.kurtosis(res[res_type]))


# create figure
fig, ax = plt.subplots(1, 1)

# plot chi-squared pdf
x= np.linspace(chi2.ppf(0.0001, dof['sum']), \
			   chi2.ppf(0.9999, dof['sum']), 300)
ax.plot(x, \
		chi2.pdf(x, dof['sum']), \
		'r-', \
		lw=5, \
		alpha=0.5, \
		label='chi2 pdf')


# plot histogram for residuals
num_bins= int(res['sum'].size / 10)
# hist(r, bins= 'knuth', density=True, histtype='stepfilled', alpha=0.5)
hist(res['sum'], \
	 bins= num_bins, \
	 density=True, \
	 histtype='stepfilled', \
	 alpha=1)

plt.show()





