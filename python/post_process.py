
import os
import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy.stats import chi2
from astropy.visualization import hist
import scipy.stats as scipy
import argparse
import itertools



def main():

    # Construct the argument parser
    parser= argparse.ArgumentParser()

    # Add the arguments to the parser
    parser.add_argument("-a", "--workspace", \
                         required=True, \
                         help="first operand")
    args= vars(parser.parse_args())

    # read params
    filename= os.path.join(args['workspace'], 'params.txt')
    params= load(filename)

    # read residuals form csv
    filename= os.path.join(params['workspace'], 'residuals.csv')
    residuals= np.genfromtxt(filename, delimiter=',', skip_header=1)
    with open(filename, 'r') as f:
        line= f.readline().strip()
        res_names= line.split('  ')
        
    # read errors from csv
    filename= os.path.join(params['workspace'], 'errors.csv')
    errors= np.genfromtxt(filename, delimiter=',', skip_header=1)
    with open(filename, 'r') as f:
        line= f.readline().strip()
        errors_names= line.split(' ')

    # read LIR from csv
    filename= os.path.join(params['workspace'], 'lir.csv')
    lir= np.genfromtxt(filename, delimiter=',', skip_header=1)
    with open(filename, 'r') as f:
        line= f.readline().strip()
        hypo_names= line.split('  ')
    

    # plot errors Vs residuals
    fig_res, axs_res= plt.subplots(4)

    # plot errors
    for errors_name, ind in zip(errors_names, range(0, errors.shape[1])):
        if errors_name == 'x' or \
           errors_name == 'y' or \
           errors_name == 'z':
            axs_res[0].plot(errors[:,0], errors[:,ind], \
                        label= errors_name)
            axs_res[0].legend()


    # plot residuals
    axs_ind= 1 # start at the second plot
    for res_name, ind in zip(res_names, range(0,residuals.shape[1])):
        if res_name == 'imu' or\
           res_name == 'gps' or\
           res_name == 'lidar':
            axs_res[axs_ind].plot(residuals[:,0], residuals[:,ind], \
                              label=res_name)
            axs_res[axs_ind].legend()
            axs_ind += 1


    # plot LIR
    fig_lir, axs_lir= plt.subplots(3)
    for hypo_name, ind in zip(hypo_names, range(0,lir.shape[1])):
        print hypo_name
        if 'gps' in hypo_name:
            axs_lir[1].plot(lir[:,0], \
                            residuals[:,ind], \
                            label=hypo_name)
            axs_lir[1].legend()
        if 'lidar' in hypo_name:
            axs_lir[2].plot(lir[:,0], \
                            residuals[:,ind], \
                            label=hypo_name)
            axs_lir[2].legend()
            

    plt.show()
        

    # # initialize 
    # res= {}
    # for res_type in res_types:

    #   if res_type == 'odom':
    #       res[res_type]= residuals[:,0]
    #   elif res_type == 'gps':
    #       res[res_type]= residuals[:,1]
    #   elif res_type == 'lidar':
    #       res[res_type]= residuals[:,2]
    #   elif res_type == 'sum':
    #       res[res_type]= residuals[:,3]
    #   else:
    #       raise ValueError('Not implemented for ' + res_type)

        
    #   # chi-squared
    #   mean, var, skew, kurt = chi2.stats(dof[res_type], moments='mvsk')
    #   print '---- ' + res_type + ' ----'
    #   print 'Expected for ' + str(dof[res_type]) + ' dof:'
    #   print 'mean: ' + str(mean) + \
    #         '\tvar: ' + str(var) +\
    #         '\tskew: ' + str(skew) +\
    #         '\tkurt: ' + str(kurt)

    #   print 'From data:'
    #   print 'mean: ' + str(np.mean(res[res_type])) + \
    #         '\tvar: ' + str(np.var(res[res_type])) +\
    #         '\tskew: ' + str(scipy.skew(res[res_type])) + \
    #         '\tkurt: ' + str(scipy.kurtosis(res[res_type]))


    # # create figure
    # fig, ax = plt.subplots(1, 1)

    # # plot chi-squared pdf
    # x= np.linspace(chi2.ppf(0.0001, dof['sum']), \
    #              chi2.ppf(0.9999, dof['sum']), 300)
    # ax.plot(x, \
    #       chi2.pdf(x, dof['sum']), \
    #       'r-', \
    #       lw=5, \
    #       alpha=0.5, \
    #       label='chi2 pdf')


    # # plot histogram for residuals
    # num_bins= int(res['sum'].size / 10)
    # # hist(r, bins= 'knuth', density=True, histtype='stepfilled', alpha=0.5)
    # hist(res['sum'], \
    #    bins= num_bins, \
    #    density=True, \
    #    histtype='stepfilled', \
    #    alpha=1)

    # plt.show()



def load(filename):
    # initilize params dict
    params= {}

    FILE= open(filename)
    for line in FILE:
        # check if it's an empty line
        if not line.strip(): continue

        # check if it's a comment -> continue
        if line[0] == '#': continue

        # check if there is a comment after the param value
        comment_ind= line.find('#')
        if not comment_ind == -1:
            line= line[0:comment_ind]

        # get the name & value
        name, value= line.split("=")
        value= value.strip()
        if value.isdigit():
            value= float(value)

        # add to params dict
        params[name]= value

    return params

if __name__ == '__main__':
    main()


