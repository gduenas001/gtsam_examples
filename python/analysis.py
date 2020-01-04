
import os
import matplotlib.pyplot as plt
import numpy as np
# import csv
from scipy.stats import chi2
from astropy.visualization import hist
import scipy.stats as scipy
import argparse
import itertools
import logging

class Data(object):
    '''
    Data class to store the data saved in the files
    Usually the dictionary contains two keys:
    - values
    - names
    '''

    def __init__(self):
        self.residuals= {}
        self.errors= {}
        self.var= {}
        self.lir= {}
        



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
    params= load_params(filename)

    # load data
    data= load_data(params['workspace'])

    # save plots 
    make_plots(params['workspace'], data)

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



def load_params(filename):
    '''
    Loads the parameters from the copy of params
    stored in the folder of the log
    '''

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



def load_data(workspace, \
              residuals= True, \
              errors= True, \
              variances= True, \
              lir= True):
    '''
    Loads residuals, errors and LIR if set to true
    Returns a Data obj with dictionary entries:
    - names
    - values
    '''

    data= Data()
    # read residuals form csv
    if residuals:
        filename= os.path.join(workspace, 'residuals.csv')
        data.residuals['values']= np.genfromtxt(filename, delimiter=',', skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.residuals['names']= line.split('  ')
        
    # read errors from csv
    if errors:
        filename= os.path.join(workspace, 'errors.csv')
        data.errors['values']= np.genfromtxt(filename, delimiter=',', skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.errors['names']= line.split(' ')

    # read variances from csv
    if variances:
        filename= os.path.join(workspace, 'variance.csv')
        data.var['values']= np.genfromtxt(filename, delimiter=',', skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.var['names']= line.split(' ')


    # read LIR from csv 
    if lir:
        filename= os.path.join(workspace, 'lir.csv')
        # skip two lines b/c at time 0, LIR is not set
        data.lir['values']= np.genfromtxt(filename, delimiter=',', skip_header=2)
        # set zeros to 1e-12 (careful, set times to 1e-12 too)
        mask= data.lir['values'] < 1e-12
        data.lir['values'][mask]= 1e-12
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.lir['names']= line.split('  ')


    # return data
    return data
    


def make_plots(workspace, \
               data, \
               residuals= True, \
               errors= True, \
               lir= True):
    '''
    Plots the data loaded in load_data and set to true
    '''
    
    # --> figure residuals
    fig_res, axs_res= plt.subplots(4)
    plt.xlabel('Time [s]')

    # plot errors
    for errors_name, ind in zip(data.errors['names'], \
                                    range(0, data.errors['values'].shape[1])):
        if errors_name == 'x' or \
           errors_name == 'y' or \
           errors_name == 'z':
            axs_res[0].plot(data.errors['values'][:,0], \
                        data.errors['values'][:,ind], \
                        label= errors_name)
            axs_res[0].grid(b=True)
            axs_res[0].legend()

    # plot residuals
    axs_ind= 1 # start at the second plot
    for res_name, ind in zip(data.residuals['names'], \
                         range(0,data.residuals['values'].shape[1])):
        if res_name == 'imu' or\
           res_name == 'gps' or\
           res_name == 'lidar':
            axs_res[axs_ind].plot(data.residuals['values'][:,0], \
                              data.residuals['values'][:,ind], \
                              label=res_name)
            axs_res[axs_ind].grid(b=True)
            axs_res[axs_ind].legend()
            axs_ind += 1

    # save figure
    filename= os.path.join(workspace, 'residuals.png')
    fig_res.savefig(filename, dpi=400)


    # --> figure variances
    fig_var, axs_var= plt.subplots(3)
    plt.xlabel('Time [s]')

    # plot errors
    for errors_name, ind in zip(data.errors['names'], \
                            range(0, data.errors['values'].shape[1])):
        if errors_name == 'x':
            axs_var[0].plot(data.errors['values'][:,0], \
                       np.abs(data.errors['values'][:,ind]), \
                       label= errors_name + ' error')
            axs_var[0].grid(b=True)
            axs_var[0].legend()
        if errors_name == 'y':
            axs_var[1].plot(data.errors['values'][:,0], \
                       np.abs(data.errors['values'][:,ind]), \
                       label= errors_name + ' error')
            axs_var[1].grid(b=True)
            axs_var[1].legend()
        if errors_name == 'yaw':
            axs_var[2].plot(data.errors['values'][:,0], \
                       np.abs(data.errors['values'][:,ind]), \
                       label= errors_name + ' error')
            axs_var[2].grid(b=True)
            axs_var[2].legend()

    # plot variances
    for var_name, ind in zip(data.var['names'], \
                          range(0, data.var['values'].shape[1])):
        if var_name == 'x':
            axs_var[0].plot(data.var['values'][:,0], \
                            np.sqrt(data.var['values'][:,ind]), \
                            label=var_name + ' 1sig. SD')
            axs_var[0].grid(b=True)
            axs_var[0].legend()
        if var_name == 'y':
            axs_var[1].plot(data.var['values'][:,0], \
                            np.sqrt(data.var['values'][:,ind]), \
                            label=var_name + ' 1sig. SD')
            axs_var[1].grid(b=True)
            axs_var[1].legend()
        if var_name == 'yaw':
            axs_var[2].plot(data.var['values'][:,0], \
                            np.sqrt(data.var['values'][:,ind]), \
                            label=var_name + ' 1sig. SD')
            axs_var[2].grid(b=True)
            axs_var[2].legend()


    # save figure
    filename= os.path.join(workspace, 'variances.png')
    fig_var.savefig(filename, dpi=400)


    # --> figure LIR
    fig_lir, axs_lir= plt.subplots(3)
    plt.xlabel('Time [s]')

    # plot errors
    for errors_name, ind in zip(data.errors['names'], \
                            range(0, data.errors['values'].shape[1])):
        if errors_name == 'x' or \
           errors_name == 'y' or \
           errors_name == 'z':
            axs_lir[0].plot(data.errors['values'][:,0], \
                       data.errors['values'][:,ind], \
                       label= errors_name)
            axs_lir[0].grid(b=True)
            axs_lir[0].legend()

    # plot LIR per hypothesis
    for hypo_name, ind in zip(data.lir['names'], \
                          range(0, data.lir['values'].shape[1])):
        if 'gps' in hypo_name:
            axs_lir[1].plot(data.lir['values'][:,0], \
                            data.lir['values'][:,ind], \
                            label=hypo_name)
            axs_lir[1].grid(b=True)
            axs_lir[1].set_yscale('log')
            axs_lir[1].legend()
        if 'lidar' in hypo_name:
            axs_lir[2].plot(data.lir['values'][:,0], \
                            data.lir['values'][:,ind], \
                            label=hypo_name)
            axs_lir[2].grid(b=True)
            axs_lir[2].set_yscale('log')
            axs_lir[2].legend()

    # save figure
    filename= os.path.join(workspace, 'lir.png')
    fig_lir.savefig(filename, dpi=400)
    



if __name__ == '__main__':
    main()


