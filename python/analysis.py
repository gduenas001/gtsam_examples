
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
from mpl_toolkits.mplot3d import Axes3D
import logging 
from Data import Data

# config logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

        


# -------------------------------------------------------
# -------------------------------------------------------
def make_residuals_plot(data, workspace):
    fig, axs= plt.subplots(4)
    plt.xlabel('Time [s]')

    # plot errors
    for errors_name, ind in zip(data.errors['names'], \
                                    range(0, data.errors['values'].shape[1])):
        if errors_name == 'x' or \
           errors_name == 'y' or \
           errors_name == 'z':
            axs[0].plot(data.errors['values'][:,0], \
                        data.errors['values'][:,ind], \
                        label= errors_name)
            axs[0].grid(b=True)
            axs[0].legend()

    # plot residuals
    axs_ind= 1 # start at the second plot
    for res_name, ind in zip(data.residuals['names'], \
                         range(0,data.residuals['values'].shape[1])):
        if res_name == 'imu' or\
           res_name == 'gps' or\
           res_name == 'lidar':
            axs[axs_ind].plot(data.residuals['values'][:,0], \
                              data.residuals['values'][:,ind], \
                              label=res_name)
            axs[axs_ind].grid(b=True)
            axs[axs_ind].legend()
            axs_ind += 1

    # save figure
    filename= os.path.join(workspace, 'residuals.png')
    fig.savefig(filename, dpi=400)


# -------------------------------------------------------
# -------------------------------------------------------
def make_variance_plot(data, workspace):

    logger.debug('Enter make_variance_plot.')

    # first figure for (x y z)
    fig, axs= plt.subplots(3)
    plt.xlabel('Time [s]')
    plt.ylabel('Errors + 1sig. SD [m]')

    logger.info('The error names are: ')
    logger.info(data.errors['names'])
    # plot errors (x y z)
    for errors_name, ind in zip(data.errors['names'], \
                            range(0, data.errors['values'].shape[1])):
        if errors_name == 'x':
            logger.info('Plot ' + errors_name + ' with index ' + str(ind))
            axs[0].plot(data.errors['values'][:,0], \
                       np.abs(data.errors['values'][:,ind]), \
                       label= errors_name + ' error')
            axs[0].grid(b=True)
            axs[0].legend()
        if errors_name == 'y':
            logger.info('Plot ' + errors_name + ' with index ' + str(ind))
            axs[1].plot(data.errors['values'][:,0], \
                       np.abs(data.errors['values'][:,ind]), \
                       label= errors_name + ' error')
            axs[1].grid(b=True)
            axs[1].legend()
        if errors_name == 'z':
            logger.info('Plot ' + errors_name + ' with index ' + str(ind))
            axs[2].plot(data.errors['values'][:,0], \
                       np.abs(data.errors['values'][:,ind]), \
                       label= errors_name + ' error')
            axs[2].grid(b=True)
            axs[2].legend()        

    logger.info('The variance names are: ')
    logger.info(data.var['names'])
    # plot variances (x y z)
    for var_name, ind in zip(data.var['names'], \
                          range(0, data.var['values'].shape[1])):
        if var_name == 'x':
            logger.info('Plot ' + var_name + ' with index ' + str(ind))
            axs[0].plot(data.var['values'][:,0], \
                        np.sqrt(data.var['values'][:,ind]), \
                        linestyle='--', \
                        label=var_name + ' 1sig. SD')
            axs[0].grid(b=True)
            axs[0].legend()
        if var_name == 'y':
            logger.info('Plot ' + var_name + ' with index ' + str(ind))
            axs[1].plot(data.var['values'][:,0], \
                        np.sqrt(data.var['values'][:,ind]), \
                        linestyle='--', \
                        label=var_name + ' 1sig. SD')
            axs[1].grid(b=True)
            axs[1].legend()
        if var_name == 'z':
            logger.info('Plot ' + var_name + ' with index ' + str(ind))
            axs[2].plot(data.var['values'][:,0], \
                        np.sqrt(data.var['values'][:,ind]), \
                        linestyle='--', \
                        label=var_name + ' 1sig. SD')
            axs[2].grid(b=True)
            axs[2].legend()

    # save figure
    filename= os.path.join(workspace, 'variances_xyz.png')
    fig.savefig(filename, dpi=400)

    # second figure for (roll pitch yaw)
    fig, axs= plt.subplots(3)
    plt.xlabel('Time [s]')
    plt.ylabel('Error + 1sig. SD [Rad]')

     # plot errors (roll pitch yaw)
    for errors_name, ind in zip(data.errors['names'], \
                            range(0, data.errors['values'].shape[1])):
        if errors_name == 'roll':
            logger.info('Plot ' + errors_name + ' with index ' + str(ind))
            axs[0].plot(data.errors['values'][:,0], \
                        np.abs(data.errors['values'][:,ind]), \
                        label=errors_name + ' 1sig. SD')
            axs[0].grid(b=True)
            axs[0].legend()
        if errors_name == 'pitch':
            logger.info('Plot ' + errors_name + ' with index ' + str(ind))
            axs[1].plot(data.errors['values'][:,0], \
                        np.abs(data.errors['values'][:,ind]), \
                        label=errors_name + ' 1sig. SD')
            axs[1].grid(b=True)
            axs[1].legend()
        if errors_name == 'yaw':
            logger.info('Plot ' + errors_name + ' with index ' + str(ind))
            axs[2].plot(data.errors['values'][:,0], \
                        np.abs(data.errors['values'][:,ind]), \
                        label=errors_name + ' 1sig. SD')
            axs[2].grid(b=True)
            axs[2].legend()

    # plot variances (roll pitch yaw)
    for var_name, ind in zip(data.var['names'], \
                          range(0, data.var['values'].shape[1])):
        if var_name == 'roll':
            logger.info('Plot ' + var_name + ' with index ' + str(ind))
            axs[0].plot(data.var['values'][:,0], \
                        np.sqrt(data.var['values'][:,ind]), \
                        linestyle='--', \
                        label=var_name + ' 1sig. SD')
            axs[0].grid(b=True)
            axs[0].legend()
        if var_name == 'pitch':
            logger.info('Plot ' + var_name + ' with index ' + str(ind))
            axs[1].plot(data.var['values'][:,0], \
                        np.sqrt(data.var['values'][:,ind]), \
                        linestyle='--', \
                        label=var_name + ' 1sig. SD')
            axs[1].grid(b=True)
            axs[1].legend()
        if var_name == 'yaw':
            logger.info('Plot ' + var_name + ' with index ' + str(ind))
            axs[2].plot(data.var['values'][:,0], \
                        np.sqrt(data.var['values'][:,ind]), \
                        linestyle='--', \
                        label=var_name + ' 1sig. SD')
            axs[2].grid(b=True)
            axs[2].legend()

    # save figure
    filename= os.path.join(workspace, 'variances_rpy.png')
    fig.savefig(filename, dpi=400)

    logger.debug('Exit make_variance_plot.')


# -------------------------------------------------------
# -------------------------------------------------------
def make_lir_plot(data, workspace):
    fig, axs= plt.subplots(3)
    plt.xlabel('Time [s]')

    # plot errors
    for errors_name, ind in zip(data.errors['names'], \
                            range(0, data.errors['values'].shape[1])):
        if errors_name == 'x' or \
           errors_name == 'y' or \
           errors_name == 'z':
            axs[0].plot(data.errors['values'][:,0], \
                        data.errors['values'][:,ind], \
                        label= errors_name)
            axs[0].grid(b=True)
            axs[0].legend()

    # plot LIR per hypothesis
    for hypo_name, ind in zip(data.lir['names'], \
                          range(0, data.lir['values'].shape[1])):
        if 'gps' in hypo_name:
            axs[1].plot(data.lir['values'][:,0], \
                        data.lir['values'][:,ind], \
                        label=hypo_name)
            axs[1].grid(b=True)
            axs[1].set_yscale('log')
            axs[1].legend()
        if 'lidar' in hypo_name:
            axs[2].plot(data.lir['values'][:,0], \
                        data.lir['values'][:,ind], \
                        label=hypo_name)
            axs[2].grid(b=True)
            axs[2].set_yscale('log')
            axs[2].legend()

    # save figure
    filename= os.path.join(workspace, 'lir.png')
    fig.savefig(filename, dpi=400)


# -------------------------------------------------------
# -------------------------------------------------------
def make_trajectory_plot(data, params, workspace):

    # initialize figure
    fig= plt.figure()
    axs= plt.subplot(111, projection='3d')

    # plot estimated + true trajectory
    axs.plot(data.estimated_states['values'][:,4], \
                  data.estimated_states['values'][:,5], \
                  data.estimated_states['values'][:,6], \
                  color= 'b', linestyle='-', marker='o')

    axs.plot(data.true_states['values'][:,4], \
                  data.true_states['values'][:,5], \
                  data.true_states['values'][:,6], \
                  color= 'r', linestyle='-')

    # plot landmarks
    axs.scatter3D(params['landmark'][:][0], \
                  params['landmark'][:][1], \
                  params['landmark'][:][2], \
                  color= 'g', marker='^', s=100)

    # add axis
    axs.set_xlabel('X [m]')
    axs.set_ylabel('Y [m]')
    axs.set_zlabel('Z [m]')

     # save figure
    filename= os.path.join(workspace, 'trajectory_1.png')
    fig.savefig(filename, dpi=400)

    filename= os.path.join(workspace, 'trajectory_2.png')
    axs.view_init(azim=30)
    fig.savefig(filename, dpi=400)


# -------------------------------------------------------
# -------------------------------------------------------
def make_plots(data, \
               params, \
               workspace= [], \
               residuals_plot= True, \
               variances_plot= True, \
               lir_plot= True, \
               trajectory_plot= True):
    '''
    Plots the data loaded in load_data and set to true
    - residuals
    - variances (and errors)
    - LIR
    - trajectory with estimate + true positions
    '''

    # if workspace is not specified, used the one in params
    if workspace == []:
        workspace= params['workspace']

    # --> figure residuals
    if residuals_plot:
        make_residuals_plot(data, workspace)
    
    # --> figure variances
    if variances_plot:
        make_variance_plot(data, workspace)

    # --> figure LIR
    if lir_plot:
        make_lir_plot(data, workspace)
    
    # --> plot trajectory + landmarks
    if trajectory_plot:
        make_trajectory_plot(data, params, workspace)


# -------------------------------------------------------
# -------------------------------------------------------
def load_data(workspace, \
              load_residuals= True, \
              load_errors= True, \
              load_variances= True, \
              load_lir= True, \
              load_trajectory= True):
    '''
    Loads residuals, errors and LIR if set to true
    Returns a Data obj with dictionary entries:
    - names
    - values
    '''

    data= Data()
    # read residuals form csv
    if load_residuals:
        filename= os.path.join(workspace, 'residuals.csv')
        data.residuals['values']= np.genfromtxt(filename, delimiter=',', skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.residuals['names']= line.split('  ')
            # eliminate possible empty entries
            data.residuals['names']= filter(lambda a: a != '', data.residuals['names'])

    # read errors from csv
    if load_errors:
        filename= os.path.join(workspace, 'errors.csv')
        data.errors['values']= np.genfromtxt(filename, delimiter=',', skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.errors['names']= line.split('  ')
            # eliminate possible empty entries
            data.errors['names']= filter(lambda a: a != '', data.errors['names'])

    # read variances from csv
    if load_variances:
        filename= os.path.join(workspace, 'variances.csv')
        data.var['values']= np.genfromtxt(filename, delimiter=',', skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.var['names']= line.split('  ')
            # eliminate possible empty entries
            data.var['names']= filter(lambda a: a != '', data.var['names'])

    # read LIR from csv 
    if load_lir:
        filename= os.path.join(workspace, 'lir.csv')
        # skip two lines b/c at time 0, LIR is not set
        data.lir['values']= np.genfromtxt(filename, delimiter=',', skip_header=2)
        # set zeros to 1e-12 (careful, set times to 1e-12 too)
        mask= data.lir['values'] < 1e-12
        data.lir['values'][mask]= 1e-12
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.lir['names']= line.split('  ')
            # eliminate possible empty entries
            data.lir['names']= filter(lambda a: a != '', data.lir['names'])

    # read estimated and true states
    if load_trajectory:
        # read estimated states
        filename= os.path.join(workspace, 'estimated_states.csv')
        data.estimated_states['values']= np.genfromtxt(filename, \
                                                       delimiter=',', \
                                                       skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.estimated_states['names']= line.split('  ')
            # eliminate possible empty entries
            data.estimated_states['names']= filter(lambda a: a != '', data.estimated_states['names'])
        
        # read true states
        filename= os.path.join(workspace, 'true_states.csv')
        data.true_states['values']= np.genfromtxt(filename, \
                                                       delimiter=',', \
                                                       skip_header=1)
        with open(filename, 'r') as f:
            line= f.readline().strip()
            data.true_states['names']= line.split('  ')
            # eliminate possible empty entries
            data.true_states['names']= filter(lambda a: a != '', data.true_states['names'])
        
    # return data
    return data
    

# -------------------------------------------------------
# -------------------------------------------------------
def load_params(workspace):
    '''
    Loads the parameters from the copy of params
    stored in the folder of the log
    '''

    # the full path to the params file
    filename= os.path.join(workspace, 'params.txt')
    
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

        # add the landmarks differently
        if name == 'landmark':
            x,y,z= value.split(',')
            value= [float(x), float(y), float(z)]
            if 'landmark' in params:
                    params[name].append(value)
            else:
                params[name]= []
                params[name].append(value)
                
        # for the other parameters
        else:
            if value.isdigit():
                value= float(value)

            # add to params dict
            params[name]= value

    return params


# -------------------------------------------------------
# --------------------------- MAIN ----------------------
def main():

    # Construct the argument parser
    parser= argparse.ArgumentParser()

    # Add the arguments to the parser
    parser.add_argument("-a", "--workspace", \
                         required=True, \
                         help="first operand")
    args= vars(parser.parse_args())

    # read params
    params= load_params(args['workspace'])

    # load data
    data= load_data(params['workspace'])

    # save plots 
    make_plots(data, \
               params, \
               workspace= params['workspace'], \
               residuals_plot= True, \
               variances_plot= True, \
               lir_plot= True, \
               trajectory_plot= True)

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


if __name__ == '__main__':
    main()


