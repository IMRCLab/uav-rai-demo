import argparse

import matplotlib.pyplot as plt
import numpy as np
import rowan

import cfusdlog

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    args = parser.parse_args()

    data_usd = cfusdlog.decode(args.file_usd)

    start_time = np.inf
    for _,v in data_usd.items():
        start_time = min(start_time, v['timestamp'][0])

    time_fF = (data_usd['fixedFrequency']['timestamp'] - start_time) / 1e3

    T = len(data_usd['fixedFrequency']['timestamp'])

    ep = np.array([
        data_usd['fixedFrequency']['ctrlMel.ep_x'],
        data_usd['fixedFrequency']['ctrlMel.ep_y'],
        data_usd['fixedFrequency']['ctrlMel.ep_z'],
    ]).T

    ev = np.array([
        data_usd['fixedFrequency']['ctrlMel.ev_x'],
        data_usd['fixedFrequency']['ctrlMel.ev_y'],
        data_usd['fixedFrequency']['ctrlMel.ev_z'],
    ]).T

    eR = np.array([
        data_usd['fixedFrequency']['ctrlMel.eR_x'],
        data_usd['fixedFrequency']['ctrlMel.eR_y'],
        data_usd['fixedFrequency']['ctrlMel.eR_z'],
    ]).T

    ew = np.array([
        data_usd['fixedFrequency']['ctrlMel.ew_x'],
        data_usd['fixedFrequency']['ctrlMel.ew_y'],
        data_usd['fixedFrequency']['ctrlMel.ew_z'],
    ]).T

    fig, ax = plt.subplots(4, 3, sharex='all', sharey='row')
    ax[0,0].plot(time_fF, ep[:,0], label='ep.x')
    ax[0,1].plot(time_fF, ep[:,1], label='ep.y')
    ax[0,2].plot(time_fF, ep[:,2], label='ep.z')
    ax[0,0].set_ylabel('Position error [m]')

    ax[1,0].plot(time_fF, ev[:,0], label='ev.x')
    ax[1,1].plot(time_fF, ev[:,1], label='ev.y')
    ax[1,2].plot(time_fF, ev[:,2], label='ev.z')
    ax[1,0].set_ylabel('Velocity error [m/s]')

    ax[2,0].plot(time_fF, eR[:,0], label='eR.x')
    ax[2,1].plot(time_fF, eR[:,1], label='eR.y')
    ax[2,2].plot(time_fF, eR[:,2], label='eR.z')
    ax[2,0].set_ylabel('Angular error [?]')

    ax[3,0].plot(time_fF, ew[:,0], label='ew.x')
    ax[3,1].plot(time_fF, ew[:,1], label='ew.y')
    ax[3,2].plot(time_fF, ew[:,2], label='ew.z')
    ax[3,0].set_ylabel('Angular velocity error [rad/s]')

    # ax[0].legend()

    # ax[1].plot(time_fF, np.degrees(mocap_rpy[:, 1]), label='mocap')
    # ax[1].plot(time_fF, np.degrees(cf_rpy[:, 1]), label='state estimate')

    # ax[2].plot(time_fF, np.degrees(mocap_rpy[:, 2]), label='mocap')
    # ax[2].plot(time_fF, np.degrees(cf_rpy[:, 2]), label='state estimate')

    plt.legend()

    plt.show()

