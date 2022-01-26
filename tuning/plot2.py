import argparse

import matplotlib.pyplot as plt
import numpy as np

import cfusdlog

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    args = parser.parse_args()

    data_usd = cfusdlog.decode(args.file_usd)

    start_time = np.inf
    end_time = 0
    for _,v in data_usd.items():
        start_time = min(start_time, v['timestamp'][0])
        end_time = max(end_time, v['timestamp'][-1])

    time_fF = (data_usd['fixedFrequency']['timestamp'] - start_time) / 1e3

    T = len(data_usd['fixedFrequency']['timestamp'])

    # ep = np.array([
    #     data_usd['fixedFrequency']['ctrlMel.ep_x'],
    #     data_usd['fixedFrequency']['ctrlMel.ep_y'],
    #     data_usd['fixedFrequency']['ctrlMel.ep_z'],
    # ]).T

    # ev = np.array([
    #     data_usd['fixedFrequency']['ctrlMel.ev_x'],
    #     data_usd['fixedFrequency']['ctrlMel.ev_y'],
    #     data_usd['fixedFrequency']['ctrlMel.ev_z'],
    # ]).T

    # eR = np.array([
    #     data_usd['fixedFrequency']['ctrlMel.eR_x'],
    #     data_usd['fixedFrequency']['ctrlMel.eR_y'],
    #     data_usd['fixedFrequency']['ctrlMel.eR_z'],
    # ]).T

    # ew = np.array([
    #     data_usd['fixedFrequency']['ctrlMel.ew_x'],
    #     data_usd['fixedFrequency']['ctrlMel.ew_y'],
    #     data_usd['fixedFrequency']['ctrlMel.ew_z'],
    # ]).T

    time_eP = (data_usd['estPosition']['timestamp'] - start_time) / 1e3
    stats = np.diff(time_eP)
    print("mocap stats", np.min(stats), np.max(stats), np.mean(stats))

    pos_mocap = np.array([
        data_usd['estPosition']['locSrv.x'],
        data_usd['estPosition']['locSrv.y'],
        data_usd['estPosition']['locSrv.z'],
    ]).T

    pos_ekf = np.array([
        data_usd['fixedFrequency']['stateEstimateZ.x'],
        data_usd['fixedFrequency']['stateEstimateZ.y'],
        data_usd['fixedFrequency']['stateEstimateZ.z'],
    ]).T / 1000.0

    vel_ekf = np.array([
        data_usd['fixedFrequency']['stateEstimateZ.vx'],
        data_usd['fixedFrequency']['stateEstimateZ.vy'],
        data_usd['fixedFrequency']['stateEstimateZ.vz'],
    ]).T / 1000.0

    time_cFS = (data_usd['cmdFullState']['timestamp'] - start_time) / 1e3
    stats = np.diff(time_cFS)
    print("cmd stats", np.min(stats), np.max(stats), np.mean(stats))

    pos_desired = np.array([
        data_usd['cmdFullState']['ctrltarget.x'],
        data_usd['cmdFullState']['ctrltarget.y'],
        data_usd['cmdFullState']['ctrltarget.z'],
    ]).T

    vel_desired = np.array([
        data_usd['cmdFullState']['ctrltarget.vx'],
        data_usd['cmdFullState']['ctrltarget.vy'],
        data_usd['cmdFullState']['ctrltarget.vz'],
    ]).T

    fig, ax = plt.subplots(3, 3, sharex='all', sharey='row')
    ax[0,0].plot(time_fF, pos_ekf[:,0], label='EKF')
    ax[0,0].scatter(time_eP, pos_mocap[:,0], label='mocap')
    ax[0,0].scatter(time_cFS, pos_desired[:,0], label='desired')
    ax[0,1].plot(time_fF, pos_ekf[:,1], label='EKF')
    ax[0,1].scatter(time_eP, pos_mocap[:,1], label='mocap')
    ax[0,1].scatter(time_cFS, pos_desired[:,1], label='desired')
    ax[0,2].plot(time_fF, pos_ekf[:,2], label='EKF')
    ax[0,2].scatter(time_eP, pos_mocap[:,2], label='mocap')
    ax[0,2].scatter(time_cFS, pos_desired[:,2], label='desired')
    ax[0,0].set_ylabel('Position [m]')
    ax[0,0].legend()

    ax[1,0].plot(time_fF, vel_ekf[:,0], label='EKF')
    ax[1,0].scatter(time_cFS, vel_desired[:,0], label='desired')
    ax[1,1].plot(time_fF, vel_ekf[:,1], label='EKF')
    ax[1,1].scatter(time_cFS, vel_desired[:,1], label='desired')
    ax[1,2].plot(time_fF, vel_ekf[:,2], label='EKF')
    ax[1,2].scatter(time_cFS, vel_desired[:,2], label='desired')
    ax[1,0].set_ylabel('Velocity [m/s]')
    ax[1,0].legend()

    common_time = np.arange(0, (end_time - start_time)/1e3, 0.01)
    for i in range(3):
        p1 = np.interp(common_time, time_fF, pos_ekf[:,i])
        p2 = np.interp(common_time, time_cFS, pos_desired[:,i])
        e = p1-p2
        ax[2,i].plot(common_time, e)

    ax[2,0].set_ylabel('Position Error [m]')
    ax[2,0].legend()


    # ax[1,0].plot(time_fF, ep[:,0], label='ep.x')
    # ax[1,1].plot(time_fF, ep[:,1], label='ep.y')
    # ax[1,2].plot(time_fF, ep[:,2], label='ep.z')
    # ax[1,0].set_ylabel('Position error [m]')

    # ax[2,0].plot(time_fF, ev[:,0], label='ev.x')
    # ax[2,1].plot(time_fF, ev[:,1], label='ev.y')
    # ax[2,2].plot(time_fF, ev[:,2], label='ev.z')
    # ax[2,0].set_ylabel('Velocity error [m/s]')

    # ax[3,0].plot(time_fF, eR[:,0], label='eR.x')
    # ax[3,1].plot(time_fF, eR[:,1], label='eR.y')
    # ax[3,2].plot(time_fF, eR[:,2], label='eR.z')
    # ax[3,0].set_ylabel('Angular error [?]')

    # ax[4,0].plot(time_fF, ew[:,0], label='ew.x')
    # ax[4,1].plot(time_fF, ew[:,1], label='ew.y')
    # ax[4,2].plot(time_fF, ew[:,2], label='ew.z')
    # ax[4,0].set_ylabel('Angular velocity error [rad/s]')

    plt.show()

