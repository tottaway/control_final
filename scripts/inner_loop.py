import os
from tqdm import tqdm

from logs.env_log import EnvLog
from logs.controller_log import ControllerLog
import matplotlib.pyplot as plt


def calc_overshoot(env_log, controller_log):
    # assumes that table starts from zeros and goes positive in both x and y
    max_x = max(env_log.table_theta_x)
    max_y = max(env_log.table_theta_y)

    return max(max_x - controller_log.ref_theta_x[-1],
               max_y - controller_log.ref_theta_y[-1])

def calc_settling_time(env_log, controller_log):
    settling_time_x = float('inf')
    settling_time_y = float('inf')

    settled = False
    x_thresh = 0.01
    for t, x, ref_x in zip(env_log.t, env_log.table_theta_x, controller_log.ref_theta_x):
        if abs(x - ref_x) > x_thresh:
            settled = False
            settling_time_x = float('inf')
        else:
            if not settled:
                settling_time_x = t
            settled = True

    settled = False
    y_thresh = 0.01
    for t, y, ref_y in zip(env_log.t, env_log.table_theta_y, controller_log.ref_theta_y):
        if abs(y - ref_y) > y_thresh:
            settled = False
            settling_time_y = float('inf')
        else:
            if not settled:
                settling_time_y = t
            settled = True

    return max(settling_time_x, settling_time_y)



def inner_pd(log_dir):
    best_file = ""
    best_settling_time = float("inf")
    best_env_log = None
    best_controller_log = None
    for filename in tqdm(os.listdir(log_dir)):
        path = os.path.join(log_dir, filename)

        if not os.path.isfile(path):
            raise Exception("How?")
        
        if filename.startswith("env_"):
            env_log = EnvLog(path)

            controller_path = path.replace("env", "controller")
            if not os.path.isfile(path):
                raise Exception("There should always be a matching controller file")
            controller_log = ControllerLog(controller_path)
            
            overshoot = calc_overshoot(env_log, controller_log)

            if overshoot < 0.05:
               settling_time = calc_settling_time(env_log, controller_log)
               if settling_time < best_settling_time:
                   best_settling_time = settling_time
                   best_file = filename
                   best_env_log = env_log
                   best_controller_log = controller_log

    print(best_file)
    print(best_settling_time)
    best_env_log.make_plots(["table_theta_x", "table_theta_y"], show=False)
    plt.show()
