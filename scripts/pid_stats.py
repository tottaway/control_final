import os
import matplotlib.pyplot as plt

from logs.env_log import EnvLog
from tqdm import tqdm


def pid_stats(log_dir):
    env_logs = []
    best_file = ""
    best_settling_time = float("inf")
    best_env_log = None
    for filename in tqdm(os.listdir(log_dir)):
        path = os.path.join(log_dir, filename)

        if not os.path.isfile(path):
            raise Exception("How?")
        
        if filename.startswith("env_"):
           env_log = EnvLog(path)

           if env_log.is_stable():
               settling_time = env_log.get_settling_time()
               if settling_time < best_settling_time:
                   best_settling_time = settling_time
                   best_file = filename
                   best_env_log = env_log

    print(best_file)
    print(best_settling_time)
    best_env_log.make_plots(["ball_x", "ball_y"], show=False)
    plt.show()
