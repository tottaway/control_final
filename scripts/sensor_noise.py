import numpy as np
import os
from tqdm import tqdm

from logs.env_log import EnvLog
from logs.sensor_log import SensorLog

def calculate_sensor_noise_helper(sensor_log, env_log):
    sensor_meas = np.array([sensor_log.ball_x, sensor_log.ball_y])

    # This is terrible
    env_meas_idx = []
    for i, t in enumerate(env_log.t):
        if t in sensor_log.t:
            env_meas_idx.append(i)

    env_meas = np.array([env_log.ball_x, env_log.ball_y])[:, env_meas_idx]

    return np.cov(sensor_meas - env_meas)

def calculate_sensor_noise(log_dir, filenames=[]):
    if len(filenames) == 0:
        filenames = os.listdir(log_dir)

    covs = []
    for filename in tqdm(filenames):
        path = os.path.join(log_dir, filename)

        if not os.path.isfile(path):
            raise Exception("How?")
        
        if filename.startswith("sensor_"):
            sensor_log = SensorLog(path)
            env_path = path.replace("sensor", "env")
            if not os.path.isfile(path):
                raise Exception("There should always be a matching env file")
            env_log = EnvLog(env_path)
            if env_log.is_stable():
                covs.append(calculate_sensor_noise_helper(sensor_log, env_log))

    average_cov = sum(covs) / len(covs)

    print(average_cov)
