import csv
import matplotlib.pyplot as plt
import argparse

from logs.env_log import EnvLog
from logs.sensor_log import SensorLog
from logs.controller_log import ControllerLog

from pid_stats import pid_stats
from sensor_noise import calculate_sensor_noise
from inner_loop import inner_pd

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor_noise", action="store_true", default=False)
    parser.add_argument("--inner_pd", action="store_true", default=False)
    parser.add_argument("--pid_stats", action="store_true", default=False)
    parser.add_argument("--log_dir", action="store", default="")
    parser.add_argument("--plot_file", nargs="+", action="store", default="")
    parser.add_argument("--plots", nargs="+", action="store", default=[],
            help="Determines which plots to make")
    args = parser.parse_args()
    
    if len(args.plots) > 0:
        for file_name in args.plot_file:
            log_type = file_name.split("/")[-1].split("_")[0]
            if log_type == "env":
                log = EnvLog(file_name)
            elif log_type == "controller":
                log = ControllerLog(file_name)
            elif log_type == "sensor":
                log = SensorLog(file_name)
            else:
                print("Invalid log_type {}".format(log_type))
                exit()

            log.make_plots(args.plots, show=False)
        
        plt.legend()
        plt.show()

    if (args.pid_stats):
        if args.log_dir == "":
            print("Failed to specify log_dir to generate pid stats from")
            exit()
        pid_stats(args.log_dir)

    if (args.sensor_noise):
        if args.log_dir == "":
            print("Failed to specify log_dir to calculate sensor noise")
            exit()
        calculate_sensor_noise(args.log_dir)

    if (args.inner_pd):
        if args.log_dir == "":
            print("Failed to specify log_dir to generate inner loop stats from")
            exit()
        inner_pd(args.log_dir)



