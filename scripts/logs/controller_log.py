from logs.log import Log
import csv

class ControllerLog(Log):
    def __init__(self, filename):
        with open(filename, mode='r') as f:
            reader = csv.DictReader(f)

            self.t = []
            self.torque_x = []
            self.torque_y = []
            self.ref_theta_x = []
            self.ref_theta_y = []

            for row in reader:
                self.t.append(float(row["t"]))
                self.torque_x.append(float(row["torque_x"]))
                self.torque_y.append(float(row["torque_y"]))
                self.ref_theta_x.append(float(row["ref_theta_x"]))
                self.ref_theta_y.append(float(row["ref_theta_y"]))

