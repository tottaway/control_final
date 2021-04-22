from logs.log import Log
import csv

class SensorLog(Log):
    def __init__(self, filename):
        with open(filename, mode='r') as f:
            reader = csv.DictReader(f)

            self.t = []
            self.ball_x = []
            self.ball_y = []

            for row in reader:
                self.t.append(float(row["t"]))
                self.ball_x.append(float(row["ball_x"]))
                self.ball_y.append(float(row["ball_y"]))

