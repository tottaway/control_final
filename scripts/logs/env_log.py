from logs.log import Log
import csv

class EnvLog(Log):
    def __init__(self, filename):
        with open(filename, mode='r') as f:
            reader = csv.DictReader(f)

            self.t = []
            self.ball_x = []
            self.ball_y = []
            self.ball_z = []
            self.ball_xdot = []
            self.ball_ydot = []
            self.ball_zdot = []
            self.ball_aor = []
            self.ball_omega = []

            self.table_x = []
            self.table_y = []
            self.table_z = []
            self.table_xdot = []
            self.table_ydot = []
            self.table_zdot = []
            self.table_theta_x = []
            self.table_theta_y = []
            self.table_theta_dot_x = []
            self.table_theta_dot_y = []

            for row in reader:
                self.t.append(float(row["t"]))
                self.ball_x.append(float(row["ball_x"]))
                self.ball_y.append(float(row["ball_y"]))
                self.ball_z.append(float(row["ball_z"]))

                self.ball_xdot.append(float(row["ball_xdot"]))
                self.ball_ydot.append(float(row["ball_ydot"]))
                self.ball_zdot.append(float(row["ball_zdot"]))

                self.ball_zdot.append([
                    float(row["ball_aor_x"]),
                    float(row["ball_aor_y"]),
                    float(row["ball_aor_z"])
                ])

                self.ball_omega.append(float(row["ball_omega"]))

                self.table_x.append(float(row["table_x"]))
                self.table_y.append(float(row["table_y"]))
                self.table_z.append(float(row["table_z"]))
                self.table_xdot.append(float(row["table_xdot"]))
                self.table_ydot.append(float(row["table_ydot"]))
                self.table_zdot.append(float(row["table_zdot"]))
                self.table_theta_x.append(float(row["table_theta_x"]))
                self.table_theta_y.append(float(row["table_theta_y"]))
                self.table_theta_dot_x.append(float(row["table_theta_dot_x"]))
                self.table_theta_dot_y.append(float(row["table_theta_dot_y"]))

    def get_x_rise_time(self):
        for t, x in zip(self.t, self.ball_x):
            if x > -0.02:
                return t
        return float('inf')

    def get_y_rise_time(self):
        for t, y in zip(self.t, self.ball_y):
            if y < 0.02:
                return t
        return float('inf')

    def get_x_max(self):
        return max(self.ball_x)

    def get_y_max(self):
        return max(self.ball_y)

    def get_x_min(self):
        return min(self.ball_x)

    def get_y_min(self):
        return min(self.ball_y)

    def get_settling_time(self):
        settling_time_x = float('inf')
        settling_time_y = float('inf')

        settled = False
        x_thresh_low = -0.01
        x_thresh_high = 0.01
        for t, x in zip(self.t, self.ball_x):
            if x < x_thresh_low or x > x_thresh_high:
                settled = False
                settling_time_x = float('inf')
            else:
                if not settled:
                    settling_time_x = t
                settled = True

        settled = False
        y_thresh_low = -0.01
        y_thresh_high = 0.01
        for t, y in zip(self.t, self.ball_y):
            if y < y_thresh_low or y > y_thresh_high:
                settled = False
                settling_time_y = float('inf')
            else:
                if not settled:
                    settling_time_y = t
                settled = True

        return max(settling_time_x, settling_time_y)


    def is_stable(self):
        return (self.get_x_max() <= 0.4 and
                self.get_x_min() >= -0.4 and
                self.get_y_max() <= 0.4 and
                self.get_y_min() >= -0.4)

