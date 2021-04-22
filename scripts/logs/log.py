import matplotlib.pyplot as plt

class Log():
    def __init__(self):
        print("You should not instatiate the Log class directly, instead use" +
              " one of the subclasses")
        exit()

    def make_plots(self, plots, show=True):
        for plot in plots:
            if not hasattr(self, plot):
                print("Warning: skipping invalid plot {} for log_type: {}"
                        .format(plot, type(plot)))
                continue
            
            plt.plot(self.t, getattr(self, plot))
            if show:
                plt.show()
