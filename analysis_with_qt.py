import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QLabel,
    QWidget,
    QScrollArea,
    QHBoxLayout,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

def analyze_data(file_path, percentiles, delimiter=','):
    # Data analysis function remains unchanged
    try:
        data = np.loadtxt(file_path, delimiter=delimiter, comments="#")
        column_opt_time = data[:, 8]
        median = np.median(column_opt_time)
        std_dev = np.std(column_opt_time)
        tail_percentiles = [90, 99]
        tail_values = {p: np.percentile(column_opt_time, p) for p in tail_percentiles}

        cumulative_percentile_means = []
        cumulative_percentile_values = np.percentile(column_opt_time, percentiles)
        for upper_bound in cumulative_percentile_values:
            cumulative_range_data = column_opt_time[column_opt_time <= upper_bound]
            mean = np.mean(cumulative_range_data) if cumulative_range_data.size > 0 else float("nan")
            cumulative_percentile_means.append(mean)

        return {
            "data": data,
            "median": median,
            "std_dev": std_dev,
            "tail_values": tail_values,
            "cumulative_percentile_means": cumulative_percentile_means,
            "cumulative_percentile_values": cumulative_percentile_values,
            "percentiles": percentiles,
        }
    except Exception as e:
        print(f"Error analyzing data: {e}")
        return None


class AnalysisApp(QMainWindow):
    def __init__(self, results):
        super().__init__()
        self.results = results
        self.setWindowTitle("Data Analysis and Visualization")

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # Main Layout
        main_layout = QVBoxLayout(self.central_widget)

        # Scroll Area for Metrics
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)

        # Add metrics to the scroll area
        scroll_layout.addWidget(QLabel(f"Optimization Time : Median: {self.results['median']:.2f} msecs"))
        scroll_layout.addWidget(QLabel(f"Optimization Time : Standard Deviation: {self.results['std_dev']:.2f} msecs"))
        for p, value in self.results["tail_values"].items():
            scroll_layout.addWidget(QLabel(f"{p}th Percentile : {value:.2f} msecs"))
        scroll_layout.addWidget(QLabel("Optimization Time : Cumulative Percentile Means:"))
        for i, mean in enumerate(self.results["cumulative_percentile_means"]):
            scroll_layout.addWidget(QLabel(f"0-{self.results['percentiles'][i]} percentile: Mean = {mean:.2f} msecs"))

        main_layout.addWidget(scroll_area)

        # Add Plot Canvas with Toolbar
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)

        self.canvas = FigureCanvas(self.create_plots())
        self.toolbar = NavigationToolbar(self.canvas, self)

        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)

        main_layout.addWidget(plot_widget)

    def create_plots(self):
        fig = Figure(figsize=(10, 8))
        axs = fig.subplots(2, 2).flatten()

        column_counter = self.results["data"][:, 0]
        column_ego_speed = self.results["data"][:, 6]
        column_est_lead_car_speed = self.results["data"][:, 2]
        column_est_following_dist = self.results["data"][:, 1]
        column_accel_command = self.results["data"][:, 3]
        column_opt_time = self.results["data"][:, 8]
        column_speed_limit = np.full(column_counter.size, 29.058)
        column_accel_up_limit = np.full(column_counter.size, 2.00)
        column_accel_low_limit = np.full(column_counter.size, -4.00)

        axs[0].plot(column_counter, column_ego_speed, label='Ego speed (in m/s)', color='blue', linewidth=2.5, markersize=8)
        axs[0].plot(column_counter, column_est_lead_car_speed, label='Est LeadCar speed (in m/s) w Noise', color='green', linewidth=0.5, alpha=0.5)
        axs[0].plot(column_counter, column_speed_limit, label='Speed Limit (in m/s)', color='red', linestyle='--')
        axs[0].set_title("Ego Speed & Lead Car Speed")
        axs[0].set_ylabel("Speed (in m/s)")
        axs[0].set_xlabel("TimeSteps")
        axs[0].grid(True)
        axs[0].legend()
        


        axs[1].plot(column_counter, column_est_following_dist, label='Gap between cars (in meters)', color='blue')
        # commented for debugging
        # Annotate specific points
        times = [0, 500, 1500, 3200, 5000, 6500, 8000, 9500,11000]
        for t in times:
            y_val = column_est_following_dist[t]  # Calculate y-value
            
            if t==0:
                axs[1].annotate(f'{y_val:.2f} m (Initial Gap)',  # Text to display
                        (t, y_val),      # Location of the text
                        textcoords="offset points",  # Text positioning
                        xytext=(0, 10),  # Offset in pixels (x, y)
                        ha='center',     # Horizontal alignment
                        arrowprops=dict(arrowstyle='->', color='gray'))  # Optional arrow            
            else:
                axs[1].annotate(f'{y_val:.2f} m',  # Text to display
                        (t, y_val),      # Location of the text
                        textcoords="offset points",  # Text positioning
                        xytext=(0, 10),  # Offset in pixels (x, y)
                        ha='center',     # Horizontal alignment
                        arrowprops=dict(arrowstyle='->', color='gray'))  # Optional arrow
        
        axs[1].set_title("Estimated Following Distance")
        axs[1].set_ylabel("Gap between cars (in meter)")
        axs[1].set_xlabel("TimeSteps")
        axs[1].grid(True)

        axs[2].plot(column_counter, column_accel_command, label='Accel Command (in m/ss)', color='blue')
        axs[2].plot(column_counter, column_accel_up_limit, color='red', linestyle='--')
        axs[2].plot(column_counter, column_accel_low_limit, color='red', linestyle='--')
        axs[2].set_title("Measured Accel Command with Constraints")
        axs[2].set_ylabel("Accel (in m/ss)")
        axs[2].set_xlabel("TimeSteps")
        axs[2].grid(True)

        axs[3].hist(column_opt_time, bins=15, color="g", alpha=0.7)
        axs[3].set_title("Histogram of Opt time per step")
        axs[3].set_xlabel("Time (in msecs)")
        axs[3].set_ylabel("Frequencies of Controller Steps")
        axs[3].grid(True)

        return fig


def main():
    file_path = "output/results.txt"  # Replace with your file path
    percentiles = [25, 50, 75, 100]
    results = analyze_data(file_path, percentiles)
    if not results:
        return

    app = QApplication(sys.argv)
    window = AnalysisApp(results)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
