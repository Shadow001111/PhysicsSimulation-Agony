import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class BenchmarkViewerApp:
    METRIC_COLUMNS = {
        "Mean": "Mean_us",
        "Median": "Median_us",
        "P90": "P90_us",
        "P99": "P99_us",
        "Min": "Min_us",
        "Max": "Max_us",
        "StdDev": "StdDev_us",
    }

    def __init__(self, root):
        self.root = root
        self.root.title("Collision Detector Benchmark Viewer")
        self.root.geometry("1100x750")

        self.current_metric = tk.StringVar(value="Mean")
        self.parsed_data = None
        self.density_tabs = {}

        # --- Top Control Panel ---
        self.control_frame = tk.Frame(self.root, bg="#f0f0f0", bd=1, relief=tk.RAISED)
        self.control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        self.load_btn = tk.Button(
            self.control_frame,
            text="Load Benchmark CSV",
            command=self.load_csv,
            font=("Arial", 11, "bold"),
            bg="#007acc",
            fg="white",
            padx=10,
            pady=5
        )
        self.load_btn.pack(side=tk.LEFT, padx=5, pady=5)

        tk.Label(
            self.control_frame,
            text="Metric:",
            bg="#f0f0f0",
            font=("Arial", 10, "bold")
        ).pack(side=tk.LEFT, padx=(15, 5))

        self.metric_box = ttk.Combobox(
            self.control_frame,
            textvariable=self.current_metric,
            values=list(self.METRIC_COLUMNS.keys()),
            state="readonly",
            width=10
        )
        self.metric_box.pack(side=tk.LEFT, padx=5, pady=5)
        self.metric_box.bind("<<ComboboxSelected>>", lambda e: self.refresh_plots())

        self.file_label = tk.Label(
            self.control_frame,
            text="No file loaded. Please click the button to select your benchmark CSV.",
            font=("Arial", 10, "italic"),
            fg="#666666",
            bg="#f0f0f0"
        )
        self.file_label.pack(side=tk.LEFT, padx=15, pady=5)

        # --- Main Container ---
        self.main_frame = tk.Frame(self.root, bg="white")
        self.main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Notebook for arbitrary densities
        self.notebook = ttk.Notebook(self.main_frame)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.reset_view()

    def reset_view(self):
        # Clear all tabs
        for tab in self.notebook.winfo_children():
            tab.destroy()
        self.density_tabs.clear()

        empty_tab = tk.Frame(self.notebook, bg="white")
        empty_tab.pack(fill=tk.BOTH, expand=True)
        self.notebook.add(empty_tab, text="No Data")

    def load_csv(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if not file_path:
            return

        try:
            parsed_data = {}

            with open(file_path, mode="r", newline="", encoding="utf-8") as f:
                clean_lines = (line for line in f if not line.startswith("="))
                reader = csv.DictReader(clean_lines)

                if not reader.fieldnames:
                    raise ValueError("CSV header is missing.")

                reader.fieldnames = [h.strip() for h in reader.fieldnames]

                required = {"BodyCount", "Density", "Threading"}
                if not required.issubset(set(reader.fieldnames)):
                    raise ValueError(f"CSV header format mismatch. Expected at least: {sorted(required)}")

                has_full_stats = all(col in reader.fieldnames for col in self.METRIC_COLUMNS.values())
                has_avg_only = "AvgTime_us" in reader.fieldnames

                if not has_full_stats and not has_avg_only:
                    raise ValueError(
                        "CSV must contain either the full stats columns "
                        "(Mean_us, Median_us, P90_us, P99_us, Min_us, Max_us, StdDev_us) "
                        "or at least AvgTime_us."
                    )

                for row in reader:
                    density = row["Density"].strip()
                    threading_raw = row["Threading"].strip().lower()
                    threading = "Threaded" if "thread" in threading_raw else "Single"

                    body_count = int(row["BodyCount"].strip())

                    if density not in parsed_data:
                        parsed_data[density] = {
                            "Single": {},
                            "Threaded": {}
                        }

                    if has_full_stats:
                        for metric_name, col_name in self.METRIC_COLUMNS.items():
                            value = float(row[col_name].strip())
                            parsed_data[density][threading].setdefault(metric_name, []).append((body_count, value))
                    else:
                        value = float(row["AvgTime_us"].strip())
                        parsed_data[density][threading].setdefault("Mean", []).append((body_count, value))

            # Sort all series by BodyCount
            for density in parsed_data:
                for threading in parsed_data[density]:
                    for metric_name in parsed_data[density][threading]:
                        parsed_data[density][threading][metric_name].sort(key=lambda item: item[0])

            self.parsed_data = parsed_data
            self.file_label.config(
                text=f"Active File: {file_path}",
                fg="#006600",
                font=("Arial", 10, "bold")
            )
            self.rebuild_tabs()
            self.refresh_plots()

        except Exception as e:
            messagebox.showerror("Parsing Failure", f"An error occurred while inspecting the CSV file:\n\n{str(e)}")

    def rebuild_tabs(self):
        for tab in self.notebook.winfo_children():
            tab.destroy()
        self.density_tabs.clear()

        if not self.parsed_data:
            self.reset_view()
            return

        for density in sorted(self.parsed_data.keys()):
            tab = tk.Frame(self.notebook, bg="white")
            tab.pack(fill=tk.BOTH, expand=True)
            self.notebook.add(tab, text=density)

            fig, ax = plt.subplots(figsize=(10.5, 5.5))
            fig.patch.set_facecolor("#ffffff")

            canvas = FigureCanvasTkAgg(fig, master=tab)
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

            self.density_tabs[density] = {
                "frame": tab,
                "fig": fig,
                "ax": ax,
                "canvas": canvas
            }

    def refresh_plots(self):
        if not self.parsed_data:
            self.reset_view()
            return

        metric = self.current_metric.get()

        def split_xy(points):
            if not points:
                return [], []
            x, y = zip(*points)
            return x, y

        for density, widgets in self.density_tabs.items():
            ax = widgets["ax"]
            canvas = widgets["canvas"]

            ax.clear()

            single_points = self.parsed_data.get(density, {}).get("Single", {}).get(metric, [])
            threaded_points = self.parsed_data.get(density, {}).get("Threaded", {}).get(metric, [])

            s_x, s_y = split_xy(single_points)
            t_x, t_y = split_xy(threaded_points)

            if s_x:
                ax.plot(
                    s_x, s_y,
                    label=f"Single-Threaded ({metric})",
                    color="#1f77b4",
                    linestyle="--",
                    marker="o",
                    linewidth=2
                )

            if t_x:
                ax.plot(
                    t_x, t_y,
                    label=f"Multi-Threaded ({metric})",
                    color="#ff7f0e",
                    linestyle="-",
                    marker="s",
                    linewidth=2
                )

            ax.set_title(f"{density} — {metric}", fontsize=13, fontweight="bold", color="#2c3e50", pad=12)
            ax.set_xlabel("Total Scene Bodies", fontsize=10, fontweight="bold")
            ax.set_ylabel("Latency (μs)", fontsize=10, fontweight="bold")
            ax.grid(True, which="both", linestyle=":", alpha=0.6)

            if s_x or t_x:
                ax.legend(frameon=True, facecolor="#fcfcfc", edgecolor="#dddddd")

            fig = widgets["fig"]
            fig.tight_layout(pad=3.0)
            canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = BenchmarkViewerApp(root)
    root.mainloop()