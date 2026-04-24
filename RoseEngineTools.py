import math
import json
import importlib
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    from svg.path import parse_path
except ImportError:
    parse_path = None

try:
    serial = importlib.import_module("serial")
    list_ports = importlib.import_module("serial.tools.list_ports")
except Exception:
    serial = None
    list_ports = None


# Rotate the angular reference so 0° is at the top of the plot.
ANGLE_OFFSET_DEG = 90.0
GUI_BACKGROUND_COLOR = "#dff1ff"


class RosetteSvgViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Rose Engine Tools")
        #self.geometry("1150x760")
        self.minsize(960, 600)
        self.configure(bg=GUI_BACKGROUND_COLOR)

        self._centered_polylines = None
        self._serial_conn = None
        self._serial_poll_job = None
        self._serial_rx_buffer = ""
        self._gcode_send_queue = []
        self._gcode_send_waiting_ok = False
        self._gcode_send_total = 0
        self._gcode_send_sent = 0
        self._settings_path = Path(__file__).with_name("settings.json")
        self._settings = self._load_settings()
        self.invert_z_var = tk.BooleanVar(value=bool(self._settings.get("invert_z_direction", False)))
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._configure_theme()
        self._build_ui()
        self._plot_empty_state()

    def _configure_theme(self):
        style = ttk.Style(self)
        style.configure("TFrame", background=GUI_BACKGROUND_COLOR)
        style.configure("TLabelframe", background=GUI_BACKGROUND_COLOR)
        style.configure("TLabelframe.Label", background=GUI_BACKGROUND_COLOR)
        style.configure("TLabel", background=GUI_BACKGROUND_COLOR)
        style.configure("TNotebook", background=GUI_BACKGROUND_COLOR)

    def _build_ui(self):
        outer = ttk.Frame(self)
        outer.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(outer, highlightthickness=0, bg=GUI_BACKGROUND_COLOR)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        container = ttk.Frame(canvas, padding=10)

        container.bind(
            "<Configure>",
            lambda event: canvas.configure(scrollregion=canvas.bbox("all")),
        )

        container_window = canvas.create_window((0, 0), window=container, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(container_window, width=event.width),
        )

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # ── Tab notebook ──────────────────────────────────────────────────────
        notebook = ttk.Notebook(container)
        notebook.pack(fill=tk.BOTH, expand=True)

        svg_tab      = ttk.Frame(notebook)
        gcode_tab    = ttk.Frame(notebook)
        serial_tab     = ttk.Frame(notebook)
        reciprocator_tab = ttk.Frame(notebook)
        plunge_tab = ttk.Frame(notebook)
        spherical_sliderest_tab = ttk.Frame(notebook)
        settings_tab = ttk.Frame(notebook)

        notebook.add(svg_tab,      text="Rosette")
        notebook.add(reciprocator_tab, text="Reciprocator")
        notebook.add(plunge_tab, text="Plunge")
        notebook.add(spherical_sliderest_tab, text="Spherical Sliderest")
        notebook.add(gcode_tab,    text="gCode Gen")
        notebook.add(serial_tab,     text="Serial Terminal")
        notebook.add(settings_tab, text="Settings")

        self._build_svg_tab(svg_tab)
        self._build_gcode_tab(gcode_tab)
        self._build_serial_tab(serial_tab)
        self._build_reciprocator_tab(reciprocator_tab)
        self._build_plunge_tab(plunge_tab)
        self._build_spherical_sliderest_tab(spherical_sliderest_tab)
        self._build_settings_tab(settings_tab)

    def _build_placeholder_tab(self, parent, name):
        ttk.Label(parent, text=f"{name} — coming soon", font=("TkDefaultFont", 11)).pack(
            expand=True
        )

    def _build_plunge_tab(self, parent):
        frame = ttk.Frame(parent, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_rowconfigure(0, weight=1)

        controls = ttk.LabelFrame(frame, text="Plunge Controls", padding=10, width=320)
        controls.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        controls.grid_propagate(False)

        def add_labeled_entry(parent_widget, label_text, text_var, pady=(10, 0)):
            row = ttk.Frame(parent_widget)
            row.pack(fill=tk.X, pady=pady)
            ttk.Label(row, text=label_text).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=text_var, width=14).pack(side=tk.RIGHT)

        self.plunge_total_depth_var = tk.StringVar(value="1.0")
        add_labeled_entry(controls, "Total depth", self.plunge_total_depth_var, pady=(0, 0))

        self.plunge_step_depth_var = tk.StringVar(value="0.1")
        add_labeled_entry(controls, "Step depth", self.plunge_step_depth_var)

        self.plunge_retract_depth_var = tk.StringVar(value="0.2")
        add_labeled_entry(controls, "Retract depth", self.plunge_retract_depth_var)

        self.plunge_safe_z_var = tk.StringVar(value="4.0")
        add_labeled_entry(controls, "Safe Z", self.plunge_safe_z_var)

        self.plunge_x_advance_var = tk.StringVar(value="1.0")
        add_labeled_entry(controls, "X advance", self.plunge_x_advance_var)

        self.plunge_number_of_cuts_var = tk.StringVar(value="10")
        add_labeled_entry(controls, "Number of cuts", self.plunge_number_of_cuts_var)

        self.plunge_feedrate_var = tk.StringVar(value="200")
        add_labeled_entry(controls, "Feedrate", self.plunge_feedrate_var)

        self.plunge_number_of_divisions_var = tk.StringVar(value="1")
        add_labeled_entry(controls, "Number of divisions", self.plunge_number_of_divisions_var)

        self.plunge_rotate_repeat_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Rotate & Repeat",
            variable=self.plunge_rotate_repeat_var,
            command=self._update_plunge_preview,
        ).pack(anchor=tk.W, pady=(10, 0))

        ttk.Button(controls, text="Generate gCode", command=self._on_generate_plunge_gcode).pack(
            fill=tk.X, pady=(12, 0)
        )

        preview_frame = ttk.LabelFrame(frame, text="Cut Preview", padding=8)
        preview_frame.grid(row=0, column=1, sticky="nsew")
        preview_frame.grid_rowconfigure(0, weight=1)
        preview_frame.grid_columnconfigure(0, weight=1)

        self.plunge_figure = Figure(figsize=(9, 5), dpi=100)
        self.plunge_ax = self.plunge_figure.add_subplot(121)
        self.plunge_polar_ax = self.plunge_figure.add_subplot(122, projection="polar")
        self.plunge_canvas = FigureCanvasTkAgg(self.plunge_figure, master=preview_frame)
        self.plunge_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        for variable in (
            self.plunge_total_depth_var,
            self.plunge_step_depth_var,
            self.plunge_retract_depth_var,
            self.plunge_safe_z_var,
            self.plunge_x_advance_var,
            self.plunge_number_of_cuts_var,
            self.plunge_number_of_divisions_var,
        ):
            variable.trace_add("write", self._update_plunge_preview)
        self._update_plunge_preview()

    def _get_plunge_parameters(self):
        total_depth = float(self.plunge_total_depth_var.get())
        step_depth = float(self.plunge_step_depth_var.get())
        retract_depth = float(self.plunge_retract_depth_var.get())
        safe_z = float(self.plunge_safe_z_var.get())
        x_advance = float(self.plunge_x_advance_var.get())
        number_of_cuts = int(self.plunge_number_of_cuts_var.get())
        feedrate = float(self.plunge_feedrate_var.get())
        division_count = int(self.plunge_number_of_divisions_var.get())
        if (
            total_depth <= 0.0
            or step_depth <= 0.0
            or retract_depth < 0.0
            or safe_z < 0.0
            or x_advance < 0.0
            or number_of_cuts < 1
            or feedrate <= 0.0
            or division_count < 1
        ):
            raise ValueError
        return total_depth, step_depth, retract_depth, safe_z, x_advance, number_of_cuts, feedrate, division_count

    def _build_plunge_depth_levels(self, total_depth, step_depth):
        depth_levels = []
        current_depth = step_depth
        while current_depth < total_depth:
            depth_levels.append(current_depth)
            current_depth += step_depth
        depth_levels.append(total_depth)
        return depth_levels

    def _update_plunge_preview(self, *_):
        self.plunge_ax.clear()
        self.plunge_polar_ax.clear()
        self.plunge_ax.set_title("Plunge Cut Preview (X-Z)")
        self.plunge_ax.set_xlabel("X (mm)")
        self.plunge_ax.set_ylabel("Z (mm)")
        self.plunge_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.8)

        self.plunge_polar_ax.set_title("A-Axis Preview (Polar)")
        self.plunge_polar_ax.set_theta_zero_location("N")
        self.plunge_polar_ax.set_theta_direction(-1)
        self.plunge_polar_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.8)

        try:
            total_depth, step_depth, retract_depth, safe_z, x_advance, number_of_cuts, _feedrate, division_count = (
                self._get_plunge_parameters()
            )
            depth_levels = self._build_plunge_depth_levels(total_depth, step_depth)
            x_positions = [cut_index * x_advance for cut_index in range(number_of_cuts)]
            plunge_points_z = [-depth for depth in depth_levels]
            rotate_repeat = self.plunge_rotate_repeat_var.get()
            cut_iterations = division_count if rotate_repeat else 1
            division_angle = 360.0 / float(division_count)

            for x_target in x_positions:
                self.plunge_ax.plot([x_target, x_target], [0.0, -total_depth], color="#1f77b4", linewidth=1.6)
                self.plunge_ax.scatter(
                    [x_target] * len(plunge_points_z),
                    plunge_points_z,
                    color="#ff7f0e",
                    s=16,
                    zorder=3,
                )

            self.plunge_ax.axhline(0.0, color="#666666", linewidth=1.0)
            if retract_depth > 0.0:
                self.plunge_ax.axhline(retract_depth, color="#2ca02c", linewidth=1.0, linestyle="--", label="Retract Z")
            if safe_z > 0.0:
                self.plunge_ax.axhline(safe_z, color="#7f7f7f", linewidth=1.0, linestyle="--", label="Safe Z")

            x_extent = x_positions[-1] if x_positions else 0.0
            x_margin = max(0.5, x_advance * 0.5)
            z_top = max(safe_z, retract_depth, 0.0) + 0.5
            z_bottom = -total_depth - 0.5
            self.plunge_ax.set_xlim(-x_margin, x_extent + x_margin)
            self.plunge_ax.set_ylim(z_bottom, z_top)

            radial_positions = x_positions if x_positions else [0.0]
            if radial_positions[-1] <= 0.0:
                radial_positions = [index + 1.0 for index in range(number_of_cuts)]

            theta_values = [math.radians(iteration * division_angle) for iteration in range(cut_iterations)]
            for theta in theta_values:
                self.plunge_polar_ax.plot(
                    [theta, theta],
                    [0.0, radial_positions[-1]],
                    color="#d9d9d9",
                    linewidth=0.8,
                    zorder=1,
                )

            for radius in radial_positions:
                self.plunge_polar_ax.scatter(
                    theta_values,
                    [radius] * len(theta_values),
                    color="#1f77b4",
                    s=16,
                    zorder=3,
                )

            self.plunge_polar_ax.set_rlabel_position(135)
            self.plunge_polar_ax.set_ylim(0.0, max(radial_positions[-1], 1.0) * 1.05)
            if not rotate_repeat:
                self.plunge_polar_ax.text(
                    0.5,
                    0.05,
                    "Rotate & Repeat OFF",
                    transform=self.plunge_polar_ax.transAxes,
                    ha="center",
                    va="bottom",
                    fontsize=9,
                )

            division_text = f"{division_count} groups" if rotate_repeat else "1 group"
            self.plunge_ax.text(
                0.02,
                0.02,
                f"Cuts: {number_of_cuts}\nPasses per cut: {len(depth_levels)}\nRotate & repeat: {division_text}",
                transform=self.plunge_ax.transAxes,
                va="bottom",
                ha="left",
                fontsize=9,
                bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8, "edgecolor": "#bbbbbb"},
            )
        except Exception:
            self.plunge_ax.text(
                0.5,
                0.5,
                "Enter valid plunge values\n(cuts/divisions >= 1, positive depths)",
                transform=self.plunge_ax.transAxes,
                ha="center",
                va="center",
            )
            self.plunge_polar_ax.text(
                0.5,
                0.5,
                "Enter valid values",
                transform=self.plunge_polar_ax.transAxes,
                ha="center",
                va="center",
            )

        self.plunge_canvas.draw_idle()

    def _build_spherical_sliderest_tab(self, parent):
        frame = ttk.Frame(parent, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_rowconfigure(0, weight=1)

        controls = ttk.LabelFrame(frame, text="Spherical Slide Rest Controls", padding=10, width=320)
        controls.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        controls.grid_propagate(False)

        def add_labeled_entry(parent_widget, label_text, text_var, pady=(10, 0)):
            row = ttk.Frame(parent_widget)
            row.pack(fill=tk.X, pady=pady)
            ttk.Label(row, text=label_text).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=text_var, width=14).pack(side=tk.RIGHT)

        self.spherical_b_start_var = tk.StringVar(value="0")
        add_labeled_entry(controls, "B starting angle", self.spherical_b_start_var, pady=(0, 0))

        self.spherical_b_end_var = tk.StringVar(value="90")
        add_labeled_entry(controls, "B ending angle", self.spherical_b_end_var)

        self.spherical_radius_var = tk.StringVar(value="10")
        add_labeled_entry(controls, "Radius", self.spherical_radius_var)

        self.spherical_samples_var = tk.StringVar(value="91")
        add_labeled_entry(controls, "Samples", self.spherical_samples_var)

        ttk.Label(controls, text="Direction").pack(anchor=tk.W, pady=(10, 0))
        self.spherical_direction_var = tk.StringVar(value="L-R")
        direction_frame = ttk.Frame(controls)
        direction_frame.pack(anchor=tk.W, pady=(4, 0))
        ttk.Radiobutton(
            direction_frame,
            text="L-R",
            value="L-R",
            variable=self.spherical_direction_var,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            direction_frame,
            text="R-L",
            value="R-L",
            variable=self.spherical_direction_var,
        ).pack(side=tk.LEFT, padx=(10, 0))

        self.spherical_curve_inverted_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Invert Curve (XZ diagonal)",
            variable=self.spherical_curve_inverted_var,
            command=self._update_spherical_sliderest_preview,
        ).pack(anchor=tk.W, pady=(10, 0))

        self.spherical_suppress_b_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Suppress B axis move",
            variable=self.spherical_suppress_b_var,
        ).pack(anchor=tk.W, pady=(10, 0))

        self.spherical_depth_of_cut_var = tk.StringVar(value="0.1")
        add_labeled_entry(controls, "Depth of cut", self.spherical_depth_of_cut_var)

        self.spherical_feedrate_var = tk.StringVar(value="200")
        add_labeled_entry(controls, "Feedrate", self.spherical_feedrate_var)

        ttk.Button(
            controls,
            text="Generate gCode",
            command=self._on_generate_spherical_sliderest_gcode,
        ).pack(fill=tk.X, pady=(12, 0))

        preview_frame = ttk.LabelFrame(frame, text="Arc Preview", padding=8)
        preview_frame.grid(row=0, column=1, sticky="nsew")
        preview_frame.grid_rowconfigure(0, weight=1)
        preview_frame.grid_columnconfigure(0, weight=1)

        self.spherical_figure = Figure(figsize=(6, 5), dpi=100)
        self.spherical_ax = self.spherical_figure.add_subplot(111)
        self.spherical_canvas = FigureCanvasTkAgg(self.spherical_figure, master=preview_frame)
        self.spherical_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        for variable in (
            self.spherical_b_start_var,
            self.spherical_b_end_var,
            self.spherical_radius_var,
            self.spherical_samples_var,
            self.spherical_depth_of_cut_var,
        ):
            variable.trace_add("write", self._update_spherical_sliderest_preview)
        self.spherical_direction_var.trace_add("write", self._update_spherical_sliderest_preview)
        self._update_spherical_sliderest_preview()

    def _get_spherical_sliderest_parameters(self):
        start_angle = float(self.spherical_b_start_var.get())
        end_angle = float(self.spherical_b_end_var.get())
        radius = float(self.spherical_radius_var.get())
        depth_of_cut = float(self.spherical_depth_of_cut_var.get())
        feedrate = float(self.spherical_feedrate_var.get())
        sample_count = int(self.spherical_samples_var.get())
        if radius <= 0.0 or depth_of_cut < 0.0 or feedrate <= 0.0 or sample_count < 2:
            raise ValueError
        direction = self.spherical_direction_var.get()
        x_sign = 1.0 if direction == "L-R" else -1.0
        return start_angle, end_angle, radius, depth_of_cut, feedrate, sample_count, direction, x_sign

    def _build_spherical_sliderest_samples(
        self,
        start_angle,
        end_angle,
        radius,
        depth_of_cut,
        sample_count,
        x_sign,
        invert_curve,
    ):
        sweep_deg = end_angle - start_angle
        start_angle_rad = math.radians(start_angle)
        samples = []
        for index in range(sample_count):
            fraction = index / float(sample_count - 1)
            current_angle = start_angle + sweep_deg * fraction
            current_angle_rad = math.radians(current_angle)
            x_arc_delta = x_sign * radius * (math.cos(current_angle_rad) - math.cos(start_angle_rad))
            z_arc_delta = radius * (math.sin(current_angle_rad) - math.sin(start_angle_rad))
            if invert_curve:
                x_arc_delta, z_arc_delta = z_arc_delta, x_arc_delta
            x_target = x_arc_delta
            z_target = -depth_of_cut + z_arc_delta
            samples.append((current_angle, x_target, z_target))
        return samples

    def _update_spherical_sliderest_preview(self, *_):
        self.spherical_ax.clear()
        self.spherical_ax.set_title("Spherical Slide Rest Path")
        self.spherical_ax.set_xlabel("X (mm)")
        self.spherical_ax.set_ylabel("Z (mm)")
        self.spherical_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.8)

        try:
            start_angle, end_angle, radius, depth_of_cut, _feedrate, sample_count, direction, x_sign = (
                self._get_spherical_sliderest_parameters()
            )
            samples = self._build_spherical_sliderest_samples(
                start_angle,
                end_angle,
                radius,
                depth_of_cut,
                sample_count,
                x_sign,
                self.spherical_curve_inverted_var.get(),
            )
            x_values = [sample[1] for sample in samples]
            z_values = [sample[2] for sample in samples]
            self.spherical_ax.plot(x_values, z_values, color="#1f77b4", linewidth=1.8)
            self.spherical_ax.scatter([x_values[0]], [z_values[0]], color="#2ca02c", label="Start", zorder=3)
            self.spherical_ax.scatter([x_values[-1]], [z_values[-1]], color="#d62728", label="End", zorder=3)
            self.spherical_ax.set_aspect("equal", adjustable="box")
            self.spherical_ax.legend(loc="best")
            self.spherical_ax.text(
                0.02,
                0.02,
                f"B {start_angle:.1f}° -> {end_angle:.1f}°\n{direction}, {sample_count} samples",
                transform=self.spherical_ax.transAxes,
                va="bottom",
                ha="left",
                fontsize=9,
                bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8, "edgecolor": "#bbbbbb"},
            )
        except Exception:
            self.spherical_ax.text(
                0.5,
                0.5,
                "Enter valid spherical sliderest values\n(samples must be 2 or more)",
                transform=self.spherical_ax.transAxes,
                ha="center",
                va="center",
            )

        self.spherical_canvas.draw_idle()

    def _build_reciprocator_tab(self, parent):
        body = ttk.Frame(parent, padding=(6, 6, 6, 6))
        body.pack(fill=tk.BOTH, expand=True)
        body.grid_rowconfigure(0, weight=1)
        body.grid_columnconfigure(1, weight=1)
        self._reciprocator_loaded_wave_x = None
        self._reciprocator_loaded_wave_y = None
        self._reciprocator_wave_last_non_load = "Sine"

        left_panel_host = ttk.LabelFrame(body, text="Waveform Controls", width=300)
        left_panel_host.grid(row=0, column=0, sticky="ns", padx=(0, 8))
        left_panel_host.grid_propagate(False)

        left_canvas = tk.Canvas(left_panel_host, highlightthickness=0, bg=GUI_BACKGROUND_COLOR)
        left_scrollbar = tk.Scrollbar(left_panel_host, orient=tk.VERTICAL, command=left_canvas.yview, width=14)
        left_panel = ttk.Frame(left_canvas, padding=10)

        left_panel.bind(
            "<Configure>",
            lambda event: left_canvas.configure(scrollregion=left_canvas.bbox("all")),
        )
        left_panel_window = left_canvas.create_window((0, 0), window=left_panel, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        left_canvas.bind(
            "<Configure>",
            lambda event: left_canvas.itemconfigure(left_panel_window, width=event.width),
        )

        left_panel_host.grid_rowconfigure(0, weight=1)
        left_panel_host.grid_columnconfigure(0, weight=1)
        left_canvas.grid(row=0, column=0, sticky="nsew")
        left_scrollbar.grid(row=0, column=1, sticky="ns")

        def add_labeled_entry(parent_widget, label_text, text_var, pady=(10, 0), state=None):
            row = ttk.Frame(parent_widget)
            row.pack(fill=tk.X, pady=pady)
            ttk.Label(row, text=label_text).pack(side=tk.LEFT)
            entry_kwargs = {"textvariable": text_var, "width": 14}
            if state is not None:
                entry_kwargs["state"] = state
            ttk.Entry(row, **entry_kwargs).pack(side=tk.RIGHT)

        wave_group = ttk.LabelFrame(left_panel, text="Waveform", padding=8)
        wave_group.pack(fill=tk.X)

        self.reciprocator_wave_var = tk.StringVar(value="Sine")
        ttk.Radiobutton(
            wave_group,
            text="Sine",
            value="Sine",
            variable=self.reciprocator_wave_var,
            command=self._on_reciprocator_wave_changed,
        ).pack(anchor=tk.W)
        ttk.Radiobutton(
            wave_group,
            text="Saw",
            value="Saw",
            variable=self.reciprocator_wave_var,
            command=self._on_reciprocator_wave_changed,
        ).pack(anchor=tk.W)
        ttk.Radiobutton(
            wave_group,
            text="Triangle",
            value="Triangle",
            variable=self.reciprocator_wave_var,
            command=self._on_reciprocator_wave_changed,
        ).pack(anchor=tk.W)
        ttk.Radiobutton(
            wave_group,
            text="Arc",
            value="Arc",
            variable=self.reciprocator_wave_var,
            command=self._on_reciprocator_wave_changed,
        ).pack(anchor=tk.W)
        ttk.Radiobutton(
            wave_group,
            text="Load",
            value="Load",
            variable=self.reciprocator_wave_var,
            command=self._on_reciprocator_wave_changed,
        ).pack(anchor=tk.W)

        self.reciprocator_period_var = tk.StringVar(value="10")
        add_labeled_entry(left_panel, "Period (Width) [mm]", self.reciprocator_period_var, pady=(12, 0))

        self.reciprocator_amplitude_var = tk.StringVar(value="5")
        add_labeled_entry(left_panel, "Amplitude (Height) [deg]", self.reciprocator_amplitude_var)

        self.reciprocator_invert_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            left_panel,
            text="Invert",
            variable=self.reciprocator_invert_var,
            command=self._update_reciprocator_plot,
        ).pack(anchor=tk.W, pady=(10, 0))

        ttk.Label(left_panel, text="Phase").pack(anchor=tk.W, pady=(10, 0))
        self.reciprocator_phase_var = tk.DoubleVar(value=0.0)
        ttk.Scale(
            left_panel,
            from_=0.0,
            to=180.0,
            orient=tk.HORIZONTAL,
            variable=self.reciprocator_phase_var,
            command=self._update_reciprocator_plot,
        ).pack(fill=tk.X)
        self.reciprocator_phase_display_var = tk.StringVar(value="0.0")
        add_labeled_entry(
            left_panel,
            "Phase Value",
            self.reciprocator_phase_display_var,
            pady=(4, 0),
            state="readonly",
        )

        cut_controls = ttk.LabelFrame(left_panel, text="Cut Controls", padding=8)
        cut_controls.pack(fill=tk.X, pady=(12, 0))

        self.reciprocator_feedrate_var = tk.StringVar(value="200")
        add_labeled_entry(cut_controls, "Feedrate", self.reciprocator_feedrate_var, pady=(0, 0))

        ttk.Label(cut_controls, text="Cut Direction").pack(anchor=tk.W, pady=(10, 0))
        self.reciprocator_cut_direction_var = tk.StringVar(value="L-R")
        cut_direction_frame = ttk.Frame(cut_controls)
        cut_direction_frame.pack(anchor=tk.W, pady=(4, 0))
        ttk.Radiobutton(
            cut_direction_frame,
            text="L-R",
            value="L-R",
            variable=self.reciprocator_cut_direction_var,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            cut_direction_frame,
            text="R-L",
            value="R-L",
            variable=self.reciprocator_cut_direction_var,
        ).pack(side=tk.LEFT, padx=(10, 0))

        self.reciprocator_depth_of_cut_var = tk.StringVar(value="0.1")
        add_labeled_entry(cut_controls, "Depth of cut [mm]", self.reciprocator_depth_of_cut_var)

        self.reciprocator_length_of_cut_var = tk.StringVar(value="50")
        add_labeled_entry(cut_controls, "Length of cut [mm]", self.reciprocator_length_of_cut_var)

        self.reciprocator_samples_var = tk.StringVar(value=str(self._settings["default_sample_count"]))
        add_labeled_entry(cut_controls, "Samples", self.reciprocator_samples_var)

        self.reciprocator_pulloff_var = tk.StringVar(value="1.0")
        add_labeled_entry(cut_controls, "Pull off amount [mm]", self.reciprocator_pulloff_var)

        self.reciprocator_return_to_zero_x_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            cut_controls,
            text="Return to Zero X",
            variable=self.reciprocator_return_to_zero_x_var,
        ).pack(anchor=tk.W, pady=(10, 0))

        self.reciprocator_repeat_cut_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            cut_controls,
            text="Repeat Cut",
            variable=self.reciprocator_repeat_cut_var,
        ).pack(anchor=tk.W, pady=(10, 0))

        self.reciprocator_num_divisions_var = tk.StringVar(value="1")
        add_labeled_entry(cut_controls, "Number of divisions", self.reciprocator_num_divisions_var)

        ttk.Button(cut_controls, text="Generate gCode", command=self._on_generate_reciprocator_gcode).pack(
            fill=tk.X, pady=(12, 0)
        )

        right_panel = ttk.Frame(body)
        right_panel.grid(row=0, column=1, sticky="nsew")

        self.reciprocator_figure = Figure(figsize=(7, 5), dpi=100)
        self.reciprocator_ax = self.reciprocator_figure.add_subplot(111)
        self.reciprocator_ax.set_title("Reciprocator Profile")
        self.reciprocator_ax.set_xlabel("X (mm)")
        self.reciprocator_ax.set_ylabel("A (deg)")
        self.reciprocator_ax.set_ylim(-15.0, 15.0)
        self.reciprocator_ax.axhline(0.0, color="#888888", linewidth=0.8)
        self.reciprocator_ax.axvline(0.0, color="#888888", linewidth=0.8)
        self.reciprocator_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.8)

        self.reciprocator_canvas = FigureCanvasTkAgg(self.reciprocator_figure, master=right_panel)
        self.reciprocator_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.reciprocator_period_var.trace_add("write", self._update_reciprocator_plot)
        self.reciprocator_amplitude_var.trace_add("write", self._update_reciprocator_plot)
        self.reciprocator_length_of_cut_var.trace_add("write", self._update_reciprocator_plot)
        self.reciprocator_canvas.draw_idle()
        self._update_reciprocator_plot()

    def _on_reciprocator_wave_changed(self):
        selected_wave = self.reciprocator_wave_var.get()
        if selected_wave == "Load":
            file_path = filedialog.askopenfilename(
                title="Load Waveform SVG",
                filetypes=[("SVG files", "*.svg"), ("All files", "*.*")],
            )
            if not file_path:
                fallback_wave = self._reciprocator_wave_last_non_load
                self.reciprocator_wave_var.set(fallback_wave)
                self._update_reciprocator_plot()
                return
            try:
                self._load_reciprocator_wave_from_svg(Path(file_path))
            except Exception as exc:
                messagebox.showerror("Load Waveform", f"Unable to load SVG waveform: {exc}")
                fallback_wave = self._reciprocator_wave_last_non_load
                self.reciprocator_wave_var.set(fallback_wave)
                self._update_reciprocator_plot()
                return
        else:
            self._reciprocator_wave_last_non_load = selected_wave

        self._update_reciprocator_plot()

    def _load_reciprocator_wave_from_svg(self, svg_path: Path):
        polylines = self._extract_polylines(svg_path)
        if not polylines:
            raise ValueError("No drawable SVG geometry found.")

        def line_length(points):
            length = 0.0
            for i in range(len(points) - 1):
                dx = points[i + 1][0] - points[i][0]
                dy = points[i + 1][1] - points[i][1]
                length += math.hypot(dx, dy)
            return length

        curve = max(
            polylines,
            key=lambda points: (
                max(p[0] for p in points) - min(p[0] for p in points),
                line_length(points),
            ),
        )
        if len(curve) < 2:
            raise ValueError("Selected curve must contain at least two points.")

        points = np.array(curve, dtype=float)
        left_idx = int(np.argmin(points[:, 0]))
        points = np.vstack((points[left_idx:], points[:left_idx]))

        if len(points) > 2 and points[1, 0] < points[0, 0]:
            points = np.vstack((points[0:1], points[:0:-1]))

        x_vals = points[:, 0]
        y_vals = points[:, 1]

        min_x = float(np.min(x_vals))
        max_x = float(np.max(x_vals))
        x_span = max_x - min_x
        if x_span <= 0.0:
            raise ValueError("Waveform SVG must span a non-zero width in X.")

        x_norm = (x_vals - min_x) / x_span
        order = np.argsort(x_norm)
        x_sorted = x_norm[order]
        y_sorted = y_vals[order]

        x_unique, inverse = np.unique(x_sorted, return_inverse=True)
        y_accum = np.bincount(inverse, weights=y_sorted)
        counts = np.bincount(inverse)
        y_unique = y_accum / np.maximum(1, counts)

        min_y = float(np.min(y_unique))
        max_y = float(np.max(y_unique))
        y_span = max_y - min_y
        if y_span > 0.0:
            y_center = 0.5 * (max_y + min_y)
            y_norm = (y_unique - y_center) / (0.5 * y_span)
        else:
            y_norm = np.zeros_like(y_unique)

        y_norm = np.clip(y_norm, -1.0, 1.0)

        if x_unique[0] > 0.0:
            x_unique = np.insert(x_unique, 0, 0.0)
            y_norm = np.insert(y_norm, 0, y_norm[0])
        if x_unique[-1] < 1.0:
            x_unique = np.append(x_unique, 1.0)
            y_norm = np.append(y_norm, y_norm[-1])

        self._reciprocator_loaded_wave_x = x_unique
        self._reciprocator_loaded_wave_y = y_norm

    def _update_reciprocator_plot(self, *_):
        try:
            period_mm = float(self.reciprocator_period_var.get())
            amplitude_deg = float(self.reciprocator_amplitude_var.get())
            length_mm = float(self.reciprocator_length_of_cut_var.get())
            if period_mm <= 0.0 or amplitude_deg < 0.0 or length_mm <= 0.0:
                raise ValueError
        except (AttributeError, ValueError):
            return

        phase_deg = float(self.reciprocator_phase_var.get())
        self.reciprocator_phase_display_var.set(f"{phase_deg:.1f}")
        x, y = self._build_reciprocator_profile(
            period_mm=period_mm,
            amplitude_deg=amplitude_deg,
            length_mm=length_mm,
            phase_deg=phase_deg,
            invert_wave=self.reciprocator_invert_var.get(),
            sample_count=800,
        )
        peak = amplitude_deg / 2.0

        self.reciprocator_ax.clear()
        self.reciprocator_ax.plot(x, y, color="#1f77b4", linewidth=1.8)
        self.reciprocator_ax.set_title("Reciprocator Profile")
        self.reciprocator_ax.set_xlabel("X (mm)")
        self.reciprocator_ax.set_ylabel("A (deg)")
        self.reciprocator_ax.set_xlim(0.0, length_mm)

        if peak > 0.0:
            self.reciprocator_ax.set_ylim(-peak, peak)
        else:
            self.reciprocator_ax.set_ylim(-1.0, 1.0)

        self.reciprocator_ax.axhline(0.0, color="#888888", linewidth=0.8)
        self.reciprocator_ax.axvline(0.0, color="#888888", linewidth=0.8)
        self.reciprocator_ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.8)
        self.reciprocator_canvas.draw_idle()

    def _build_reciprocator_profile(
        self,
        period_mm,
        amplitude_deg,
        length_mm,
        phase_deg,
        invert_wave,
        sample_count,
    ):
        peak = amplitude_deg / 2.0
        x = np.linspace(0.0, length_mm, sample_count)
        phase_offset = phase_deg / 360.0
        phase = x / period_mm + phase_offset

        wave = self.reciprocator_wave_var.get()
        if wave == "Saw":
            frac = phase - np.floor(phase)
            y = (2.0 * frac - 1.0) * peak
        elif wave == "Triangle":
            y = (2.0 * peak / math.pi) * np.arcsin(np.sin(2.0 * math.pi * phase))
        elif wave == "Arc":
            frac = phase - np.floor(phase)
            first_half = frac < 0.5

            # Map each half period to [0, 1] for a semicircle hump/trough.
            u = np.where(first_half, frac / 0.5, (frac - 0.5) / 0.5)
            semicircle = np.sqrt(np.maximum(0.0, 1.0 - (2.0 * u - 1.0) ** 2))
            y = np.where(first_half, peak * semicircle, -peak * semicircle)
        elif wave == "Load":
            if self._reciprocator_loaded_wave_x is None or self._reciprocator_loaded_wave_y is None:
                y = np.zeros_like(x)
            else:
                frac = phase - np.floor(phase)
                y = peak * np.interp(frac, self._reciprocator_loaded_wave_x, self._reciprocator_loaded_wave_y)
        else:
            y = peak * np.sin(2.0 * math.pi * phase)

        if invert_wave:
            y = -y

        return x, y

    def _on_generate_reciprocator_gcode(self):
        try:
            period_mm = float(self.reciprocator_period_var.get())
            amplitude_deg = float(self.reciprocator_amplitude_var.get())
            length_mm = float(self.reciprocator_length_of_cut_var.get())
            feedrate = float(self.reciprocator_feedrate_var.get())
            depth_of_cut = float(self.reciprocator_depth_of_cut_var.get())
            pulloff_mm = float(self.reciprocator_pulloff_var.get())
            division_count = int(self.reciprocator_num_divisions_var.get())
            sample_count = int(self.reciprocator_samples_var.get())
            if (
                period_mm <= 0.0
                or amplitude_deg < 0.0
                or length_mm <= 0.0
                or feedrate <= 0.0
                or depth_of_cut < 0.0
                or pulloff_mm < 0.0
                or division_count < 1
                or sample_count < 1
            ):
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "Invalid Reciprocator Input",
                "Period, length, feedrate, number of divisions, and samples must be positive. Amplitude, depth of cut, and pull off amount must be zero or greater.",
            )
            return

        phase_deg = float(self.reciprocator_phase_var.get())
        if self.reciprocator_wave_var.get() == "Load":
            if self._reciprocator_loaded_wave_x is None or self._reciprocator_loaded_wave_y is None:
                messagebox.showerror("Invalid Reciprocator Input", "Load waveform selected, but no SVG waveform is loaded.")
                return
        direction = self.reciprocator_cut_direction_var.get()
        x_sign = 1.0 if direction == "L-R" else -1.0
        x_samples, a_samples = self._build_reciprocator_profile(
            period_mm=period_mm,
            amplitude_deg=amplitude_deg,
            length_mm=length_mm,
            phase_deg=phase_deg,
            invert_wave=self.reciprocator_invert_var.get(),
            sample_count=sample_count,
        )

        lines = [
            "; Reciprocator waveform cut",
            "G21 ; mm units",
            "G90 ; absolute positioning",
            "M3 S100",
            f"; waveform={self.reciprocator_wave_var.get()} invert={self.reciprocator_invert_var.get()} phase_deg={phase_deg:.4f}",
            f"; period_mm={period_mm:.4f} amplitude_deg={amplitude_deg:.4f} length_mm={length_mm:.4f}",
            f"; cut_direction={direction} x_sign={x_sign:+.0f} depth_of_cut_mm={depth_of_cut:.4f}",
            f"; feedrate={feedrate:.4f}",
            f"; repeat_cut={self.reciprocator_repeat_cut_var.get()} divisions={division_count}",
            "G0 X0.0000 A0.0000",
        ]
        start_a = float(a_samples[0]) if len(a_samples) else 0.0

        cut_iterations = division_count if self.reciprocator_repeat_cut_var.get() else 1
        division_angle = 360.0 / float(division_count)
        should_return_x = self.reciprocator_repeat_cut_var.get() or self.reciprocator_return_to_zero_x_var.get()

        for iteration in range(cut_iterations):
            division_base_angle = iteration * division_angle if self.reciprocator_repeat_cut_var.get() else 0.0
            next_division_angle = (iteration + 1) * division_angle
            lines.append(f"; cut iteration {iteration + 1} of {cut_iterations}")

            lines.append("G0 X0.0000")
            lines.append(f"G0 A{division_base_angle:.4f}")
            if abs(start_a) > 1e-9:
                lines.append(f"G0 A{division_base_angle + start_a:.4f}")

            lines.append(f"G1 Z{-depth_of_cut:.4f} F{feedrate:.4f}")

            for x_val, a_val in zip(x_samples, a_samples):
                current_x = x_sign * float(x_val)
                current_a = division_base_angle + float(a_val)
                lines.append(f"G1 X{current_x:.4f} A{current_a:.4f} F{feedrate:.4f}")

            lines.append(f"G1 Z{pulloff_mm:.4f} F{feedrate:.4f}")

            if should_return_x:
                lines.append("G0 X0.0000")

            lines.append(f"G0 A{division_base_angle:.4f}")

            if self.reciprocator_repeat_cut_var.get():
                lines.append(f"G0 A{next_division_angle:.4f}")

            if pulloff_mm > 0.0:
                lines.append("G0 Z0.0000")

        lines.append("M5")
        lines.append("M2")

        lines.append(f"; points: {len(x_samples)}")
        output = "\n".join(lines) + "\n"

        self.gcode_text.config(state=tk.NORMAL)
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_text.insert(tk.END, output)
        self.gcode_text.config(state=tk.DISABLED)

    def _on_generate_plunge_gcode(self):
        try:
            total_depth, step_depth, retract_depth, safe_z, x_advance, number_of_cuts, feedrate, division_count = (
                self._get_plunge_parameters()
            )
        except ValueError:
            messagebox.showerror(
                "Invalid Plunge Input",
                "Total depth and step depth must be positive. Retract depth, safe Z, and X advance must be zero or greater. Number of cuts, feedrate, and number of divisions must be positive.",
            )
            return

        rotate_repeat = self.plunge_rotate_repeat_var.get()
        cut_iterations = division_count if rotate_repeat else 1
        division_angle = 360.0 / float(division_count)
        depth_levels = self._build_plunge_depth_levels(total_depth, step_depth)

        lines = [
            "; Plunge cut program",
            "G21 ; mm units",
            "G90 ; absolute positioning",
            "M3 S100",
            f"; total_depth_mm={total_depth:.4f}",
            f"; step_depth_mm={step_depth:.4f}",
            f"; retract_depth_mm={retract_depth:.4f}",
            f"; safe_z_mm={safe_z:.4f}",
            f"; x_advance_mm={x_advance:.4f}",
            f"; number_of_cuts={number_of_cuts}",
            f"; feedrate={feedrate:.4f}",
            f"; rotate_repeat={rotate_repeat} divisions={division_count}",
            "G0 X0.0000 A0.0000 Z0.0000",
        ]

        for iteration in range(cut_iterations):
            division_angle_target = iteration * division_angle if rotate_repeat else 0.0
            lines.append(f"; cut group {iteration + 1} of {cut_iterations}")
            lines.append(f"G0 A{division_angle_target:.4f}")

            for cut_index in range(number_of_cuts):
                x_target = cut_index * x_advance
                lines.append(f"G0 Z{safe_z:.4f}")
                lines.append(f"G0 X{x_target:.4f}")
                lines.append("G0 Z0.0000")
                for plunge_depth in depth_levels:
                    lines.append(f"G1 Z{-plunge_depth:.4f} F{feedrate:.4f}")
                    lines.append(f"G0 Z{retract_depth:.4f}")

            lines.append(f"G0 Z{safe_z:.4f}")
            lines.append("G0 X0.0000")
            lines.append("G0 Z0.0000")

        lines.append("M5")
        lines.append("M2")
        output = "\n".join(lines) + "\n"

        self.gcode_text.config(state=tk.NORMAL)
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_text.insert(tk.END, output)
        self.gcode_text.config(state=tk.DISABLED)

    def _on_generate_spherical_sliderest_gcode(self):
        try:
            start_angle, end_angle, radius, depth_of_cut, feedrate, sample_count, direction, x_sign = (
                self._get_spherical_sliderest_parameters()
            )
        except ValueError:
            messagebox.showerror(
                "Invalid Spherical Sliderest Input",
                "Radius and feedrate must be positive. Depth of cut must be zero or greater. Samples must be 2 or more.",
            )
            return

        samples = self._build_spherical_sliderest_samples(
            start_angle,
            end_angle,
            radius,
            depth_of_cut,
            sample_count,
            x_sign,
            self.spherical_curve_inverted_var.get(),
        )
        suppress_b = self.spherical_suppress_b_var.get()

        lines = [
            "; Spherical sliderest cut",
            "G21 ; mm units",
            "G90 ; absolute positioning",
            "M3 S100",
            f"; b_start_deg={start_angle:.4f} b_end_deg={end_angle:.4f}",
            f"; radius_mm={radius:.4f} depth_of_cut_mm={depth_of_cut:.4f}",
            f"; direction={direction} x_sign={x_sign:+.0f}",
            f"; curve_inverted_xz_diagonal={self.spherical_curve_inverted_var.get()}",
            f"; feedrate={feedrate:.4f}",
            f"; samples={sample_count}",
            f"; suppress_b_axis_move={suppress_b}",
            "; assumes X0 Z0 at cut start and B is already at starting angle",
        ]

        for current_angle, x_target, z_target in samples:
            if suppress_b:
                lines.append(f"G1 X{x_target:.4f} Z{z_target:.4f} F{feedrate:.4f}")
            else:
                lines.append(f"G1 X{x_target:.4f} Z{z_target:.4f} B{current_angle:.4f} F{feedrate:.4f}")

        lines.append("M5")
        lines.append("M2")
        lines.append(f"; points: {len(samples)}")
        output = "\n".join(lines) + "\n"

        self.gcode_text.config(state=tk.NORMAL)
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_text.insert(tk.END, output)
        self.gcode_text.config(state=tk.DISABLED)

    def _build_serial_tab(self, parent):
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        toolbar = ttk.Frame(frame)
        toolbar.pack(fill=tk.X)

        ttk.Label(toolbar, text="Serial Port:").pack(side=tk.LEFT)
        self.serial_port_var = tk.StringVar(value="")
        self.serial_port_combo = ttk.Combobox(
            toolbar,
            textvariable=self.serial_port_var,
            width=22,
            state="readonly",
        )
        self.serial_port_combo.pack(side=tk.LEFT, padx=(8, 8))

        self.refresh_ports_btn = ttk.Button(toolbar, text="Refresh", command=self._refresh_serial_ports)
        self.refresh_ports_btn.pack(side=tk.LEFT)

        self.connect_serial_btn = ttk.Button(toolbar, text="Connect", command=self._toggle_serial_connection)
        self.connect_serial_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.get_descriptions_btn = ttk.Button(
            toolbar,
            text="Get Descriptions",
            command=self._get_descriptions,
        )
        self.get_descriptions_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.clear_serial_btn = ttk.Button(toolbar, text="Clear", command=self._clear_serial_terminal)
        self.clear_serial_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.suppress_idle_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar, text="Suppress Idle", variable=self.suppress_idle_var).pack(
            side=tk.LEFT, padx=(12, 0)
        )

        self.filter_ok_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar, text="Filter OK", variable=self.filter_ok_var).pack(
            side=tk.LEFT, padx=(8, 0)
        )

        self.serial_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(toolbar, textvariable=self.serial_status_var).pack(side=tk.LEFT, padx=(12, 0))

        body = ttk.Frame(frame)
        body.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        terminal_frame = ttk.Frame(body)
        terminal_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.serial_terminal = scrolledtext.ScrolledText(
            terminal_frame,
            state=tk.DISABLED,
            wrap=tk.NONE,
            font=("Courier", 10),
        )
        self.serial_terminal.pack(fill=tk.BOTH, expand=True)

        send_row = ttk.Frame(terminal_frame)
        send_row.pack(fill=tk.X, pady=(8, 0))

        ttk.Label(send_row, text="Send:").pack(side=tk.LEFT)
        self.serial_send_var = tk.StringVar(value="")
        self.serial_send_entry = ttk.Entry(send_row, textvariable=self.serial_send_var)
        self.serial_send_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 8))
        self.serial_send_entry.bind("<Return>", self._on_serial_send_return)

        self.serial_send_btn = ttk.Button(send_row, text="Send", command=self._send_serial_text)
        self.serial_send_btn.pack(side=tk.LEFT)
        self._set_serial_send_enabled(False)

        left_panel = ttk.LabelFrame(body, text="Controls", padding=10, width=250)
        left_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        left_panel.pack_propagate(False)

        ttk.Label(left_panel, text="Jog Controls").pack(anchor=tk.W)

        jog_frame = ttk.Frame(left_panel)
        jog_frame.pack(anchor=tk.N, pady=(12, 0))

        ttk.Button(jog_frame, text="Z+", width=8, command=lambda: self._on_jog_axis("Z", +1)).grid(
            row=0, column=1, padx=4, pady=4
        )
        ttk.Button(jog_frame, text="X-", width=8, command=lambda: self._on_jog_axis("X", -1)).grid(
            row=1, column=0, padx=4, pady=4
        )
        ttk.Label(jog_frame, text="XYZ").grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(jog_frame, text="X+", width=8, command=lambda: self._on_jog_axis("X", +1)).grid(
            row=1, column=2, padx=4, pady=4
        )
        ttk.Button(jog_frame, text="Z-", width=8, command=lambda: self._on_jog_axis("Z", -1)).grid(
            row=2, column=1, padx=4, pady=4
        )

        ttk.Separator(left_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(14, 10))
        ttk.Label(left_panel, text="Jog Increment").pack(anchor=tk.W)

        self.jog_increment_var = tk.StringVar(value="1")
        increment_frame = ttk.Frame(left_panel)
        increment_frame.pack(anchor=tk.W, pady=(8, 0))

        ttk.Radiobutton(increment_frame, text="0.1", value="0.1", variable=self.jog_increment_var).grid(
            row=0, column=0, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(increment_frame, text="1", value="1", variable=self.jog_increment_var).grid(
            row=0, column=1, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(increment_frame, text="5", value="5", variable=self.jog_increment_var).grid(
            row=1, column=0, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(increment_frame, text="10", value="10", variable=self.jog_increment_var).grid(
            row=1, column=1, padx=4, pady=2, sticky=tk.W
        )

        ttk.Separator(left_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(14, 10))
        ttk.Label(left_panel, text="Jog Mode").pack(anchor=tk.W)

        self.jog_mode_var = tk.StringVar(value="Rapid")
        jog_mode_frame = ttk.Frame(left_panel)
        jog_mode_frame.pack(anchor=tk.W, pady=(8, 0))

        ttk.Radiobutton(jog_mode_frame, text="Rapid", value="Rapid", variable=self.jog_mode_var).grid(
            row=0, column=0, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(jog_mode_frame, text="Fine", value="Fine", variable=self.jog_mode_var).grid(
            row=0, column=1, padx=4, pady=2, sticky=tk.W
        )

        ttk.Separator(left_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(14, 10))
        ttk.Label(left_panel, text="Rotation").pack(anchor=tk.W)

        rotate_frame = ttk.Frame(left_panel)
        rotate_frame.pack(anchor=tk.N, pady=(8, 0))

        ttk.Button(rotate_frame, text="A-", width=8, command=lambda: self._on_jog_axis("A", -1)).grid(
            row=0, column=0, padx=4, pady=4
        )
        ttk.Button(rotate_frame, text="A+", width=8, command=lambda: self._on_jog_axis("A", +1)).grid(
            row=0, column=1, padx=4, pady=4
        )
        ttk.Button(rotate_frame, text="B-", width=8, command=lambda: self._on_jog_axis("B", -1)).grid(
            row=1, column=0, padx=4, pady=4
        )
        ttk.Button(rotate_frame, text="B+", width=8, command=lambda: self._on_jog_axis("B", +1)).grid(
            row=1, column=1, padx=4, pady=4
        )

        ttk.Separator(left_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(14, 10))
        ttk.Label(left_panel, text="Home").pack(anchor=tk.W)

        self.home_axis_var = tk.StringVar(value="All")
        home_frame = ttk.Frame(left_panel)
        home_frame.pack(anchor=tk.W, pady=(8, 0))

        ttk.Radiobutton(home_frame, text="X", value="X", variable=self.home_axis_var).grid(
            row=0, column=0, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(home_frame, text="Z", value="Z", variable=self.home_axis_var).grid(
            row=0, column=1, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(home_frame, text="A", value="A", variable=self.home_axis_var).grid(
            row=1, column=0, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(home_frame, text="B", value="B", variable=self.home_axis_var).grid(
            row=1, column=1, padx=4, pady=2, sticky=tk.W
        )
        ttk.Radiobutton(home_frame, text="All", value="All", variable=self.home_axis_var).grid(
            row=2, column=0, columnspan=2, padx=4, pady=2, sticky=tk.W
        )

        ttk.Button(left_panel, text="Home", command=self._on_home_axis).pack(fill=tk.X, pady=(8, 0))

        ttk.Separator(left_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(14, 10))
        ttk.Label(left_panel, text="Zero").pack(anchor=tk.W)

        ttk.Button(left_panel, text="Zero", command=self._on_zero_axis).pack(fill=tk.X, pady=(8, 0))

        if serial is None:
            self.serial_status_var.set("pyserial not installed")
            self.refresh_ports_btn.config(state=tk.DISABLED)
            self.connect_serial_btn.config(state=tk.DISABLED)
            self._append_serial_terminal("[Error] pyserial is required. Install with: pip install pyserial\n")
            return

        self._refresh_serial_ports()

    def _build_settings_tab(self, parent):
        frame = ttk.Frame(parent, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        form = ttk.LabelFrame(frame, text="Motion Feedrates", padding=10)
        form.pack(anchor=tk.NW, fill=tk.X)

        ttk.Label(form, text="Angular Feedrate").grid(row=0, column=0, sticky=tk.W)
        self.angular_feed_var = tk.StringVar(value=f"{self._settings['angular_feedrate']}")
        ttk.Entry(form, textvariable=self.angular_feed_var, width=14).grid(
            row=0, column=1, sticky=tk.W, padx=(10, 0)
        )

        ttk.Label(form, text="Z Feedrate").grid(row=1, column=0, sticky=tk.W, pady=(8, 0))
        self.z_feed_var = tk.StringVar(value=f"{self._settings['z_feedrate']}")
        ttk.Entry(form, textvariable=self.z_feed_var, width=14).grid(
            row=1, column=1, sticky=tk.W, padx=(10, 0), pady=(8, 0)
        )

        ttk.Label(form, text="Default Number of Samples").grid(
            row=2, column=0, sticky=tk.W, pady=(8, 0)
        )
        self.default_samples_var = tk.StringVar(value=f"{self._settings['default_sample_count']}")
        ttk.Entry(form, textvariable=self.default_samples_var, width=14).grid(
            row=2, column=1, sticky=tk.W, padx=(10, 0), pady=(8, 0)
        )

        ttk.Label(form, text="Fine Jog Feedrate").grid(row=3, column=0, sticky=tk.W, pady=(8, 0))
        self.fine_jog_feed_var = tk.StringVar(value=f"{self._settings['fine_jog_feedrate']}")
        ttk.Entry(form, textvariable=self.fine_jog_feed_var, width=14).grid(
            row=3, column=1, sticky=tk.W, padx=(10, 0), pady=(8, 0)
        )

        ttk.Checkbutton(
            form,
            text="Invert Z Direction",
            variable=self.invert_z_var,
        ).grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=(10, 0))

        save_btn = ttk.Button(form, text="Save Settings", command=self._on_save_settings)
        save_btn.grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=(12, 0))

        self.settings_status_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.settings_status_var, foreground="#2E6E3E").pack(
            anchor=tk.W, pady=(10, 0)
        )

    @staticmethod
    def _default_settings():
        return {
            "angular_feedrate": 200.0,
            "z_feedrate": 200.0,
            "default_sample_count": 720,
            "fine_jog_feedrate": 200.0,
            "invert_z_direction": False,
        }

    def _load_settings(self):
        defaults = self._default_settings()
        if not self._settings_path.exists():
            return defaults

        try:
            data = json.loads(self._settings_path.read_text(encoding="utf-8"))
            angular = float(data.get("angular_feedrate", defaults["angular_feedrate"]))
            z_feed = float(data.get("z_feedrate", defaults["z_feedrate"]))
            sample_count = int(data.get("default_sample_count", defaults["default_sample_count"]))
            fine_jog_feed = float(data.get("fine_jog_feedrate", defaults["fine_jog_feedrate"]))
            if angular <= 0 or z_feed <= 0 or sample_count < 1 or fine_jog_feed <= 0:
                raise ValueError
            invert_z = bool(data.get("invert_z_direction", defaults["invert_z_direction"]))
            return {
                "angular_feedrate": angular,
                "z_feedrate": z_feed,
                "default_sample_count": sample_count,
                "fine_jog_feedrate": fine_jog_feed,
                "invert_z_direction": invert_z,
            }
        except Exception:
            return defaults

    def _save_settings(
        self,
        angular_feedrate,
        z_feedrate,
        default_sample_count,
        fine_jog_feedrate,
        invert_z_direction,
    ):
        data = {
            "angular_feedrate": float(angular_feedrate),
            "z_feedrate": float(z_feedrate),
            "default_sample_count": int(default_sample_count),
            "fine_jog_feedrate": float(fine_jog_feedrate),
            "invert_z_direction": bool(invert_z_direction),
        }
        self._settings_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        self._settings = data

    def _get_feedrates_from_ui(self):
        try:
            angular = float(self.angular_feed_var.get())
            z_feed = float(self.z_feed_var.get())
            if angular <= 0 or z_feed <= 0:
                raise ValueError
            return angular, z_feed
        except (AttributeError, ValueError):
            raise ValueError("Angular and Z feedrates must be positive numbers.")

    def _get_sample_count_from_ui(self):
        try:
            sample_count = int(self.samples_var.get())
            if sample_count < 1:
                raise ValueError
            return sample_count
        except (AttributeError, ValueError):
            raise ValueError("Samples must be a positive whole number.")

    def _update_sample_step_display(self, *_):
        try:
            sample_count = int(self.samples_var.get())
            if sample_count < 1:
                raise ValueError
            step_deg = 360.0 / float(sample_count)
            self.sample_step_var.set(f"{step_deg:.6f}")
        except (AttributeError, ValueError):
            self.sample_step_var.set("—")

    def _get_default_sample_count_from_settings_ui(self):
        try:
            sample_count = int(self.default_samples_var.get())
            if sample_count < 1:
                raise ValueError
            return sample_count
        except (AttributeError, ValueError):
            raise ValueError("Default Number of Samples must be a positive whole number.")

    def _get_fine_jog_feedrate_from_settings_ui(self):
        try:
            fine_feedrate = float(self.fine_jog_feed_var.get())
            if fine_feedrate <= 0:
                raise ValueError
            return fine_feedrate
        except (AttributeError, ValueError):
            raise ValueError("Fine Jog Feedrate must be a positive number.")

    def _on_save_settings(self):
        try:
            angular, z_feed = self._get_feedrates_from_ui()
            default_samples = self._get_default_sample_count_from_settings_ui()
            fine_jog_feed = self._get_fine_jog_feedrate_from_settings_ui()
        except ValueError as exc:
            messagebox.showerror("Invalid Settings", str(exc))
            return

        try:
            self._save_settings(angular, z_feed, default_samples, fine_jog_feed, self.invert_z_var.get())
            self.samples_var.set(str(default_samples))
            self.settings_status_var.set("Settings saved to settings.json")
        except OSError as exc:
            messagebox.showerror("Save Error", f"Unable to save settings: {exc}")

    def _build_gcode_tab(self, parent):
        toolbar = ttk.Frame(parent, padding=(6, 6, 6, 0))
        toolbar.pack(fill=tk.X)

        save_btn = ttk.Button(toolbar, text="Save", command=self._on_save_gcode)
        save_btn.pack(side=tk.LEFT)

        clear_btn = ttk.Button(toolbar, text="Clear", command=self._on_clear_gcode)
        clear_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.send_gcode_serial_btn = ttk.Button(
            toolbar,
            text="Send to Serial",
            command=self._on_send_gcode_to_serial,
            state=tk.DISABLED,
        )
        self.send_gcode_serial_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.stop_gcode_serial_btn = ttk.Button(
            toolbar,
            text="Stop Sending",
            command=self._on_stop_gcode_send,
            state=tk.DISABLED,
        )
        self.stop_gcode_serial_btn.pack(side=tk.LEFT, padx=(8, 0))

        invert_chk = ttk.Checkbutton(
            toolbar,
            text="Invert Z Direction",
            variable=self.invert_z_var,
        )
        invert_chk.pack(side=tk.LEFT, padx=(14, 0))

        body = ttk.Frame(parent, padding=(6, 6, 6, 6))
        body.pack(fill=tk.BOTH, expand=True)

        self.gcode_text = scrolledtext.ScrolledText(
            body,
            state=tk.DISABLED,
            wrap=tk.NONE,
            font=("Courier", 10),
        )
        self.gcode_text.pack(fill=tk.BOTH, expand=True)

    def _build_svg_tab(self, parent):
        # ── Top toolbar ───────────────────────────────────────────────────────
        toolbar = ttk.Frame(parent, padding=(6, 6, 6, 0))
        toolbar.pack(fill=tk.X)

        open_btn = ttk.Button(toolbar, text="Open Rosette SVG", command=self.open_svg)
        open_btn.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="Select an SVG file to begin.")
        status_label = ttk.Label(toolbar, textvariable=self.status_var)
        status_label.pack(side=tk.LEFT, padx=12)

        # ── Main body (left panel + plot) ─────────────────────────────────────
        body = ttk.Frame(parent, padding=(6, 6, 6, 6))
        body.pack(fill=tk.BOTH, expand=True)

        # Left control panel
        left_panel = ttk.LabelFrame(body, text="Controls", padding=10, width=160)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        left_panel.pack_propagate(False)

        ttk.Label(left_panel, text="Samples").pack(anchor=tk.W)
        self.samples_var = tk.StringVar(value=str(self._settings["default_sample_count"]))
        samples_entry = ttk.Entry(left_panel, textvariable=self.samples_var, width=10)
        samples_entry.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(left_panel, text="Degrees per Sample").pack(anchor=tk.W)
        self.sample_step_var = tk.StringVar(value="—")
        sample_step_entry = ttk.Entry(
            left_panel,
            textvariable=self.sample_step_var,
            state="readonly",
            width=10,
        )
        sample_step_entry.pack(fill=tk.X, pady=(0, 10))

        self.samples_var.trace_add("write", self._update_sample_step_display)
        self._update_sample_step_display()

        sample_btn = ttk.Button(left_panel, text="Sample", command=self._on_sample)
        sample_btn.pack(fill=tk.X)

        gen_btn = ttk.Button(left_panel, text="Generate gCode", command=self._on_generate_gcode)
        gen_btn.pack(fill=tk.X, pady=(8, 0))

        ttk.Separator(left_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(14, 8))

        ttk.Label(left_panel, text="Max Radius").pack(anchor=tk.W)
        self.max_radius_var = tk.StringVar(value="—")
        max_r_entry = ttk.Entry(left_panel, textvariable=self.max_radius_var,
                                state="readonly", width=10)
        max_r_entry.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(left_panel, text="Min Radius").pack(anchor=tk.W)
        self.min_radius_var = tk.StringVar(value="—")
        min_r_entry = ttk.Entry(left_panel, textvariable=self.min_radius_var,
                                state="readonly", width=10)
        min_r_entry.pack(fill=tk.X)

        ttk.Label(left_panel, text="Amplitude").pack(anchor=tk.W, pady=(8, 0))
        self.amplitude_var = tk.StringVar(value="—")
        amp_entry = ttk.Entry(left_panel, textvariable=self.amplitude_var,
                              state="readonly", width=10)
        amp_entry.pack(fill=tk.X)

        self.figure = Figure(figsize=(8, 7), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")

        self.canvas = FigureCanvasTkAgg(self.figure, master=body)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right results panel
        right_panel = ttk.LabelFrame(body, text="Sample Results", padding=6, width=220)
        right_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=(8, 0))
        right_panel.pack_propagate(False)

        self.results_text = scrolledtext.ScrolledText(
            right_panel, state=tk.DISABLED, wrap=tk.NONE,
            font=("Courier", 9), width=24
        )
        self.results_text.pack(fill=tk.BOTH, expand=True)

    def _on_sample(self):
        if self._centered_polylines is None:
            messagebox.showwarning("No SVG", "Open an SVG file first.")
            return

        try:
            sample_count = self._get_sample_count_from_ui()
        except ValueError:
            messagebox.showerror("Invalid Input", "Samples must be a positive whole number.")
            return

        step_deg = 360.0 / float(sample_count)

        samples = self._sample_geometry(self._centered_polylines, step_deg)
        count = len(samples)

        lines = []
        lines.append(f"samples={sample_count}  step={step_deg:.6f}°  points={count}")
        lines.append(f"{'Angle (°)':>10}  {'Distance':>12}")
        lines.append("-" * 25)
        for angle, dist in samples:
            lines.append(f"{angle:10.4f}  {dist:12.6f}")
        output = "\n".join(lines) + "\n"

        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete("1.0", tk.END)
        self.results_text.insert(tk.END, output)
        self.results_text.config(state=tk.DISABLED)

    def _on_generate_gcode(self):
        if self._centered_polylines is None:
            messagebox.showwarning("No SVG", "Open an SVG file first.")
            return

        try:
            sample_count = self._get_sample_count_from_ui()
        except ValueError:
            messagebox.showerror("Invalid Input", "Samples must be a positive whole number.")
            return

        step_deg = 360.0 / float(sample_count)

        samples = self._sample_geometry(self._centered_polylines, step_deg)
        if not samples:
            messagebox.showwarning("No Samples", "No geometry intersections were found for the current SVG.")
            return

        try:
            angular_feed, z_feed = self._get_feedrates_from_ui()
            default_samples = self._get_default_sample_count_from_settings_ui()
            fine_jog_feed = self._get_fine_jog_feedrate_from_settings_ui()
        except ValueError as exc:
            messagebox.showerror("Invalid Settings", str(exc))
            return

        invert_z = self.invert_z_var.get()

        try:
            self._save_settings(angular_feed, z_feed, default_samples, fine_jog_feed, invert_z)
            self.settings_status_var.set("Settings saved to settings.json")
        except (AttributeError, OSError):
            # If settings UI is unavailable for any reason, continue generating gCode.
            pass

        max_radius = max(radius for _, radius in samples)
        lines = [
            "; Generated from sampled SVG geometry",
            "; Assumes cutter is zeroed at maximum radius (Z0 at r=max)",
            "G21 ; mm units",
            "G90 ; absolute positioning",
            f"; max_radius={max_radius:.6f}",
            "; Z depth at each sample is: max_radius - sampled_radius",
            f"; angular_feedrate={angular_feed:.4f}",
            f"; z_feedrate={z_feed:.4f}",
            f"; invert_z_direction={invert_z}",
            "; Combined A/Z moves use the slower of angular and Z feedrates",
        ]

        first_angle, first_radius = samples[0]
        z_sign = -1.0 if invert_z else 1.0
        first_z = z_sign * max(0.0, max_radius - first_radius)
        lines.append(f"G0 A{first_angle:.4f} Z{first_z:.4f}")

        for angle, radius in samples:
            z_depth = z_sign * max(0.0, max_radius - radius)
            combined_feed = min(angular_feed, z_feed)
            lines.append(f"G1 A{angle:.4f} Z{z_depth:.4f} F{combined_feed:.4f}")

        lines.append(f"; points: {len(samples)}")
        output = "\n".join(lines) + "\n"

        self.gcode_text.config(state=tk.NORMAL)
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_text.insert(tk.END, output)
        self.gcode_text.config(state=tk.DISABLED)

    def _on_clear_gcode(self):
        self.gcode_text.config(state=tk.NORMAL)
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_text.config(state=tk.DISABLED)

    def _on_save_gcode(self):
        output = self.gcode_text.get("1.0", tk.END)
        if not output.strip():
            messagebox.showwarning("Save gCode", "There is no gCode to save.")
            return

        file_path = filedialog.asksaveasfilename(
            title="Save gCode",
            defaultextension=".nc",
            filetypes=[
                ("G-code files", "*.nc *.gcode *.tap *.txt"),
                ("All files", "*.*"),
            ],
        )
        if not file_path:
            return

        try:
            Path(file_path).write_text(output, encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Save Error", f"Unable to save gCode: {exc}")

    def _on_send_gcode_to_serial(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            messagebox.showwarning("Serial Port", "Connect to a serial port first.")
            return
        if self._gcode_send_queue or self._gcode_send_waiting_ok:
            messagebox.showwarning("Send to Serial", "A gCode send is already in progress.")
            return

        gcode_text = self.gcode_text.get("1.0", tk.END)
        lines_to_send = []
        for line in gcode_text.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith(";"):
                continue
            lines_to_send.append(stripped)

        if not lines_to_send:
            messagebox.showwarning("Send to Serial", "No executable gCode lines found.")
            return

        self._gcode_send_queue = list(lines_to_send)
        self._gcode_send_total = len(lines_to_send)
        self._gcode_send_sent = 0
        self._gcode_send_waiting_ok = False
        self.serial_status_var.set(f"Sending gCode 0/{self._gcode_send_total}")
        self._append_serial_terminal(f"[gCode] Starting send: {self._gcode_send_total} lines\n")
        self.send_gcode_serial_btn.config(state=tk.DISABLED)
        self.stop_gcode_serial_btn.config(state=tk.NORMAL)
        self._send_next_queued_gcode_line()

    def _on_stop_gcode_send(self):
        if not (self._gcode_send_queue or self._gcode_send_waiting_ok):
            return
        self.serial_status_var.set("gCode send stopped")
        self._cancel_gcode_send("Stopped by user")

    def _send_next_queued_gcode_line(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            self._cancel_gcode_send("Serial connection lost.")
            return

        if not self._gcode_send_queue:
            self.serial_status_var.set(f"gCode send complete ({self._gcode_send_total} lines)")
            self._append_serial_terminal(f"[gCode] Send complete: {self._gcode_send_total} lines\n")
            self._gcode_send_total = 0
            self._gcode_send_sent = 0
            self._gcode_send_waiting_ok = False
            self.send_gcode_serial_btn.config(state=tk.NORMAL)
            self.stop_gcode_serial_btn.config(state=tk.DISABLED)
            return

        line = self._gcode_send_queue.pop(0)
        try:
            self._serial_conn.write(f"{line}\n".encode("utf-8"))
            self._append_serial_terminal(f"> {line}\n")
            self._gcode_send_waiting_ok = True
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()

    def _cancel_gcode_send(self, reason):
        if self._gcode_send_queue or self._gcode_send_waiting_ok:
            self._append_serial_terminal(f"[gCode] Send canceled: {reason}\n")
        self._gcode_send_queue.clear()
        self._gcode_send_waiting_ok = False
        self._gcode_send_total = 0
        self._gcode_send_sent = 0
        if hasattr(self, "send_gcode_serial_btn"):
            state = tk.NORMAL if self._serial_conn is not None and self._serial_conn.is_open else tk.DISABLED
            self.send_gcode_serial_btn.config(state=state)
        if hasattr(self, "stop_gcode_serial_btn"):
            self.stop_gcode_serial_btn.config(state=tk.DISABLED)

    def _append_serial_terminal(self, text):
        self.serial_terminal.config(state=tk.NORMAL)
        self.serial_terminal.insert(tk.END, text)
        self.serial_terminal.see(tk.END)
        self.serial_terminal.config(state=tk.DISABLED)

    def _clear_serial_terminal(self):
        self.serial_terminal.config(state=tk.NORMAL)
        self.serial_terminal.delete("1.0", tk.END)
        self.serial_terminal.config(state=tk.DISABLED)

    def _refresh_serial_ports(self):
        if list_ports is None:
            return
        ports = [port.device for port in list_ports.comports()]
        self.serial_port_combo["values"] = ports

        if ports:
            if self.serial_port_var.get() not in ports:
                self.serial_port_var.set(ports[0])
            self.serial_status_var.set(f"Found {len(ports)} port(s)")
        else:
            self.serial_port_var.set("")
            self.serial_status_var.set("No serial ports found")

    def _toggle_serial_connection(self):
        if self._serial_conn and self._serial_conn.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        port = self.serial_port_var.get().strip()
        if not port:
            messagebox.showwarning("Serial Port", "Select a serial port first.")
            return

        try:
            self._serial_conn = serial.Serial(port=port, baudrate=115200, timeout=0)
            self._serial_rx_buffer = ""
            self.connect_serial_btn.config(text="Disconnect")
            self.serial_status_var.set(f"Connected: {port} @ 115200")
            self._set_serial_send_enabled(True)
            self._append_serial_terminal(f"[Connected] {port} @ 115200\n")
            self._start_serial_polling()
        except Exception as exc:
            self._serial_conn = None
            messagebox.showerror("Serial Connection", f"Unable to connect to {port}: {exc}")

    def _disconnect_serial(self):
        self._stop_serial_polling()
        self._cancel_gcode_send("Disconnected")
        self._serial_rx_buffer = ""
        if self._serial_conn is not None:
            try:
                if self._serial_conn.is_open:
                    self._serial_conn.close()
            except Exception:
                pass
        self._serial_conn = None
        self.connect_serial_btn.config(text="Connect")
        self.serial_status_var.set("Disconnected")
        self._set_serial_send_enabled(False)
        self._append_serial_terminal("[Disconnected]\n")

    def _set_serial_send_enabled(self, enabled):
        state = tk.NORMAL if enabled else tk.DISABLED
        self.serial_send_entry.config(state=state)
        self.serial_send_btn.config(state=state)
        self.get_descriptions_btn.config(state=state)
        if hasattr(self, "send_gcode_serial_btn"):
            self.send_gcode_serial_btn.config(state=state)
        if hasattr(self, "stop_gcode_serial_btn"):
            self.stop_gcode_serial_btn.config(state=tk.DISABLED)

    def _on_serial_send_return(self, _event):
        self._send_serial_text()
        return "break"

    def _send_serial_text(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            messagebox.showwarning("Serial Port", "Connect to a serial port first.")
            return
        if self._gcode_send_queue or self._gcode_send_waiting_ok:
            messagebox.showwarning("Serial Busy", "Wait for the active gCode send to complete.")
            return

        text = self.serial_send_var.get()
        if text == "":
            return

        payload = text if text.endswith("\n") else f"{text}\n"
        try:
            self._serial_conn.write(payload.encode("utf-8"))
            self._append_serial_terminal(f"> {text}\n")
            self.serial_send_var.set("")
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()

    def _on_jog_axis(self, axis, direction):
        if self._serial_conn is None or not self._serial_conn.is_open:
            messagebox.showwarning("Serial Port", "Connect to a serial port first.")
            return

        try:
            increment = float(self.jog_increment_var.get())
            if increment <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Jog Increment", "Jog increment must be a positive number.")
            return

        delta = float(direction) * increment
        mode = self.jog_mode_var.get()
        if mode == "Fine":
            try:
                fine_feedrate = self._get_fine_jog_feedrate_from_settings_ui()
            except ValueError:
                fine_feedrate = float(self._settings.get("fine_jog_feedrate", 200.0))
            command = f"G1 {axis}{delta:.4f} F{fine_feedrate:.4f}"
        else:
            command = f"G0 {axis}{delta:.4f}"

        try:
            for line in ("G91", command, "G90"):
                self._serial_conn.write(f"{line}\n".encode("utf-8"))
                self._append_serial_terminal(f"> {line}\n")
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()

    def _on_home_axis(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            messagebox.showwarning("Serial Port", "Connect to a serial port first.")
            return

        axis_selection = self.home_axis_var.get()
        
        if axis_selection == "All":
            command = "G0 X0 Z0 A0 B0"
        else:
            command = f"G0 {axis_selection}0"

        try:
            self._serial_conn.write(f"{command}\n".encode("utf-8"))
            self._append_serial_terminal(f"> {command}\n")
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()

    def _on_zero_axis(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            messagebox.showwarning("Serial Port", "Connect to a serial port first.")
            return

        axis_selection = self.home_axis_var.get()
        
        if axis_selection == "All":
            command = "G92 X0 Z0 A0 B0"
        else:
            command = f"G92 {axis_selection}0"

        try:
            self._serial_conn.write(f"{command}\n".encode("utf-8"))
            self._append_serial_terminal(f"> {command}\n")
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()

    def _get_descriptions(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            messagebox.showwarning("Serial Port", "Connect to a serial port first.")
            return

        try:
            for index in range(200,681):
                command = f"$SED={index}\n"
                self._serial_conn.write(command.encode("utf-8"))
                self._append_serial_terminal(f"> $SED={index}\n")
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()

    def _start_serial_polling(self):
        self._stop_serial_polling()
        self._serial_poll_job = self.after(50, self._poll_serial_data)

    def _stop_serial_polling(self):
        if self._serial_poll_job is not None:
            self.after_cancel(self._serial_poll_job)
            self._serial_poll_job = None

    def _poll_serial_data(self):
        if self._serial_conn is None or not self._serial_conn.is_open:
            self._serial_poll_job = None
            return

        try:
            waiting = self._serial_conn.in_waiting
            if waiting > 0:
                data = self._serial_conn.read(waiting)
                if data:
                    decoded = data.decode("utf-8", errors="replace")
                    self._serial_rx_buffer += decoded
                    while "\n" in self._serial_rx_buffer:
                        raw_line, self._serial_rx_buffer = self._serial_rx_buffer.split("\n", 1)
                        line = raw_line.rstrip("\r")
                        stripped_line = line.strip()

                        if self._gcode_send_waiting_ok and stripped_line.lower() == "ok":
                            self._gcode_send_waiting_ok = False
                            self._gcode_send_sent += 1
                            self.serial_status_var.set(
                                f"Sending gCode {self._gcode_send_sent}/{self._gcode_send_total}"
                            )
                            self._send_next_queued_gcode_line()
                        elif self._gcode_send_waiting_ok and stripped_line.lower().startswith("error"):
                            self.serial_status_var.set(f"gCode error: {stripped_line}")
                            self._cancel_gcode_send(stripped_line)

                        if self.suppress_idle_var.get() and stripped_line.startswith("<Idle"):
                            continue
                        if self.filter_ok_var.get() and stripped_line == "ok":
                            continue
                        self._append_serial_terminal(f"{line}\n")
        except Exception as exc:
            self.serial_status_var.set(f"Serial error: {exc}")
            self._append_serial_terminal(f"\n[Error] {exc}\n")
            self._disconnect_serial()
            return

        self._serial_poll_job = self.after(50, self._poll_serial_data)

    def _on_close(self):
        self._disconnect_serial()
        self.destroy()

    def _plot_empty_state(self):
        self.ax.clear()
        self.ax.set_title("Rosette SVG Preview")
        self._draw_polar_guides(radius=1.0)
        self.ax.set_xlim(-1.1, 1.1)
        self.ax.set_ylim(-1.1, 1.1)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.canvas.draw_idle()

    def open_svg(self):
        if parse_path is None:
            messagebox.showerror(
                "Missing Dependency",
                "The package 'svg.path' is required. Install it with:\n\npip install svg.path",
            )
            return

        file_path = filedialog.askopenfilename(
            title="Open Rosette SVG",
            filetypes=[("SVG files", "*.svg"), ("All files", "*.*")],
        )
        if not file_path:
            return

        try:
            polylines = self._extract_polylines(Path(file_path))
            if not polylines:
                raise ValueError("No drawable SVG geometry found.")
            centered = self._center_polylines(polylines)
            self._centered_polylines = centered
            self._plot_polylines(centered)
            self.status_var.set(f"Loaded: {Path(file_path).name}")
        except Exception as exc:
            messagebox.showerror("Unable to load SVG", str(exc))

    def _extract_polylines(self, svg_path: Path):
        tree = ET.parse(svg_path)
        root = tree.getroot()

        polylines = []
        for elem in root.iter():
            tag = elem.tag.split("}")[-1].lower()

            if tag == "path":
                d = elem.attrib.get("d", "").strip()
                if d:
                    polylines.extend(self._sample_path_d(d))
            elif tag == "line":
                x1 = self._to_float(elem.attrib.get("x1", "0"))
                y1 = self._to_float(elem.attrib.get("y1", "0"))
                x2 = self._to_float(elem.attrib.get("x2", "0"))
                y2 = self._to_float(elem.attrib.get("y2", "0"))
                polylines.append([(x1, -y1), (x2, -y2)])
            elif tag in {"polyline", "polygon"}:
                pts = self._parse_points(elem.attrib.get("points", ""))
                if tag == "polygon" and pts:
                    pts.append(pts[0])
                if len(pts) >= 2:
                    polylines.append(pts)
            elif tag == "circle":
                cx = self._to_float(elem.attrib.get("cx", "0"))
                cy = self._to_float(elem.attrib.get("cy", "0"))
                r = self._to_float(elem.attrib.get("r", "0"))
                if r > 0:
                    polylines.append(self._sample_ellipse(cx, cy, r, r))
            elif tag == "ellipse":
                cx = self._to_float(elem.attrib.get("cx", "0"))
                cy = self._to_float(elem.attrib.get("cy", "0"))
                rx = self._to_float(elem.attrib.get("rx", "0"))
                ry = self._to_float(elem.attrib.get("ry", "0"))
                if rx > 0 and ry > 0:
                    polylines.append(self._sample_ellipse(cx, cy, rx, ry))
            elif tag == "rect":
                x = self._to_float(elem.attrib.get("x", "0"))
                y = self._to_float(elem.attrib.get("y", "0"))
                w = self._to_float(elem.attrib.get("width", "0"))
                h = self._to_float(elem.attrib.get("height", "0"))
                if w > 0 and h > 0:
                    polylines.append(
                        [
                            (x, -y),
                            (x + w, -y),
                            (x + w, -(y + h)),
                            (x, -(y + h)),
                            (x, -y),
                        ]
                    )

        return [line for line in polylines if len(line) >= 2]

    def _sample_path_d(self, d):
        path_obj = parse_path(d)
        if not path_obj:
            return []

        lines = []
        current = []
        tol = 1e-8

        for seg in path_obj:
            start = (seg.start.real, -seg.start.imag)
            end = (seg.end.real, -seg.end.imag)

            if not current:
                current = [start]
            elif math.hypot(current[-1][0] - start[0], current[-1][1] - start[1]) > tol:
                if len(current) >= 2:
                    lines.append(current)
                current = [start]

            n = 2
            if seg.length(error=1e-4) > 0:
                # Curves are sampled more densely so rosette contours stay smooth.
                n = max(6, int(seg.length(error=1e-4) / 4.0))

            for t in np.linspace(0.0, 1.0, n, endpoint=False)[1:]:
                p = seg.point(float(t))
                current.append((p.real, -p.imag))
            current.append(end)

        if len(current) >= 2:
            lines.append(current)

        return lines

    @staticmethod
    def _sample_ellipse(cx, cy, rx, ry, samples=180):
        angles = np.linspace(0.0, 2.0 * math.pi, samples)
        return [(cx + rx * math.cos(t), -(cy + ry * math.sin(t))) for t in angles]

    @staticmethod
    def _to_float(value):
        return float(str(value).replace("px", "").strip())

    @staticmethod
    def _parse_points(points_text):
        clean = points_text.replace("\n", " ").replace("\t", " ").strip()
        if not clean:
            return []

        values = []
        for chunk in clean.replace(",", " ").split():
            values.append(float(chunk))

        pts = []
        for i in range(0, len(values) - 1, 2):
            pts.append((values[i], -values[i + 1]))
        return pts

    @staticmethod
    def _center_polylines(polylines):
        all_x = [pt[0] for line in polylines for pt in line]
        all_y = [pt[1] for line in polylines for pt in line]

        cx = 0.5 * (min(all_x) + max(all_x))
        cy = 0.5 * (min(all_y) + max(all_y))

        return [[(x - cx, y - cy) for (x, y) in line] for line in polylines]

    def _plot_polylines(self, polylines):
        self.ax.clear()
        self.ax.set_title("Rosette SVG Preview")

        max_radius = 0.0
        min_radius = math.inf
        for line in polylines:
            x = [pt[0] for pt in line]
            y = [pt[1] for pt in line]
            self.ax.plot(x, y, color="black", linewidth=1.2)

            for px, py in line:
                r = math.hypot(px, py)
                if r > max_radius:
                    max_radius = r
                if r < min_radius:
                    min_radius = r

        if max_radius == 0.0:
            max_radius = 1.0
        if min_radius == math.inf:
            min_radius = 0.0

        self.max_radius_var.set(f"{max_radius:.4f}")
        self.min_radius_var.set(f"{min_radius:.4f}")
        self.amplitude_var.set(f"{max_radius - min_radius:.4f}")

        self._draw_polar_guides(radius=max_radius)

        pad = max_radius * 0.08
        lim = max_radius + pad
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_aspect("equal", adjustable="box")

        self.canvas.draw_idle()

    @staticmethod
    def _ray_segment_intersect(dx, dy, ax, ay, bx, by):
        """Return t >= 0 for ray O+t*(dx,dy) intersecting segment A->B, or None."""
        ex, ey = bx - ax, by - ay          # segment direction
        denom = dy * ex - dx * ey          # cross(d, e)
        if abs(denom) < 1e-14:
            return None                    # parallel
        t = (ay * ex - ax * ey) / denom
        s = (ay * dx - ax * dy) / denom
        if t >= 1e-9 and -1e-9 <= s <= 1.0 + 1e-9:
            return t
        return None

    def _sample_geometry(self, polylines, step_deg):
        """For each angle from 0..360 (exclusive) at step_deg intervals, cast a ray
        from the origin and return the closest intersection as (angle_deg, distance)."""
        angles_deg = np.arange(0.0, 360.0, step_deg)
        results = []
        for angle_deg in angles_deg:
            rad = math.radians(angle_deg + ANGLE_OFFSET_DEG)
            dx, dy = math.cos(rad), math.sin(rad)
            best_t = None
            for line in polylines:
                for i in range(len(line) - 1):
                    ax, ay = line[i]
                    bx, by = line[i + 1]
                    t = self._ray_segment_intersect(dx, dy, ax, ay, bx, by)
                    if t is not None:
                        if best_t is None or t < best_t:
                            best_t = t
            if best_t is not None:
                results.append((angle_deg, best_t))
        return results

    def _draw_polar_guides(self, radius):
        for deg in range(0, 360, 45):
            rad = math.radians(deg + ANGLE_OFFSET_DEG)
            x = radius * math.cos(rad)
            y = radius * math.sin(rad)
            self.ax.plot([-x, x], [-y, y], linestyle="--", linewidth=0.6, color="#888888", alpha=0.7, zorder=0)

            # Degree labels slightly outside the guide circle.
            label_r = radius * 1.04
            lx = label_r * math.cos(rad)
            ly = label_r * math.sin(rad)
            self.ax.text(
                lx,
                ly,
                f"{deg}°",
                fontsize=8,
                color="#666666",
                ha="center",
                va="center",
                zorder=1,
            )

        self.ax.add_artist(plt.Circle((0, 0), radius, color="#BBBBBB", fill=False, linestyle=":", linewidth=0.7, zorder=0))


if __name__ == "__main__":
    app = RosetteSvgViewer()
    app.mainloop()
