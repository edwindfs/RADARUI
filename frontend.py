import threading
import tkinter as tk
from tkinter import ttk, messagebox
import math

from echoshield import PrimaryC2

class RadarDisplay(tk.Canvas):
    def __init__(self, parent, app, max_range=1000, **kwargs):
        super().__init__(parent, **kwargs)
        self.app = app
        self.max_range = max_range
        self.map_rotation = 0
        self.bind("<Button-1>", self.on_click)
        
    def set_rotation(self, angle_deg):
        self.map_rotation = angle_deg % 360
        
    def on_click(self, event):
        width = self.winfo_width()
        height = self.winfo_height()
        size = min(width, height)
        cx = width / 2
        cy = height / 2
        radius = size / 2 - 24
        
        closest_tid = None
        closest_dist = float('inf')
        
        filter_val = self.app.radar_filter_var.get()
        filter_ip = None
        if "(" in filter_val and ")" in filter_val:
            filter_ip = filter_val.split("(")[1].split(")")[0]

        with self.app.c2.lock:
            if filter_ip is None:
                kinematics = self.app.c2.fused_targets.copy()
            else:
                kinematics = self.app.c2.target_kinematics.copy()

        for tid, details in kinematics.items():
            if filter_ip is not None and not isinstance(tid, int) and tid[0] != filter_ip:
                continue
                
            r = details.get('range_m', float('inf'))
            if r > self.max_range:
                continue
            b = details.get('bearing_deg', 0)
            
            b_rad = math.radians(b - self.map_rotation)
            ratio = r / self.max_range
            px_dist = radius * ratio
            
            x = cx + px_dist * math.sin(b_rad)
            y = cy - px_dist * math.cos(b_rad)
            
            dist = math.hypot(event.x - x, event.y - y)
            if dist < 14 and dist < closest_dist:
                closest_tid = tid
                closest_dist = dist
                
        if closest_tid is not None:
            self.app._select_target_by_id(closest_tid)

    def redraw(self, active_targets, selected_tid, kinematics):
        self.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        size = min(width, height)
        cx = width / 2
        cy = height / 2
        radius = size / 2 - 24
        
        if radius <= 0: return

        # bg
        self.create_rectangle(0, 0, width, height, fill="#163628", outline="")
        
        # rings
        for ring in range(1, 5):
            r = radius * ring / 4
            self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#255c43")
            
        # crosshairs
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x = cx + math.cos(rad) * radius
            y = cy + math.sin(rad) * radius
            self.create_line(cx, cy, x, y, fill="#255c43")
            
        self.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#3caa78")
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#3caa78", outline="")
        
        # draw north indicator
        n_rad = math.radians(-self.map_rotation)
        nx = cx + math.sin(n_rad) * (radius + 12)
        ny = cy - math.cos(n_rad) * (radius + 12)
        self.create_text(nx, ny, text="N", fill="#79ffa4", font=("Arial", 10, "bold"))
        
        if self.map_rotation != 0:
            up_heading = (self.map_rotation + 360) % 360
            self.create_text(cx, cy - radius - 12, text=f"{up_heading:.0f}°", fill="#79ffa4", font=("Arial", 10, "bold"))
        
        # targets
        for tid, details in kinematics.items():
            if tid not in active_targets: continue
            
            r = details.get('range_m', float('inf'))
            if r > self.max_range:
                continue
            b = details.get('bearing_deg', 0)
            
            b_rad = math.radians(b - self.map_rotation)
            ratio = r / self.max_range
            px_dist = radius * ratio
            
            x = cx + px_dist * math.sin(b_rad)
            y = cy - px_dist * math.cos(b_rad)
            
            is_selected = (tid == selected_tid)
            marker_size = 16 if is_selected else 10
            fill_color = "#ffd666" if is_selected else "#79ffa4"
            outline_color = "#fff5c4" if is_selected else "#dcffe8"
            
            x0 = x - marker_size/2
            y0 = y - marker_size/2
            x1 = x + marker_size/2
            y1 = y + marker_size/2
            
            self.create_oval(x0, y0, x1, y1, fill=fill_color, outline=outline_color)
            
            if isinstance(tid, int):
                label = f"F-{tid}"
            else:
                ip, track_id = tid
                import echoshield
                import rada
                if ip == rada.RADA_IP:
                    label = f"RADA-{hex(track_id)}"
                else:
                    idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                    label = f"R{idx}-{hex(track_id)}"
            
            if is_selected:
                self.create_oval(x0-6, y0-6, x1+6, y1+6, outline=outline_color)
                self.create_text(x + 20, y - 4, text=label, fill=fill_color, anchor="w")
            else:
                self.create_text(x + 10, y - 2, text=label, fill=fill_color, anchor="w")

class OperatorApp:
    def __init__(self, root, c2):
        self.root = root
        self.c2 = c2
        self.root.title("Radar Operator Interface")
        self.root.geometry("1400x720")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.status_var = tk.StringVar(value="Ready")
        
        self.fields = {
            "Track ID": tk.StringVar(value="—"),
            "Classification": tk.StringVar(value="—"),
            "Range (slant)": tk.StringVar(value="—"),
            "Bearing": tk.StringVar(value="—"),
            "Latitude": tk.StringVar(value="—"),
            "Longitude": tk.StringVar(value="—"),
            "Target altitude": tk.StringVar(value="—"),
        }
        
        self._build()
        self.root.after(250, self.refresh)

    def _build(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")
        
        ttk.Label(top, text="RADAR: ECHODYNE ECHOSHIELD").pack(side="left")
        
        import echoshield
        import rada
        self.radar_filter_var = tk.StringVar(value="All Radars")
        filter_options = ["All Radars"] + [f"Radar {i+1} ({ip})" for i, ip in enumerate(echoshield.RADAR_IPS)]
        filter_options.append(f"RADA ({rada.RADA_IP})")
        self.filter_dropdown = ttk.Combobox(top, textvariable=self.radar_filter_var, values=filter_options, state="readonly", width=20)
        self.filter_dropdown.pack(side="right")
        ttk.Label(top, text="Filter:").pack(side="right", padx=(10, 2))
        
        # custom angle buttons
        self.custom_angle_var = tk.IntVar(value=0)
        self.angle_frame = ttk.Frame(top)
        self.angle_frame.pack(side="right", padx=(0, 10))
        
        ttk.Button(self.angle_frame, text="<", width=2, command=self._dec_angle).pack(side="left")
        self.angle_label = ttk.Label(self.angle_frame, text="0°", width=4, anchor="center")
        self.angle_label.pack(side="left")
        ttk.Button(self.angle_frame, text=">", width=2, command=self._inc_angle).pack(side="left")
        
        # heading mode dropdown
        self.heading_mode_var = tk.StringVar(value="True North Up")
        self.heading_dropdown = ttk.Combobox(top, textvariable=self.heading_mode_var, 
                                             values=["True North Up", "Radar Heading Up", "Custom"], 
                                             state="readonly", width=16)
        self.heading_dropdown.pack(side="right", padx=2)
        self.heading_dropdown.bind("<<ComboboxSelected>>", self._on_heading_mode_change)
        ttk.Label(top, text="Heading:").pack(side="right", padx=(10, 2))
        
        # max range selector
        self.max_range_var = tk.StringVar(value="1000")
        self.range_dropdown = ttk.Combobox(top, textvariable=self.max_range_var, 
                                           values=["500", "1000", "2000", "3000", "5000", "10000"], 
                                           width=8)
        self.range_dropdown.pack(side="right", padx=2)
        self.range_dropdown.bind("<<ComboboxSelected>>", self._on_range_change)
        self.range_dropdown.bind("<Return>", self._on_range_change)
        ttk.Label(top, text="Max Range (m):").pack(side="right", padx=(10, 2))
        
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # left table
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side="left", fill="y", padx=(0, 10))
        
        columns = ("tid", "classification")
        self.table = ttk.Treeview(left_frame, columns=columns, show="headings", height=20, selectmode="browse")
        self.table.heading("tid", text="TRACK ID")
        self.table.heading("classification", text="CLASSIFICATION")
        self.table.column("tid", width=120, anchor="center")
        self.table.column("classification", width=180, anchor="center")
        self.table.pack(fill="both", expand=True)
        self.table.bind("<<TreeviewSelect>>", self._select_row)
        
        # classification filters
        filter_frame = ttk.LabelFrame(left_frame, text="Classification Filters", padding=5)
        filter_frame.pack(fill="x", pady=(5, 0))
        
        self.class_filters = {}
        classifications = [
            "UAV (Multi-Rotor)", "UAV (Fixed-Wing)", "Walker", "Plane", 
            "Bird", "Vehicle", "Clutter", "Undeclared"
        ]
        
        for i, cls in enumerate(classifications): #creates the checkboxes and assigns them boolean variable
            var = tk.BooleanVar(value=True)
            self.class_filters[cls] = var
            cb = ttk.Checkbutton(filter_frame, text=cls, variable=var)
            cb.grid(row=i // 2, column=i % 2, sticky="w", padx=(0, 10))
            
        # fusion settings
        fusion_frame = ttk.LabelFrame(left_frame, text="Track Fusion", padding=5)
        fusion_frame.pack(fill="x", pady=(5, 0))
        
        self.fusion_threshold_var = tk.DoubleVar(value=20.0)
        
        def _on_fusion_slider(val):
            self.c2.fusion_threshold = float(val)
            self.fusion_label_var.set(f"Sensitivity: {float(val):.1f}m")
            
        self.fusion_label_var = tk.StringVar(value="Sensitivity: 20.0m")
        ttk.Label(fusion_frame, textvariable=self.fusion_label_var).pack(anchor="w")
        slider = ttk.Scale(fusion_frame, from_=3.0, to=100.0, orient="horizontal", variable=self.fusion_threshold_var, command=_on_fusion_slider)
        slider.pack(fill="x", pady=(0, 5))
        
        # center: display
        center_frame = ttk.Frame(main_frame)
        center_frame.pack(side="left", fill="both", expand=True)
        self.radar_display = RadarDisplay(center_frame, self, bg="#0a1812")
        self.radar_display.pack(fill="both", expand=True)
        
        # right: details panel
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side="right", fill="y", padx=(10, 0))
        
        panel = ttk.LabelFrame(right_frame, text="SELECTED TARGET", padding=10)
        panel.pack(fill="x")
        
        for i, name in enumerate(self.fields):
            ttk.Label(panel, text=f"{name}:").grid(row=i, column=0, sticky="e", padx=(8,3), pady=8)
            ttk.Label(panel, textvariable=self.fields[name], width=20).grid(row=i, column=1, sticky="w", pady=8)
            
        self.send_button = ttk.Button(panel, text="ENGAGE (STREAM)", command=self._authorize, state="disabled")
        self.send_button.grid(row=len(self.fields), column=0, columnspan=2, pady=(20, 4), sticky="ew")
        
        self.send_once_button = ttk.Button(panel, text="ENGAGE (ONE-SHOT)", command=self._authorize_once, state="disabled")
        self.send_once_button.grid(row=len(self.fields)+1, column=0, columnspan=2, pady=(4, 4), sticky="ew")
        
        self.reset_button = ttk.Button(panel, text="RESET", command=self._reset)
        self.reset_button.grid(row=len(self.fields)+2, column=0, columnspan=2, pady=(4, 4), sticky="ew")
        
        ttk.Label(self.root, textvariable=self.status_var, anchor="w", relief="sunken").pack(fill="x", padx=10, pady=(0, 10))

    def _dec_angle(self):
        self.heading_mode_var.set("Custom")
        new_angle = (self.custom_angle_var.get() - 10) % 360
        self.custom_angle_var.set(new_angle)
        self.angle_label.config(text=f"{new_angle}°")
        if hasattr(self, 'radar_display'):
            self.radar_display.set_rotation(new_angle)

    def _inc_angle(self):
        self.heading_mode_var.set("Custom")
        new_angle = (self.custom_angle_var.get() + 10) % 360
        self.custom_angle_var.set(new_angle)
        self.angle_label.config(text=f"{new_angle}°")
        if hasattr(self, 'radar_display'):
            self.radar_display.set_rotation(new_angle)

    def _on_heading_mode_change(self, event=None):
        mode = self.heading_mode_var.get()
        if not hasattr(self, 'radar_display'):
            return
        if mode == "True North Up":
            self.radar_display.set_rotation(0)
        elif mode == "Radar Heading Up":
            import echoshield
            import rada
            heading = rada.RADA_YAW if rada.RADA_YAW != 0.0 else echoshield.RADAR_YAW
            self.radar_display.set_rotation(heading)
        elif mode == "Custom":
            self.radar_display.set_rotation(self.custom_angle_var.get())

    def _on_range_change(self, event=None): #adjusts range
        try:
            new_range = float(self.max_range_var.get())
            if new_range > 0 and hasattr(self, 'radar_display'):
                self.radar_display.max_range = new_range
        except ValueError:
            pass # ignore invalid inputs

    def _select_target_by_id(self, tid):
        if isinstance(tid, int):
            iid = f"F_{tid}"
            display_tid = f"F-{tid}"
        else:
            ip, track_id = tid
            iid = f"{ip}_{track_id}"
            import echoshield
            import rada
            if ip == rada.RADA_IP:
                display_tid = f"RADA-{hex(track_id)}"
            else:
                idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                display_tid = f"R{idx}-{hex(track_id)}"
            
        # update table selection
        for child_iid in self.table.get_children():
            if child_iid == iid:
                self.table.selection_set(iid)
                self.table.see(iid)
                break
        
        self.c2.selected_target_id = tid
        if isinstance(tid, int):
            self.c2.selected_target_details = self.c2.fused_targets.get(tid, {})
        else:
            self.c2.selected_target_details = self.c2.target_kinematics.get(tid, {})
        
        self.status_var.set(f"Selected {display_tid}; updating...")
        self.send_button.config(state="normal")
        self.send_once_button.config(state="normal")
        
        for name in ["Latitude", "Longitude", "Target altitude", "Range (slant)", "Bearing"]:
            self.fields[name].set("—")

    def _select_row(self, event):
        selected = self.table.selection()
        if not selected: return
        iid = selected[0]
        
        if iid.startswith("F_"):
            tid = int(iid.split("_")[1])
            display_tid = f"F-{tid}"
            self.c2.selected_target_id = tid
            self.c2.selected_target_details = self.c2.fused_targets.get(tid, {})
        else:
            ip, track_id_str = iid.split('_', 1)
            tid = (ip, int(track_id_str))
            import echoshield
            import rada
            if ip == rada.RADA_IP:
                display_tid = f"RADA-{hex(tid[1])}"
            else:
                idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                display_tid = f"R{idx}-{hex(tid[1])}"
            self.c2.selected_target_id = tid
            self.c2.selected_target_details = self.c2.target_kinematics.get(tid, {}) 
        
        self.status_var.set(f"Selected {display_tid}; updating...")
        self.send_button.config(state="normal")
        self.send_once_button.config(state="normal")
        
        for name in ["Latitude", "Longitude", "Target altitude", "Range (slant)", "Bearing"]:
            self.fields[name].set("—")

    def _authorize(self):
        if self.c2.selected_target_id:
            self.c2.authorized_target_id = self.c2.selected_target_id
            self.c2.target_locked = False
            
            tid = self.c2.authorized_target_id
            if isinstance(tid, int):
                display_tid = f"F-{tid}"
            else:
                ip, track_id = tid
                import echoshield
                import rada
                if ip == rada.RADA_IP:
                    display_tid = f"RADA-{hex(track_id)}"
                else:
                    idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                    display_tid = f"R{idx}-{hex(track_id)}"
            
            self.status_var.set(f"TARGET AUTHORIZED {display_tid}. Engaging...")

    def _authorize_once(self):
        if self.c2.selected_target_id:
            tid = self.c2.selected_target_id
            self.c2.send_coordinate_once(tid)
            
            if isinstance(tid, int):
                display_tid = f"F-{tid}"
            else:
                ip, track_id = tid
                import echoshield
                import rada
                if ip == rada.RADA_IP:
                    display_tid = f"RADA-{hex(track_id)}"
                else:
                    idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                    display_tid = f"R{idx}-{hex(track_id)}"
            
            self.status_var.set(f"ONE-SHOT SENT {display_tid}.")

    def _reset(self):
        self.c2.selected_target_id = None
        self.c2.authorized_target_id = None
        self.c2.target_locked = False
        
        self.table.selection_remove(self.table.selection())
        self.status_var.set("Target deselected and system reset.")
        self.send_button.config(state="disabled")
        self.send_once_button.config(state="disabled")
        
        for name in self.fields:
            self.fields[name].set("—")

    def refresh(self):
        filter_val = self.radar_filter_var.get()
        filter_ip = None
        if "(" in filter_val and ")" in filter_val:
            filter_ip = filter_val.split("(")[1].split(")")[0]
            
        with self.c2.lock:
            if filter_ip is None:
                active = {fid: fdata['classification'] for fid, fdata in self.c2.fused_targets.items()}
                kinematics = self.c2.fused_targets.copy()
            else:
                active = self.c2.active_targets.copy()
                kinematics = self.c2.target_kinematics.copy()
        
        import echoshield
        for tid, classification in active.items():
            if isinstance(tid, int):
                iid = f"F_{tid}"
                display_tid = f"F-{tid}"
            else:
                ip, track_id = tid
                if filter_ip is not None and ip != filter_ip:
                    continue
                iid = f"{ip}_{track_id}"
                import rada
                if ip == rada.RADA_IP:
                    display_tid = f"RADA-{hex(track_id)}"
                else:
                    idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                    display_tid = f"R{idx}-{hex(track_id)}"
                
            if classification in self.class_filters and not self.class_filters[classification].get():
                continue
                
            if self.table.exists(iid):
                self.table.item(iid, values=(display_tid, classification))
            else:
                self.table.insert("", "end", iid=iid, values=(display_tid, classification))
                
        for iid in self.table.get_children():
            if iid.startswith("F_"):
                tid = int(iid.split("_")[1])
            else:
                ip, track_id_str = iid.split('_', 1)
                tid = (ip, int(track_id_str))
            
            classification = active.get(tid)
            should_remove = False
            
            if tid not in active:
                should_remove = True
            elif filter_ip is not None and not isinstance(tid, int) and tid[0] != filter_ip:
                should_remove = True
            elif classification in self.class_filters and not self.class_filters[classification].get():
                should_remove = True
                
            if should_remove:
                self.table.delete(iid)
                
        if self.c2.selected_target_id:
            # check for fresh kinematics
            details = kinematics.get(self.c2.selected_target_id, self.c2.selected_target_details)
            self.c2.selected_target_details = details
            
            tid = self.c2.selected_target_id
            if isinstance(tid, int):
                display_tid = f"F-{tid}"
            else:
                ip, track_id = tid
                import rada
                if ip == rada.RADA_IP:
                    display_tid = f"RADA-{hex(track_id)}"
                else:
                    idx = echoshield.RADAR_IPS.index(ip) + 1 if ip in echoshield.RADAR_IPS else '?'
                    display_tid = f"R{idx}-{hex(track_id)}"
            
            self.fields["Track ID"].set(display_tid)
            self.fields["Classification"].set(active.get(self.c2.selected_target_id, "Unknown"))
            
            if details:
                self.fields["Latitude"].set(f"{details.get('lat', 0):.7f}")
                self.fields["Longitude"].set(f"{details.get('lon', 0):.7f}")
                self.fields["Target altitude"].set(f"{details.get('alt', 0):.1f} m")
                self.fields["Range (slant)"].set(f"{details.get('range_m', 0):.1f} m")
                self.fields["Bearing"].set(f"{details.get('bearing_deg', 0):.1f}°")
                if not self.c2.authorized_target_id:
                    self.status_var.set(f"Viewing details for {display_tid}")
                    
        # update radar display
        if hasattr(self, 'radar_display'):
            filtered_active = {}
            for k, v in active.items():
                if filter_ip is not None and not isinstance(k, int) and k[0] != filter_ip:
                    continue
                if v in self.class_filters and not self.class_filters[v].get():
                    continue
                filtered_active[k] = v
            self.radar_display.redraw(filtered_active, self.c2.selected_target_id, kinematics)
                
        self.root.after(250, self.refresh)

    def on_close(self):
        self.c2.running = False
        self.root.destroy()


def main():
    root = tk.Tk()
    root.withdraw() 
    
    c2 = PrimaryC2()
    
    import rada
    rada_manager = rada.RadaManager(c2)
    rada_manager.start()
    
    extract = messagebox.askyesno("GPS Extraction", "Do you want to extract the GPS data from the radar?")
    if extract:
        c2.fetch_and_update_gps()
        
    c2.initialize_radar()
    
    import echoshield
    for ip in echoshield.RADAR_IPS:
        listener = threading.Thread(target=c2.track_listener_thread, args=(ip,), daemon=True)
        listener.start()
    
    root.deiconify() 
    app = OperatorApp(root, c2)
    app.rada_manager = rada_manager
    root.mainloop()

if __name__ == "__main__":
    main()
