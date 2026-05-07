import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import re
import math

class RobotDebugger:
    def __init__(self, root):
        self.root = root
        self.root.title("超市机器人串口可视化调试工具 v1.8")
        self.root.geometry("1100x850") 
        
        self.ser = None
        self.is_receiving = False
        self.auto_poll_enabled = tk.BooleanVar(value=False)
        self.run_mode = tk.StringVar(value="Release") 
        
        # --- 机器人状态数据 ---
        self.pose_ideal = {'x': 250, 'y': 400, 'theta': 0} 
        self.pose_real = {'x': 0, 'y': 0, 'theta': 0}
        
        # --- 滤波与阈值参数 ---
        self.ALPHA = 0.8
        self.DIST_THRESHOLD = 200.0
        self.ANGLE_THRESHOLD = 30.0
        
        # 绘制开关
        self.show_ideal = tk.BooleanVar(value=True)
        self.show_real = tk.BooleanVar(value=True)
        
        # 场地参数 (单位: mm)
        self.FIELD_W = 3100
        self.FIELD_H = 2600
        self.SCALE = 0.2 
        
        self.cmd_library = {
            "Debug": {
                "posedebug": {"desc": "调试开关", "params": ["Hz"]},
                "GETdist":  {"desc": "查询距离", "params": []},
                "GETCpose": {"desc": "查询理想位置", "params": []},
                "GETRpose": {"desc": "查询实际位置", "params": []},
                "AdjustPose": {"desc": "位置校准", "params": []},
                "PWM": {"desc": "设置PWM", "params": ["addr", "angle"]},
                "reset":    {"desc": "重启ESP32", "params": []},
                "GOTOpose": {"desc": "移动坐标", "params": ["X", "Y", "Theta"]},
                "GOTOHeight": {"desc": "移动高度", "params": ["Height"]},
                "movepose": {"desc": "直线移动", "params": ["Y", "speed", "stop"]}
            },
            "Release": {
                "ready": {"desc": "准备运行", "params": []},
                "orderget": {"desc": "获取订单成功", "params": []},
                "get0": {"desc": "发现提货商品", "params": []},
                "get1": {"desc": "发现1号商品", "params": []},
                "get2": {"desc": "发现2号商品", "params": []},
                "get3": {"desc": "发现3号商品", "params": []},
                "get4": {"desc": "发现4号商品", "params": []},
                "true": {"desc": "确认顾客", "params": []}
            }
        }
        
        self.setup_ui()
        self.draw_field() 

    def setup_ui(self):
        # 1. 创建整体框架
        left_frame = ttk.Frame(self.root)
        left_frame.pack(side="left", fill="y", padx=10, pady=5)
        
        right_frame = ttk.Frame(self.root)
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=5)

        # 2. 创建画布与 Log 组件
        self.canvas = tk.Canvas(right_frame, width=self.FIELD_W * self.SCALE, height=self.FIELD_H * self.SCALE, bg="black")
        self.canvas.pack(pady=5)
        
        # 绑定鼠标事件
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<Leave>", lambda e: self.canvas.delete("crosshair"))

        self.log_text = tk.Text(right_frame, height=15, state="disabled", background="#f0f0f0")
        self.log_text.pack(fill="both", expand=True)

        # 3. 串口配置
        conn_frame = ttk.LabelFrame(left_frame, text="串口配置")
        conn_frame.pack(fill="x", pady=5)
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_ports(), width=15)
        self.port_combo.pack(padx=5, pady=2)
        btn_box = ttk.Frame(conn_frame)
        btn_box.pack()
        ttk.Button(btn_box, text="刷新", command=self.refresh_ports).pack(side="left", padx=2)
        self.connect_btn = ttk.Button(btn_box, text="连接", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=2)

        # 4. 模式切换
        mode_frame = ttk.LabelFrame(left_frame, text="运行模式")
        mode_frame.pack(fill="x", pady=5)
        ttk.Radiobutton(mode_frame, text="调试模式 (Debug)", variable=self.run_mode, 
                        value="Debug", command=self.update_cmd_menu).pack(anchor="w", padx=10)
        ttk.Radiobutton(mode_frame, text="运行模式 (Release)", variable=self.run_mode, 
                        value="Release", command=self.update_cmd_menu).pack(anchor="w", padx=10)

        # 5. 地图控制
        view_frame = ttk.LabelFrame(left_frame, text="地图控制")
        view_frame.pack(fill="x", pady=5)
        freq_frame = ttk.Frame(view_frame)
        freq_frame.pack(fill="x", padx=5, pady=2)
        ttk.Label(freq_frame, text="频率:").pack(side="left")
        self.freq_combo = ttk.Combobox(freq_frame, values=["1Hz", "2Hz", "5Hz", "10Hz"], width=5, state="readonly")
        self.freq_combo.current(1)
        self.freq_combo.pack(side="left", padx=5)
        ttk.Checkbutton(view_frame, text="开启上报", variable=self.auto_poll_enabled, command=self.toggle_posedebug).pack(anchor="w", padx=5)
        ttk.Checkbutton(view_frame, text="显示理想", variable=self.show_ideal, command=self.update_map).pack(anchor="w", padx=5)
        ttk.Checkbutton(view_frame, text="显示真实", variable=self.show_real, command=self.update_map).pack(anchor="w", padx=5)

        # 6. 指令区域
        cmd_frame = ttk.LabelFrame(left_frame, text="快速指令")
        cmd_frame.pack(fill="x", pady=5)
        self.cmd_selector = ttk.Combobox(cmd_frame, state="readonly")
        self.cmd_selector.pack(padx=5, pady=2)
        self.cmd_selector.bind("<<ComboboxSelected>>", self.on_cmd_select)
        self.param_box = ttk.Frame(cmd_frame)
        self.param_box.pack()
        self.param_entries = [ttk.Entry(self.param_box, width=8) for _ in range(3)]
        for e in self.param_entries: e.pack(side="left", padx=2)
        self.send_btn = ttk.Button(cmd_frame, text="发送指令", command=self.send_command, state="disabled")
        self.send_btn.pack(pady=5)
        
        self.update_cmd_menu()

    def on_mouse_move(self, event):
        self.canvas.delete("crosshair")
        cx, cy = event.x, event.y
        real_x = cx / self.SCALE
        real_y = self.FIELD_H - (cy / self.SCALE)
        
        if 0 <= real_x <= self.FIELD_W and 0 <= real_y <= self.FIELD_H:
            self.canvas.create_line(0, cy, self.FIELD_W * self.SCALE, cy, fill="#3F3F3F", dash=(2, 2), tags="crosshair")
            self.canvas.create_line(cx, 0, cx, self.FIELD_H * self.SCALE, fill="#3F3F3F", dash=(2, 2), tags="crosshair")
            
            coord_text = f"X:{int(real_x)} Y:{int(real_y)}"
            tx = cx + 10 if cx < (self.FIELD_W * self.SCALE - 80) else cx - 80
            ty = cy - 15 if cy > 20 else cy + 15
            self.canvas.create_text(tx, ty, text=coord_text, fill="cyan", anchor="nw", tags="crosshair", font=("Consolas", 9))

    def update_cmd_menu(self):
        mode = self.run_mode.get()
        cmds = list(self.cmd_library[mode].keys())
        self.cmd_selector['values'] = cmds
        self.cmd_selector.set(cmds[0] if cmds else "")
        self.on_cmd_select(None)
        self.log(f"已切换至 {mode} 指令集")

    def to_canvas(self, x, y):
        cx = x * self.SCALE
        cy = (self.FIELD_H - y) * self.SCALE 
        return cx, cy

    def draw_field(self):
        self.canvas.delete("field")
        s = self.SCALE
        # 边界
        self.canvas.create_rectangle(0, 0, self.FIELD_W*s, self.FIELD_H*s, outline="white", width=2, tags="field")
        
        # --- [新增] 上边界 4 条 300mm 粗线，间隔 5px ---
        # 300mm 在画布上是 300 * 0.2 = 60 像素
        # 5px 间隔。起点设在场地中段附近。
        start_x = (self.FIELD_W / 2 - 600) * s 
        for i in range(4):
            x_pos = start_x + (i * (300 * s + 5)) 
            self.canvas.create_line(x_pos, 5, x_pos + (300 * s), 5, fill="green", width=6, tags="field")

        # --- [新增] 下边界一条 500mm 粗线 ---
        # 500mm 在画布上是 500 * 0.2 = 100 像素
        bottom_y = self.FIELD_H * s
        self.canvas.create_line(self.FIELD_W * s / 2 - 50 * s, bottom_y -2, 
                                self.FIELD_W * s / 2 + 450 * s, bottom_y -2, fill="green", width=6, tags="field")

        # 起点与终点
        x1, y1 = self.to_canvas(0, 0); x2, y2 = self.to_canvas(500, 800)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="red", width=2, tags="field")
        self.canvas.create_text(250*s, (2600-250)*s, text="START", fill="red", tags="field")
        
        x1, y1 = self.to_canvas(3100-500, 2600-800); x2, y2 = self.to_canvas(3100, 2600)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="blue", width=2, tags="field")        
        self.canvas.create_text((3100-250)*s, 250*s, text="FINISH", fill="blue", tags="field")
        
        # 货架 (左侧与右侧)
        shelf_coords = [
            (0, 800, 500, 1800),      # 左侧货架
            (2600, 800, 3100, 1800)   # 右侧货架
        ]
        
        for sc in shelf_coords:
            sx1, sy1, sx2, sy2 = sc
            px1, py1 = self.to_canvas(sx1, sy1)
            px2, py2 = self.to_canvas(sx2, sy2)
            self.canvas.create_rectangle(px1, py1, px2, py2, outline="gray", tags="field")
            
            # --- 货架等分线划分 (5个区域需4条线) ---
            height = abs(sy2 - sy1)
            for i in range(1, 5):
                line_y = sy1 + (height / 5) * i
                lx1, ly1 = self.to_canvas(sx1, line_y)
                lx2, ly2 = self.to_canvas(sx2, line_y)
                self.canvas.create_line(lx1, ly1, lx2, ly2, fill="#4A4A4A", tags="field")

    def draw_robot(self, pose, color, is_ideal=False):
        s = self.SCALE
        cx, cy = self.to_canvas(pose['x'], pose['y'])
        angle_rad = math.radians(pose['theta'])
        rw, rh = 490 * s, 670 * s 
        
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        
        # 矩形四个顶点
        pts = [(rw/2, rh/2), (-rw/2, rh/2), (-rw/2, -rh/2), (rw/2, -rh/2)]
        points = []
        for rx, ry in pts:
            nx = rx * cos_a - ry * sin_a
            ny = rx * sin_a + ry * cos_a
            points.append(cx + nx); points.append(cy - ny)
        
        style = {"outline": color, "width": 2, "tags": "robot"}
        if is_ideal: style["dash"] = (4, 4)
        self.canvas.create_polygon(points, fill="", **style)
        
        # --- 车头朝向小三角形 (右侧边下1/3处) ---
        tri_size = 50 * s
        # 小三角形局部坐标
        tri_pts = [
            (rw/2, -rh/6),              # 顶点
            (rw/2 - tri_size, -rh/6 + tri_size), # 左上
            (rw/2 - tri_size, -rh/6 - tri_size)  # 左下
        ]
        rotated_tri = []
        for rx, ry in tri_pts:
            nx = rx * cos_a - ry * sin_a
            ny = rx * sin_a + ry * cos_a
            rotated_tri.extend([cx + nx, cy - ny])
        
        self.canvas.create_polygon(rotated_tri, fill="", outline=color, width=2, tags="robot")

        # 原有的朝向箭头
        al = 100 * s
        self.canvas.create_line(cx, cy, cx + al*cos_a, cy - al*sin_a, fill=color, arrow=tk.LAST, tags="robot")

    def update_map(self):
        self.canvas.delete("robot")
        if self.show_ideal.get():
            self.draw_robot(self.pose_ideal, "yellow", is_ideal=True)
        if self.show_real.get():
            r = self.pose_real
            if not (abs(r['x']) < 0.1 and abs(r['y']) < 0.1 and abs(r['theta']) < 0.1):
                self.draw_robot(self.pose_real, "red", is_ideal=False)

    def receive_data(self):
        while self.is_receiving:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    
                    if "Debug" in line:
                        self.root.after(0, lambda: [self.run_mode.set("Debug"), self.update_cmd_menu()])
                    elif "Release" in line:
                        self.root.after(0, lambda: [self.run_mode.set("Release"), self.update_cmd_menu()])

                    if line.startswith("Cpose") or line.startswith("Rpose"):
                        matches = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                        if len(matches) >= 3:
                            nx, ny, nt = float(matches[0]), float(matches[1]), float(matches[2])
                            if line.startswith("Cpose"):
                                self.pose_ideal.update({'x': nx, 'y': ny, 'theta': nt})
                            else:
                                if nx == 0 and ny == 0 and nt == 0: continue
                                self.pose_real['x'] = self.ALPHA * nx + (1 - self.ALPHA) * self.pose_real['x']
                                self.pose_real['y'] = self.ALPHA * ny + (1 - self.ALPHA) * self.pose_real['y']
                                self.pose_real['theta'] = self.ALPHA * nt + (1 - self.ALPHA) * self.pose_real['theta']
                            self.root.after(0, self.update_map)
                    else:
                        self.log(f"接收 << {line}")
                except Exception as e:
                    print(f"解析错误: {e}")
            time.sleep(0.01)

    def toggle_posedebug(self):
        if not self.ser: return
        hz = self.freq_combo.get().replace("Hz", "") if self.auto_poll_enabled.get() else "0"
        msg = f"posedebug {hz}\n"
        self.ser.write(msg.encode())

    def get_ports(self): return [p.device for p in serial.tools.list_ports.comports()]
    def refresh_ports(self): self.port_combo['values'] = self.get_ports()
    
    def toggle_connect(self):
        if self.ser is None:
            try:
                self.ser = serial.Serial(self.port_combo.get(), 115200, timeout=0.1)
                self.is_receiving = True
                threading.Thread(target=self.receive_data, daemon=True).start()
                self.connect_btn.config(text="断开"); self.send_btn.config(state="normal")
                self.log("连接成功")
            except Exception as e: messagebox.showerror("错误", str(e))
        else:
            self.is_receiving = False; self.ser.close(); self.ser = None
            self.connect_btn.config(text="连接"); self.send_btn.config(state="disabled")

    def on_cmd_select(self, event):
        mode = self.run_mode.get()
        cmd = self.cmd_selector.get()
        if not cmd or cmd not in self.cmd_library[mode]: return
        params = self.cmd_library[mode][cmd]["params"]
        for i, ent in enumerate(self.param_entries):
            ent.delete(0, tk.END)
            ent.config(state="normal" if i < len(params) else "disabled")

    def send_command(self):
        if not self.ser: return
        mode = self.run_mode.get()
        cmd = self.cmd_selector.get()
        vals = [e.get() for e in self.param_entries if str(e['state']) == "normal"]
        msg = f"{cmd} {' '.join(vals)}".strip() + "\n"
        self.ser.write(msg.encode()); self.log(f"发送 >> {msg.strip()}")

    def log(self, msg):
        if hasattr(self, 'log_text'):
            self.log_text.config(state="normal")
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
            self.log_text.see(tk.END); self.log_text.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk(); app = RobotDebugger(root); root.mainloop()