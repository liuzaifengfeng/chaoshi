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
        self.root.title("超市机器人串口可视化调试工具 v1.2")
        self.root.geometry("1100x800") 
        
        self.ser = None
        self.is_receiving = False
        self.auto_poll_enabled = tk.BooleanVar(value=False)
        # 频率设置：默认 2Hz (500ms)
        self.poll_freq = tk.StringVar(value="2Hz")
        
        # 机器人状态数据
        self.pose_ideal = {'x': 250, 'y': 400, 'theta': 0} 
        self.pose_real = {'x': 250, 'y': 400, 'theta': 0}
        
        # 绘制开关
        self.show_ideal = tk.BooleanVar(value=True)
        self.show_real = tk.BooleanVar(value=True)
        
        # 场地参数 (单位: mm)
        self.FIELD_W = 3100
        self.FIELD_H = 2600
        self.SCALE = 0.2  
        
        self.setup_ui()
        self.draw_field() 
        self.schedule_poll() 

    def setup_ui(self):
        left_frame = ttk.Frame(self.root)
        left_frame.pack(side="left", fill="y", padx=10, pady=5)

        # --- 1. 串口连接区域 ---
        conn_frame = ttk.LabelFrame(left_frame, text="串口配置")
        conn_frame.pack(fill="x", pady=5)
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_ports(), width=15)
        self.port_combo.pack(padx=5, pady=2)
        
        btn_box = ttk.Frame(conn_frame)
        btn_box.pack()
        self.refresh_btn = ttk.Button(btn_box, text="刷新", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=2)
        self.connect_btn = ttk.Button(btn_box, text="连接", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=2)

        # --- 2. 可视化控制区域 (新增频率选择) ---
        view_frame = ttk.LabelFrame(left_frame, text="地图控制")
        view_frame.pack(fill="x", pady=5)
        ttk.Checkbutton(view_frame, text="自动获取坐标", variable=self.auto_poll_enabled).pack(anchor="w", padx=5)
        
        freq_box = ttk.Frame(view_frame)
        freq_box.pack(fill="x", padx=5)
        ttk.Label(freq_box, text="刷新频率:").pack(side="left")
        self.freq_selector = ttk.Combobox(freq_box, values=["1Hz", "2Hz", "5Hz"], textvariable=self.poll_freq, width=5, state="readonly")
        self.freq_selector.pack(side="left", padx=5)

        ttk.Checkbutton(view_frame, text="显示理想位置 (黄色)", variable=self.show_ideal, command=self.update_map).pack(anchor="w", padx=5)
        ttk.Checkbutton(view_frame, text="显示真实位置 (红色)", variable=self.show_real, command=self.update_map).pack(anchor="w", padx=5)

        # --- 3. 指令区域 (完善所有调试函数) ---
        cmd_frame = ttk.LabelFrame(left_frame, text="快速指令")
        cmd_frame.pack(fill="x", pady=5)
        self.commands = {
            "GOTOpose": {"desc": "移动坐标", "params": ["X", "Y", "Theta"]},
            "GOTOHigh": {"desc": "移动高度", "params": ["Height"]},
            "PWM": {"desc": "设置PWM", "params": ["addr", "angle"]},
            "movepose": {"desc": "移动到位置", "params": ["Y", "speed", "stop"]},
            "GETdist":  {"desc": "查询距离", "params": []},
            "GETCpose": {"desc": "查询理想位置", "params": []},
            "GETRpose": {"desc": "查询实际位置", "params": []},
            "reset":    {"desc": "重启ESP32", "params": []}
        }
        self.cmd_selector = ttk.Combobox(cmd_frame, values=list(self.commands.keys()), state="readonly")
        self.cmd_selector.pack(padx=5, pady=2)
        self.cmd_selector.bind("<<ComboboxSelected>>", self.on_cmd_select)
        
        self.param_box = ttk.Frame(cmd_frame)
        self.param_box.pack()
        self.param_entries = [ttk.Entry(self.param_box, width=8) for _ in range(3)]
        for e in self.param_entries: e.pack(side="left", padx=2)
        
        self.send_btn = ttk.Button(cmd_frame, text="发送指令", command=self.send_command, state="disabled")
        self.send_btn.pack(pady=5)

        # 右侧地图与日志面板
        right_frame = ttk.Frame(self.root)
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=5)

        # 画布区域 (背景设为深色以便观察)
        self.canvas = tk.Canvas(right_frame, width=self.FIELD_W * self.SCALE, height=self.FIELD_H * self.SCALE, bg="#1a1a1a")
        self.canvas.pack(pady=5)

        self.log_text = tk.Text(right_frame, height=15, state="disabled", background="#f0f0f0")
        self.log_text.pack(fill="both", expand=True)

    def to_canvas(self, x, y):
        """将物理坐标(左下角0,0)转换为画布坐标"""
        cx = x * self.SCALE
        cy = (self.FIELD_H - y) * self.SCALE
        return cx, cy

    def draw_field(self):
        """绘制场地，(0,0)位于左下角"""
        self.canvas.delete("field")
        s = self.SCALE
        h = self.FIELD_H
        
        # 绘制外围框
        self.canvas.create_rectangle(0, 0, self.FIELD_W*s, h*s, outline="white", width=2, tags="field")
        
        # 绘制起点区 (左下角 500x800)
        x1, y1 = self.to_canvas(0, 0)
        x2, y2 = self.to_canvas(500, 800)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="red", width=2, tags="field")
        tx, ty = self.to_canvas(250, 400)
        self.canvas.create_text(tx, ty, text="START", fill="red", tags="field")
        
        # 绘制终点区 (右上角 500x800)
        x1, y1 = self.to_canvas(3100-500, 2600-800)
        x2, y2 = self.to_canvas(3100, 2600)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="blue", width=2, tags="field")
        tx, ty = self.to_canvas(3100-250, 2600-400)
        self.canvas.create_text(tx, ty, text="FINISH", fill="blue", tags="field")

        # 绘制货架
        # 左侧货架
        x1, y1 = self.to_canvas(0, 800)
        x2, y2 = self.to_canvas(500, 1800)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="gray", tags="field")
        # 右侧货架
        x1, y1 = self.to_canvas(2600, 800)
        x2, y2 = self.to_canvas(3100, 1800)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="gray", tags="field")

    def draw_robot(self, pose, color, is_ideal=False):
        s = self.SCALE
        # 物理坐标转画布坐标
        cx, cy = self.to_canvas(pose['x'], pose['y'])
        
        # 机器人尺寸 (490x670mm)
        rw, rh = 490 * s, 670 * s
        angle_rad = math.radians(pose['theta'])
        
        # 注意：由于Y轴翻转，标准坐标系下的正旋转在画布上表现为逆时针
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        
        # 定义原始矩形顶点(中心在0,0)
        raw_points = [
            (rw/2, rh/2), (-rw/2, rh/2), (-rw/2, -rh/2), (rw/2, -rh/2)
        ]
        
        points = []
        for rx, ry in raw_points:
            # 旋转变换
            nx = rx * cos_a - ry * sin_a
            ny = rx * sin_a + ry * cos_a
            # 映射到画布，注意 ny 符号取反，因为画布 Y 向下
            points.append(cx + nx)
            points.append(cy - ny) 
        
        style = {"outline": color, "width": 2, "tags": "robot"}
        if is_ideal: style["dash"] = (4, 4)

        self.canvas.create_polygon(points, fill="", **style)
        
        # 方向箭头
        arrow_len = 200 * s
        self.canvas.create_line(cx, cy, cx + arrow_len*cos_a, cy - arrow_len*sin_a, 
                               fill=color, arrow=tk.LAST, tags="robot")

    def update_map(self):
        self.canvas.delete("robot")
        if self.show_ideal.get():
            self.draw_robot(self.pose_ideal, "yellow", is_ideal=True)
        if self.show_real.get():
            self.draw_robot(self.pose_real, "red", is_ideal=False)

    def schedule_poll(self):
        """定时轮询逻辑，支持自定义频率"""
        if self.ser and self.ser.is_open and self.auto_poll_enabled.get():
            try:
                self.ser.write(b"GETCpose\n")
                time.sleep(0.02) # 缩短指令间隔提高响应
                self.ser.write(b"GETRpose\n")
            except: pass
        
        # 解析频率下拉框内容
        freq_str = self.poll_freq.get()
        interval = 500 # 默认 2Hz
        if freq_str == "1Hz": interval = 1000
        elif freq_str == "2Hz": interval = 500
        elif freq_str == "5Hz": interval = 200
            
        self.root.after(interval, self.schedule_poll)

    def receive_data(self):
        while self.is_receiving:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    self.log(f"接收 << {line}")
                    
                    if "Current pose" in line or "REALLY pose" in line:
                        matches = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                        if len(matches) >= 3:
                            target = self.pose_ideal if "Current pose" in line else self.pose_real
                            target['x'] = float(matches[0])
                            target['y'] = float(matches[1])
                            target['theta'] = float(matches[2])
                            self.root.after(0, self.update_map)
                except Exception as e: print(f"解析错误: {e}")
            time.sleep(0.01)

    def get_ports(self): return [p.device for p in serial.tools.list_ports.comports()]
    def refresh_ports(self): self.port_combo['values'] = self.get_ports()
    
    def toggle_connect(self):
        if self.ser is None:
            try:
                self.ser = serial.Serial(self.port_combo.get(), 115200, timeout=0.1)
                self.is_receiving = True
                threading.Thread(target=self.receive_data, daemon=True).start()
                self.connect_btn.config(text="断开")
                self.send_btn.config(state="normal")
                self.log("连接成功")
            except Exception as e: messagebox.showerror("错误", str(e))
        else:
            self.is_receiving = False
            self.ser.close()
            self.ser = None
            self.connect_btn.config(text="连接")
            self.send_btn.config(state="disabled")

    def on_cmd_select(self, event):
        cmd = self.cmd_selector.get()
        params = self.commands[cmd]["params"]
        for i, ent in enumerate(self.param_entries):
            ent.delete(0, tk.END)
            ent.config(state="normal" if i < len(params) else "disabled")

    def send_command(self):
        if not self.ser: return
        cmd = self.cmd_selector.get()
        vals = [e.get() for e in self.param_entries if str(e['state']) == "normal"]
        msg = f"{cmd}{' ' + ' '.join(vals) if vals else ''}\n"
        self.ser.write(msg.encode())
        self.log(f"发送 >> {msg.strip()}")

    def log(self, msg):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDebugger(root)
    root.mainloop()