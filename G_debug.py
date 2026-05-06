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
        self.root.title("超市机器人串口可视化调试工具 v1.1")
        self.root.geometry("1100x750") # 扩大窗口以容纳画布
        
        self.ser = None
        self.is_receiving = False
        self.auto_poll_enabled = tk.BooleanVar(value=False)
        
        # 机器人状态数据
        self.pose_ideal = {'x': 250, 'y': 400, 'theta': 0} # 对应初始化值
        self.pose_real = {'x': 250, 'y': 400, 'theta': 0}
        
        # 绘制开关
        self.show_ideal = tk.BooleanVar(value=True)
        self.show_real = tk.BooleanVar(value=True)
        
        # 场地参数 (单位: mm)
        self.FIELD_W = 3100
        self.FIELD_H = 2600
        self.SCALE = 0.2  # 缩放比例：1mm = 0.2px
        
        self.setup_ui()
        self.draw_field() # 初始绘制场地
        self.schedule_poll() # 启动轮询检查

    def setup_ui(self):
        # 左侧控制面板
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

        # --- 2. 可视化控制区域 ---
        view_frame = ttk.LabelFrame(left_frame, text="地图控制")
        view_frame.pack(fill="x", pady=5)
        ttk.Checkbutton(view_frame, text="自动获取坐标 (GETC/Rpose)", variable=self.auto_poll_enabled).pack(anchor="w", padx=5)
        ttk.Checkbutton(view_frame, text="显示理想位置 (黄色)", variable=self.show_ideal, command=self.update_map).pack(anchor="w", padx=5)
        ttk.Checkbutton(view_frame, text="显示真实位置 (红色)", variable=self.show_real, command=self.update_map).pack(anchor="w", padx=5)

        # --- 3. 指令区域 (保留原逻辑) ---
        cmd_frame = ttk.LabelFrame(left_frame, text="快速指令")
        cmd_frame.pack(fill="x", pady=5)
        self.commands = {
            "GOTOpose": {"desc": "移动坐标", "params": ["X", "Y", "Theta"]},
            "GOTOHigh": {"desc": "移动到高度", "params": ["Height"]},
            "GETdist": {"desc": "查询距离", "params": []},
            "AdjustPose": {"desc": "位置校准", "params": []},
            "GETCpose": {"desc": "查询理想位置", "params": []},
            "GETRpose": {"desc": "查询实际位置", "params": []},
            "reset": {"desc": "重启", "params": []}
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

        # 画布区域
        self.canvas = tk.Canvas(right_frame, width=self.FIELD_W * self.SCALE, height=self.FIELD_H * self.SCALE, bg="black")
        self.canvas.pack(pady=5)

        # 日志区域
        self.log_text = tk.Text(right_frame, height=12, state="disabled", background="#f0f0f0")
        self.log_text.pack(fill="both", expand=True)

    def draw_field(self):
        """根据比赛规则绘制静态场地[cite: 2]"""
        self.canvas.delete("field")
        s = self.SCALE
        # 绘制围栏 (3100x2600)
        self.canvas.create_rectangle(0, 0, self.FIELD_W*s, self.FIELD_H*s, outline="white", width=2, tags="field")
        
        # 绘制起点区 (红色 800x500)[cite: 2]
        self.canvas.create_rectangle(0, 0, 500*s, 800*s, outline="red", width=2, tags="field")
        self.canvas.create_text(250*s, 400*s, text="START", fill="red", tags="field")
        
        # 绘制终点区 (蓝色 800x500)[cite: 2]
        self.canvas.create_rectangle((3100-500)*s, (2600-800)*s, 3100*s, 2600*s, outline="blue", width=2, tags="field")
        self.canvas.create_text((3100-250)*s, (2600-400)*s, text="FINISH", fill="blue", tags="field")

        # 绘制货架 (根据代码坐标推算位置)
        # 货架1在大约 X=0, 货架2在大约 X=2600
        self.canvas.create_rectangle(0, 800*s, (0+500)*s, (800+1000)*s, outline="gray", tags="field")
        self.canvas.create_rectangle(2600*s, 800*s, (2600+500)*s, (800+1000)*s, outline="gray", tags="field")

    def draw_robot(self, pose, color, is_ideal=False):
        """在地图上绘制机器人（矩形+方向箭头）"""
        s = self.SCALE
        x, y = pose['x'] * s, pose['y'] * s
        angle_rad = math.radians(pose['theta'])
        
        # 机器人尺寸 (假设 490x670mm)
        rw, rh = 490 * s, 670 * s
        
        # 计算矩形四个顶点（考虑旋转）
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        points = [
            (x + rw/2*cos_a - rh/2*sin_a, y + rw/2*sin_a + rh/2*cos_a),
            (x - rw/2*cos_a - rh/2*sin_a, y - rw/2*sin_a + rh/2*cos_a),
            (x - rw/2*cos_a + rh/2*sin_a, y - rw/2*sin_a - rh/2*cos_a),
            (x + rw/2*cos_a + rh/2*sin_a, y + rw/2*sin_a - rh/2*cos_a)
        ]
        
        style = {"outline": color, "width": 2, "tags": "robot"}
        if is_ideal: style["dash"] = (4, 4) # 理想位置用虚线（虚影效果）

        self.canvas.create_polygon(points, fill="", **style)
        
        # 绘制方向箭头（指向 X 正方向，即车头）
        arrow_len = 30 * s
        self.canvas.create_line(x, y, x + arrow_len*cos_a, y + arrow_len*sin_a, 
                               fill=color, arrow=tk.LAST, tags="robot")

    def update_map(self):
        self.canvas.delete("robot")
        if self.show_ideal.get():
            self.draw_robot(self.pose_ideal, "yellow", is_ideal=True)
        if self.show_real.get():
            self.draw_robot(self.pose_real, "red", is_ideal=False)

    def schedule_poll(self):
        """定时轮询逻辑"""
        if self.ser and self.ser.is_open and self.auto_poll_enabled.get():
            try:
                self.ser.write(b"GETCpose\n")
                time.sleep(0.05)
                self.ser.write(b"GETRpose\n")
            except:
                pass
        self.root.after(800, self.schedule_poll) # 每800ms请求一次

    def receive_data(self):
        while self.is_receiving:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    
                    self.log(f"接收 << {line}")
                    
                    # 匹配理想坐标: "Current pose: x=..., y=..., theta=..."
                    if "Current pose" in line:
                        matches = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                        if len(matches) >= 3:
                            self.pose_ideal['x'] = float(matches[0])
                            self.pose_ideal['y'] = float(matches[1])
                            self.pose_ideal['theta'] = float(matches[2])
                            self.root.after(0, self.update_map)
                            
                    # 匹配实际坐标[cite: 4]: "REALLY pose: x=..., y=..., theta=..."
                    elif "REALLY pose" in line:
                        matches = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                        if len(matches) >= 3:
                            self.pose_real['x'] = float(matches[0])
                            self.pose_real['y'] = float(matches[1])
                            self.pose_real['theta'] = float(matches[2])
                            self.root.after(0, self.update_map)
                except Exception as e:
                    print(f"解析错误: {e}")
            time.sleep(0.01)

    # --- 以下为串口基础功能的封装 (保持与原代码结构一致) ---
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
        msg = f"{cmd} {' '.join(vals)}\n"
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