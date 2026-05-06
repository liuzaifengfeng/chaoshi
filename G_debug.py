import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class RobotDebugger:
    def __init__(self, root):
        self.root = root
        self.root.title("超市机器人串口调试工具 v1.0")
        self.root.geometry("700x650")
        
        self.ser = None
        self.is_receiving = False

        # --- 1. 串口连接区域 ---
        conn_frame = ttk.LabelFrame(root, text="串口配置")
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="端口:").pack(side="left", padx=5)
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_ports(), width=15)
        self.port_combo.pack(side="left", padx=5)
        
        self.refresh_btn = ttk.Button(conn_frame, text="刷新", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=5)

        # --- 2. 指令选择与参数输入 ---
        cmd_frame = ttk.LabelFrame(root, text="调试指令")
        cmd_frame.pack(fill="x", padx=10, pady=5)

        # 指令定义 (根据你的 main.cpp 整理)
        self.commands = {
            "GOTOpose": {"desc": "移动到坐标", "params": ["X (mm)", "Y (mm)", "Theta (度)"]},
            "GOTOHeight": {"desc": "抬升大臂高度", "params": ["高度 (0-1000)"]},
            "AdjustPose": {"desc": "雷达自动校准位置", "params": []},
            "GETCpose": {"desc": "获取理想坐标", "params": []},
            "GETRpose": {"desc": "获取雷达实际坐标", "params": []},
            "GETdist": {"desc": "读取雷达原始距离", "params": []},
            "PWM": {"desc": "控制舵机/机构", "params": ["通道(1-5)", "角度(0-180)"]},
            "movepose": {"desc": "步进移动", "params": ["方向Y", "速度", "停止(1)"]},
            "EMMpos": {"desc": "闭环电机位置控制", "params": ["地址", "方向", "脉冲数"]},
            "reset": {"desc": "重启系统", "params": []},
            "help": {"desc": "显示帮助", "params": []}
        }

        ttk.Label(cmd_frame, text="选择指令:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.cmd_selector = ttk.Combobox(cmd_frame, values=list(self.commands.keys()), state="readonly")
        self.cmd_selector.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.cmd_selector.bind("<<ComboboxSelected>>", self.on_cmd_select)

        self.desc_label = ttk.Label(cmd_frame, text="说明: 请选择一个指令", foreground="blue")
        self.desc_label.grid(row=1, column=0, columnspan=3, padx=5, pady=2, sticky="w")

        # 参数输入框
        self.param_entries = []
        self.param_labels = []
        for i in range(3):
            lbl = ttk.Label(cmd_frame, text=f"参数 {i+1}:")
            lbl.grid(row=2+i, column=0, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(cmd_frame)
            ent.grid(row=2+i, column=1, padx=5, pady=2, sticky="w")
            self.param_labels.append(lbl)
            self.param_entries.append(ent)

        self.send_btn = ttk.Button(cmd_frame, text="发送指令", command=self.send_command, state="disabled")
        self.send_btn.grid(row=5, column=1, pady=10)

        # --- 3. 数据接收区域 ---
        log_frame = ttk.LabelFrame(root, text="回传日志 (解析位置信息)")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=15, state="disabled", background="#f0f0f0")
        self.log_text.pack(fill="both", expand=True, side="left")
        
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)

    def get_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        self.port_combo['values'] = self.get_ports()

    def toggle_connect(self):
        if self.ser is None:
            try:
                port = self.port_combo.get()
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.is_receiving = True
                self.connect_btn.config(text="断开")
                self.send_btn.config(state="normal")
                threading.Thread(target=self.receive_data, daemon=True).start()
                self.log("成功连接到 " + port)
            except Exception as e:
                messagebox.showerror("错误", f"无法连接串口: {e}")
        else:
            self.is_receiving = False
            time.sleep(0.2)
            self.ser.close()
            self.ser = None
            self.connect_btn.config(text="连接")
            self.send_btn.config(state="disabled")
            self.log("串口已断开")

    def on_cmd_select(self, event):
        cmd_name = self.cmd_selector.get()
        info = self.commands[cmd_name]
        self.desc_label.config(text=f"说明: {info['desc']}")
        
        # 动态显示需要的参数输入框
        for i, entry in enumerate(self.param_entries):
            if i < len(info['params']):
                self.param_labels[i].config(text=info['params'][i])
                entry.config(state="normal")
            else:
                self.param_labels[i].config(text="-")
                entry.delete(0, tk.END)
                entry.config(state="disabled")

    def send_command(self):
        if not self.ser: return
        cmd_name = self.cmd_selector.get()
        params = [e.get() for e in self.param_entries if str(e['state']) == "normal"]
        
        # 按照 main.cpp 要求的格式：CMD P1 P2 P3
        cmd_str = f"{cmd_name} {' '.join(params)}\n"
        self.ser.write(cmd_str.encode('utf-8'))
        self.log(f"发送 >> {cmd_str.strip()}")

    def receive_data(self):
        while self.is_receiving:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log(f"接收 << {line}")
                        # 解析位置信息的逻辑
                        if "pose" in line or "distances" in line:
                            self.highlight_info(line)
                except:
                    pass
            time.sleep(0.01)

    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")

    def highlight_info(self, line):
        # 如果回传包含坐标或距离，可以在这里做专门的 UI 展示或弹窗
        # 目前直接在日志中显示
        pass

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDebugger(root)
    root.mainloop()
    