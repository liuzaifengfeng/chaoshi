import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import threading
import time

class DebugTool:
    def __init__(self, root):
        self.root = root
        self.root.title("机器人调试工具")
        self.root.geometry("800x600")
        
        self.ser = None
        self.is_connected = False
        self.receive_thread = None
        self.stop_thread = False
        
        self.debug_commands = [
            {"name": "GOTOpose", "desc": "移动机器人到指定位置", "params": [("x坐标", "float", "0"), ("y坐标", "float", "0"), ("角度theta", "float", "0")]},
            {"name": "GETCpose", "desc": "获取当前理想位置", "params": []},
            {"name": "GETRpose", "desc": "获取雷达推算的实际位置", "params": []},
            {"name": "AdjustPose", "desc": "调整机器人位置", "params": []},
            {"name": "GETdist", "desc": "获取雷达原始距离(CH0-CH3)", "params": []},
            {"name": "movepose", "desc": "移动机器人", "params": [("方向Y", "int", "1"), ("速度", "int", "10"), ("停止标志", "int", "0")]},
            {"name": "EMMpos", "desc": "设置电机位置", "params": [("电机地址", "int", "1"), ("方向", "int", "0"), ("脉冲数", "int", "1000")]},
            {"name": "GOTOHeight", "desc": "设置机器人高度", "params": [("高度", "int", "0")]},
            {"name": "PWM", "desc": "设置PWM占空比", "params": [("通道", "int", "1"), ("角度", "int", "90")]},
            {"name": "reset", "desc": "重启ESP32", "params": []},
            {"name": "help", "desc": "显示帮助信息", "params": []}
        ]
        
        self.create_widgets()
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        port_frame = ttk.Frame(main_frame)
        port_frame.grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=5)
        
        ttk.Label(port_frame, text="串口:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar(value="COM3")
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=10)
        self.port_combo['values'] = ['COM1', 'COM2', 'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(port_frame, text="波特率:").pack(side=tk.LEFT, padx=5)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(port_frame, textvariable=self.baud_var, width=10)
        self.baud_combo['values'] = ['9600', '19200', '38400', '57600', '115200', '230400']
        self.baud_combo.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(port_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        cmd_frame = ttk.Frame(main_frame)
        cmd_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(cmd_frame, text="调试命令:").pack(side=tk.LEFT, padx=5)
        self.cmd_var = tk.StringVar()
        self.cmd_combo = ttk.Combobox(cmd_frame, textvariable=self.cmd_var, width=20, state='readonly')
        self.cmd_combo['values'] = [cmd['name'] for cmd in self.debug_commands]
        self.cmd_combo.pack(side=tk.LEFT, padx=5)
        self.cmd_combo.bind('<<ComboboxSelected>>', self.on_command_selected)
        
        self.desc_label = ttk.Label(cmd_frame, text="", foreground='blue')
        self.desc_label.pack(side=tk.LEFT, padx=10)
        
        self.params_frame = ttk.Frame(main_frame)
        self.params_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        self.param_entries = []
        
        send_frame = ttk.Frame(main_frame)
        send_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        self.send_btn = ttk.Button(send_frame, text="发送命令", command=self.send_command)
        self.send_btn.pack(side=tk.LEFT, padx=5)
        
        self.clear_btn = ttk.Button(send_frame, text="清空日志", command=self.clear_log)
        self.clear_btn.pack(side=tk.LEFT, padx=5)
        
        log_frame = ttk.Frame(main_frame)
        log_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        main_frame.rowconfigure(4, weight=1)
        main_frame.columnconfigure(0, weight=1)
        
        ttk.Label(log_frame, text="串口日志:").pack(side=tk.TOP, anchor=tk.W)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, width=100, height=20)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.on_command_selected(None)
        
    def on_command_selected(self, event):
        for widget in self.params_frame.winfo_children():
            widget.destroy()
        self.param_entries.clear()
        
        cmd_name = self.cmd_var.get()
        for cmd in self.debug_commands:
            if cmd['name'] == cmd_name:
                self.desc_label.config(text=cmd['desc'])
                row = 0
                for param in cmd['params']:
                    ttk.Label(self.params_frame, text=f"{param[0]}:").grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)
                    entry = ttk.Entry(self.params_frame, width=15)
                    entry.insert(0, param[2])
                    entry.grid(row=row, column=1, padx=5, pady=2)
                    self.param_entries.append(entry)
                    row += 1
                break
        
    def toggle_connection(self):
        if not self.is_connected:
            try:
                self.ser = serial.Serial(
                    port=self.port_var.get(),
                    baudrate=int(self.baud_var.get()),
                    timeout=1
                )
                self.is_connected = True
                self.connect_btn.config(text="断开")
                self.log_text.insert(tk.END, f"已连接到 {self.port_var.get()} {self.baud_var.get()}\n")
                self.log_text.see(tk.END)
                
                self.stop_thread = False
                self.receive_thread = threading.Thread(target=self.receive_data)
                self.receive_thread.start()
            except Exception as e:
                messagebox.showerror("连接失败", str(e))
        else:
            self.stop_thread = True
            if self.receive_thread:
                self.receive_thread.join()
            self.ser.close()
            self.is_connected = False
            self.connect_btn.config(text="连接")
            self.log_text.insert(tk.END, "已断开连接\n")
            self.log_text.see(tk.END)
            
    def receive_data(self):
        while not self.stop_thread:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        self.log_text.insert(tk.END, f"<<< {data}\n")
                        self.log_text.see(tk.END)
                        self.parse_response(data)
                time.sleep(0.01)
            except Exception as e:
                if not self.stop_thread:
                    self.log_text.insert(tk.END, f"接收错误: {e}\n")
                    self.log_text.see(tk.END)
                break
                
    def parse_response(self, data):
        if "Current pose" in data:
            parts = data.split(',')
            if len(parts) >= 3:
                x = parts[0].split('=')[1].strip()
                y = parts[1].split('=')[1].strip()
                theta = parts[2].split('=')[1].strip()
                messagebox.showinfo("当前位置", f"X: {x} mm\nY: {y} mm\nTheta: {theta} 度")
        elif "REALLY pose" in data:
            parts = data.split(',')
            if len(parts) >= 3:
                x = parts[0].split('=')[1].strip()
                y = parts[1].split('=')[1].strip()
                theta = parts[2].split('=')[1].strip()
                messagebox.showinfo("实际位置", f"X: {x} mm\nY: {y} mm\nTheta: {theta} 度")
        elif "LiDAR distances" in data:
            parts = data.split(',')
            if len(parts) >= 4:
                ch0 = parts[0].split('=')[1].strip()
                ch1 = parts[1].split('=')[1].strip()
                ch2 = parts[2].split('=')[1].strip()
                ch3 = parts[3].split('=')[1].strip()
                messagebox.showinfo("雷达距离", f"CH0: {ch0} mm\nCH1: {ch1} mm\nCH2: {ch2} mm\nCH3: {ch3} mm")
                
    def send_command(self):
        if not self.is_connected:
            messagebox.showwarning("未连接", "请先连接串口")
            return
            
        cmd_name = self.cmd_var.get()
        if not cmd_name:
            messagebox.showwarning("未选择命令", "请选择调试命令")
            return
            
        params = []
        for entry in self.param_entries:
            params.append(entry.get())
            
        cmd_str = cmd_name
        if params:
            cmd_str += " " + " ".join(params)
        
        try:
            self.ser.write((cmd_str + "\n").encode('utf-8'))
            self.log_text.insert(tk.END, f">>> {cmd_str}\n")
            self.log_text.see(tk.END)
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            
    def clear_log(self):
        self.log_text.delete(1.0, tk.END)
        
    def on_closing(self):
        if self.is_connected:
            self.toggle_connection()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DebugTool(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()