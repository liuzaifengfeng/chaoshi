import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import re
import math
import asyncio
import websockets
import json

class RobotDebugger:
    def __init__(self, root):
        self.root = root
        self.root.title("超市机器人串口可视化调试工具 v1.8")
        self.root.geometry("1100x850") 
        
        self.ser = None              # 串口对象
        self.is_receiving = False    # 串口接收标志
        self.auto_poll_enabled = tk.BooleanVar(value=False)
        self.run_mode = tk.StringVar(value="Release") 
        
        # --- WebSocket 相关 ---
        self.ws = None               # WebSocket 连接对象
        self.ws_loop = None          # asyncio 事件循环
        self.ws_thread = None        # 事件循环所在线程
        self.is_ws = False           # 当前是否使用 WebSocket
        
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

    # ====================== UI 布局（完全保持原样）======================
    def setup_ui(self):
        left_frame = ttk.Frame(self.root)
        left_frame.pack(side="left", fill="y", padx=10, pady=5)
        
        right_frame = ttk.Frame(self.root)
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=5)

        self.canvas = tk.Canvas(right_frame, width=self.FIELD_W * self.SCALE, height=self.FIELD_H * self.SCALE, bg="black")
        self.canvas.pack(pady=5)
        
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<Leave>", lambda e: self.canvas.delete("crosshair"))

        self.log_text = tk.Text(right_frame, height=15, state="disabled", background="#f0f0f0")
        self.log_text.pack(fill="both", expand=True)

        conn_frame = ttk.LabelFrame(left_frame, text="串口配置")
        conn_frame.pack(fill="x", pady=5)
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_ports(), width=15)
        self.port_combo.pack(padx=5, pady=2)
        btn_box = ttk.Frame(conn_frame)
        btn_box.pack()
        ttk.Button(btn_box, text="刷新", command=self.refresh_ports).pack(side="left", padx=2)
        self.connect_btn = ttk.Button(btn_box, text="连接", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=2)

        mode_frame = ttk.LabelFrame(left_frame, text="运行模式")
        mode_frame.pack(fill="x", pady=5)
        ttk.Radiobutton(mode_frame, text="调试模式 (Debug)", variable=self.run_mode, 
                        value="Debug", command=self.update_cmd_menu).pack(anchor="w", padx=10)
        ttk.Radiobutton(mode_frame, text="运行模式 (Release)", variable=self.run_mode, 
                        value="Release", command=self.update_cmd_menu).pack(anchor="w", padx=10)

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

    # ====================== 原有工具方法（保持不变）======================
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
        self.canvas.create_rectangle(0, 0, self.FIELD_W*s, self.FIELD_H*s, outline="white", width=2, tags="field")
        
        start_x = (self.FIELD_W / 2 - 600) * s 
        for i in range(4):
            x_pos = start_x + (i * (300 * s + 5)) 
            self.canvas.create_line(x_pos, 5, x_pos + (300 * s), 5, fill="green", width=6, tags="field")

        bottom_y = self.FIELD_H * s
        self.canvas.create_line(self.FIELD_W * s / 2 - 250 * s, bottom_y -2, 
                                self.FIELD_W * s / 2 + 250 * s, bottom_y -2, fill="green", width=6, tags="field")

        x1, y1 = self.to_canvas(0, 0); x2, y2 = self.to_canvas(500, 800)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="red", width=2, tags="field")
        self.canvas.create_text(250*s, (2600-250)*s, text="START", fill="red", tags="field")
        
        x1, y1 = self.to_canvas(3100-500, 2600-800); x2, y2 = self.to_canvas(3100, 2600)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline="blue", width=2, tags="field")        
        self.canvas.create_text((3100-250)*s, 250*s, text="FINISH", fill="blue", tags="field")
        
        shelf_coords = [
            (0, 800, 500, 1800),
            (2600, 800, 3100, 1800)
        ]
        
        for sc in shelf_coords:
            sx1, sy1, sx2, sy2 = sc
            px1, py1 = self.to_canvas(sx1, sy1)
            px2, py2 = self.to_canvas(sx2, sy2)
            self.canvas.create_rectangle(px1, py1, px2, py2, outline="gray", tags="field")
            
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
        
        pts = [(rw/2, rh/2), (-rw/2, rh/2), (-rw/2, -rh/2), (rw/2, -rh/2)]
        points = []
        for rx, ry in pts:
            nx = rx * cos_a - ry * sin_a
            ny = rx * sin_a + ry * cos_a
            points.append(cx + nx); points.append(cy - ny)
        
        style = {"outline": color, "width": 2, "tags": "robot"}
        if is_ideal: style["dash"] = (4, 4)
        self.canvas.create_polygon(points, fill="", **style)
        
        tri_size = 50 * s
        tri_pts = [
            (rw/2, -rh/6),
            (rw/2 - tri_size, -rh/6 + tri_size),
            (rw/2 - tri_size, -rh/6 - tri_size)
        ]
        rotated_tri = []
        for rx, ry in tri_pts:
            nx = rx * cos_a - ry * sin_a
            ny = rx * sin_a + ry * cos_a
            rotated_tri.extend([cx + nx, cy - ny])
        
        self.canvas.create_polygon(rotated_tri, fill="", outline=color, width=2, tags="robot")

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

    # ====================== 统一数据解析 ======================
    def parse_and_update(self, msg: str):
        """解析传入的消息，同时兼容 JSON 格式 (WebSocket) 与原有的 Cpose/Rpose 正则格式"""
        # 1. 尝试 JSON 解析
        try:
            data = json.loads(msg)
            if isinstance(data, dict):
                pose_type = data.get("type", "Rpose")   # 默认为实时位姿
                x = data.get("x")
                y = data.get("y")
                theta = data.get("theta")
                if x is not None and y is not None and theta is not None:
                    if pose_type == "Cpose":
                        self.pose_ideal.update({'x': x, 'y': y, 'theta': theta})
                    else:  # Rpose 或未知
                        if not (x == 0 and y == 0 and theta == 0):
                            self.pose_real['x'] = self.ALPHA * x + (1 - self.ALPHA) * self.pose_real['x']
                            self.pose_real['y'] = self.ALPHA * y + (1 - self.ALPHA) * self.pose_real['y']
                            self.pose_real['theta'] = self.ALPHA * theta + (1 - self.ALPHA) * self.pose_real['theta']
                    self.root.after(0, self.update_map)
                return  # 处理完 JSON 立即返回
        except (json.JSONDecodeError, ValueError):
            pass   # 不是 JSON，继续原有逻辑

        # 2. 原有的 Cpose / Rpose 正则格式
        if msg.startswith("Cpose") or msg.startswith("Rpose"):
            matches = re.findall(r"[-+]?\d*\.\d+|\d+", msg)
            if len(matches) >= 3:
                nx, ny, nt = float(matches[0]), float(matches[1]), float(matches[2])
                if msg.startswith("Cpose"):
                    self.pose_ideal.update({'x': nx, 'y': ny, 'theta': nt})
                else:
                    if nx == 0 and ny == 0 and nt == 0:
                        return
                    self.pose_real['x'] = self.ALPHA * nx + (1 - self.ALPHA) * self.pose_real['x']
                    self.pose_real['y'] = self.ALPHA * ny + (1 - self.ALPHA) * self.pose_real['y']
                    self.pose_real['theta'] = self.ALPHA * nt + (1 - self.ALPHA) * self.pose_real['theta']
                self.root.after(0, self.update_map)
            return

        # 3. 模式切换关键字 (DEBUG 串口可能打印)
        if "Debug" in msg:
            self.root.after(0, lambda: [self.run_mode.set("Debug"), self.update_cmd_menu()])
        elif "Release" in msg:
            self.root.after(0, lambda: [self.run_mode.set("Release"), self.update_cmd_menu()])
        else:
            self.log(f"接收 << {msg}")

    # ====================== 串口接收线程（改为调用统一解析） ======================
    def receive_data(self):
        while self.is_receiving:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.parse_and_update(line)   # 统一解析入口
                except Exception as e:
                    print(f"串口读取错误: {e}")
            time.sleep(0.01)

    # ====================== 连接控制（智能切换） ======================
    def toggle_connect(self):
        target = self.port_combo.get().strip()
        # 断开当前连接
        if self.ser or self.is_ws:
            if self.is_ws:
                self._disconnect_ws()
            else:
                self.is_receiving = False
                if self.ser:
                    self.ser.close()
                    self.ser = None
            self.connect_btn.config(text="连接")
            self.send_btn.config(state="disabled")
            self.log("已断开连接")
            return

        # 建立新连接
        if target.startswith("ws://"):
            self._connect_ws(target)
        else:
            self._connect_serial(target)

    def _connect_serial(self, port):
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.is_receiving = True
            threading.Thread(target=self.receive_data, daemon=True).start()
            self.connect_btn.config(text="断开")
            self.send_btn.config(state="normal")
            self.log(f"串口连接成功 -> {port}")
        except Exception as e:
            messagebox.showerror("串口错误", str(e))

    def _connect_ws(self, uri):
        self.is_ws = True
        self.ws_thread = threading.Thread(target=self._run_ws_loop, args=(uri,), daemon=True)
        self.ws_thread.start()
        self.connect_btn.config(text="断开")
        self.send_btn.config(state="normal")
        self.log(f"WebSocket 连接中 -> {uri}")

    def _run_ws_loop(self, uri):
        """在独立线程中启动 asyncio 事件循环"""
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        try:
            self.ws_loop.run_until_complete(self._ws_task(uri))
        except Exception as e:
            self.root.after(0, lambda: self.log(f"WebSocket 异常: {e}"))
        finally:
            self.ws_loop.close()
            self.ws_loop = None
            self.is_ws = False
            self.root.after(0, lambda: self.connect_btn.config(text="连接"))
            self.root.after(0, lambda: self.send_btn.config(state="disabled"))

    async def _ws_task(self, uri):
        """异步连接 WebSocket 并接收消息"""
        try:
            self.ws = await websockets.connect(uri)
            self.root.after(0, lambda: self.log("WebSocket 已连接"))
            async for message in self.ws:
                # 将接收到的消息传递给统一解析函数（必须在主线程安全调用）
                self.root.after(0, lambda m=message: self.parse_and_update(m))
        except websockets.exceptions.ConnectionClosed:
            self.root.after(0, lambda: self.log("WebSocket 连接已关闭"))
        except Exception as e:
            self.root.after(0, lambda: self.log(f"WebSocket 错误: {e}"))
        finally:
            if self.ws:
                await self.ws.close()
                self.ws = None

    def _disconnect_ws(self):
        """安全断开 WebSocket 连接"""
        if self.ws and self.ws_loop:
            asyncio.run_coroutine_threadsafe(self._ws_close(), self.ws_loop)
        if self.ws_loop:
            self.ws_loop.call_soon_threadsafe(self.ws_loop.stop)
        if self.ws_thread and self.ws_thread.is_alive():
            self.ws_thread.join(timeout=2)
        self.is_ws = False
        self.ws = None
        self.ws_loop = None
        self.ws_thread = None

    async def _ws_close(self):
        if self.ws:
            await self.ws.close()

    # ====================== 发送指令（WebSocket / 串口兼容） ======================
    def send_command(self):
        mode = self.run_mode.get()
        cmd = self.cmd_selector.get()
        vals = [e.get() for e in self.param_entries if str(e['state']) == "normal"]
        msg = f"{cmd} {' '.join(vals)}".strip() + "\n"

        if self.is_ws:
            if self.ws and self.ws_loop:
                asyncio.run_coroutine_threadsafe(self._ws_send(msg), self.ws_loop)
                self.log(f"发送(WS) >> {msg.strip()}")
            else:
                self.log("WebSocket 未连接，无法发送")
        elif self.ser:
            try:
                self.ser.write(msg.encode())
                self.log(f"发送(串口) >> {msg.strip()}")
            except Exception as e:
                self.log(f"发送失败: {e}")
        else:
            self.log("未连接任何设备")

    async def _ws_send(self, msg: str):
        if self.ws:
            await self.ws.send(msg)

    # ====================== posedebug 触发（兼容 WebSocket） ======================
    def toggle_posedebug(self):
        if not self.ser and not self.is_ws:
            return
        hz = self.freq_combo.get().replace("Hz", "") if self.auto_poll_enabled.get() else "0"
        msg = f"posedebug {hz}\n"
        if self.is_ws:
            if self.ws and self.ws_loop:
                asyncio.run_coroutine_threadsafe(self._ws_send(msg), self.ws_loop)
        elif self.ser:
            self.ser.write(msg.encode())

    # ====================== 端口列表（包含 WebSocket URL） ======================
    def get_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        # 插入固定 WebSocket 地址，与 COM 端口并列
        ports.append("ws://10.190.108.32.local/ws")
        return ports

    def refresh_ports(self):
        self.port_combo['values'] = self.get_ports()

    def on_cmd_select(self, event):
        mode = self.run_mode.get()
        cmd = self.cmd_selector.get()
        if not cmd or cmd not in self.cmd_library[mode]:
            return
        params = self.cmd_library[mode][cmd]["params"]
        for i, ent in enumerate(self.param_entries):
            ent.delete(0, tk.END)
            ent.config(state="normal" if i < len(params) else "disabled")

    def log(self, msg):
        if hasattr(self, 'log_text'):
            self.log_text.config(state="normal")
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDebugger(root)
    root.mainloop()