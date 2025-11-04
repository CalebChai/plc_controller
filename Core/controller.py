import serial
import time
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread

# ===== 串口配置 =====
PORT = "COM15"
BAUD = 115200
TIMEOUT = 1

# ===== 运动指令映射（按需改）=====
CMD_FORWARD = "W"
CMD_BACK    = "S"   # 若下位机要 'B' 表示后退，这里改成 "B"
CMD_LEFT    = "A"
CMD_RIGHT   = "D"
CMD_STOP    = "X"   # 可选：空格发送停止

# ===== 全局状态 =====
ser = None
output_status = [False]*10   # False=低, True=高
text_box = None
status_tip = None
out_labels = None

def send_raw(cmd_str: str):
    if ser and ser.is_open:
        ser.write(cmd_str.encode() + b'\n')
        log(f"已发送: {cmd_str}")
    else:
        log("串口未打开，发送失败", is_err=True)

def open_serial():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        log(f"已打开串口 {PORT}")
    except Exception as e:
        ser = None
        messagebox.showerror("错误", f"无法打开串口 {PORT}\n{e}")

def init_all_low():
    """上电后将所有端口置0"""
    for i in range(10):
        send_raw(f"{i}0")
        output_status[i] = False
    refresh_io_labels()

def toggle_port(port_idx: int):
    """反向当前状态并发送"""
    if not (0 <= port_idx <= 9):
        return
    output_status[port_idx] = not output_status[port_idx]
    cmd = f"{port_idx}{1 if output_status[port_idx] else 0}"
    send_raw(cmd)
    refresh_io_labels()

def on_keypress(event):
    """键盘：0–9 反向某端口；W/A/S/D 控制运动；空格发送停止（可选）"""
    # 避免在日志框内输入时触发
    if event.widget == text_box:
        return

    ch = event.char.lower() if event.char else ""

    # 端口反向：数字键 0-9
    if ch.isdigit():
        port_idx = int(ch)
        if 0 <= port_idx <= 9:
            toggle_port(port_idx)
            return

    # 运动：w/a/s/d
    if ch == 'w':
        send_raw(CMD_FORWARD)
        set_status("前进")
    elif ch == 's':
        send_raw(CMD_BACK)
        set_status("后退")
    elif ch == 'a':
        send_raw(CMD_LEFT)
        set_status("左转")
    elif ch == 'd':
        send_raw(CMD_RIGHT)
        set_status("右转")
    elif ch == ' ':
        send_raw(CMD_STOP) 
        set_status("停止")

def set_status(msg: str):
    if status_tip:
        status_tip.config(text=f"状态：{msg}")

def log(msg, is_err=False):
    if text_box:
        text_box.insert(tk.END, f"{'[ERR] ' if is_err else ''}{msg}\n")
        text_box.see(tk.END)
    else:
        print(msg)

def refresh_io_labels():
    for i in range(10):
        out_labels[i]["text"] = f"输出口 {i}: {'高' if output_status[i] else '低'}"

def read_serial_worker():
    while True:
        try:
            if ser and ser.in_waiting:
                data = ser.readline().decode(errors="ignore").strip()
                if data:
                    log(f"收到: {data}")
            else:
                time.sleep(0.01)
        except Exception as e:
            log(f"串口读取异常: {e}", is_err=True)
            time.sleep(0.2)

def build_gui():
    global text_box, out_labels, status_tip
    win = tk.Tk()
    win.title("上位机：端口反向 + WASD 运动控制")
    win.geometry("820x420")

    # 顶部提示与状态
    top = tk.Frame(win)
    top.pack(fill="x", padx=10, pady=(8, 4))

    tk.Label(top, text="数字 0–9：反向对应端口；W/A/S/D：前/左/后/右；空格：停止",
             font=("Arial", 11)).pack(side="left")
    status_tip = tk.Label(top, text="状态：就绪", fg="#006400")
    status_tip.pack(side="right")

    # 左：端口状态与按钮
    left = ttk.LabelFrame(win, text="端口状态/控制")
    left.pack(side="left", fill="y", padx=10, pady=10)

    out_labels = []
    for i in range(10):
        row = tk.Frame(left)
        row.pack(anchor="w", pady=3)
        lbl = tk.Label(row, text=f"输出口 {i}: 低", width=16, anchor="w")
        lbl.pack(side="left", padx=(6, 8))
        out_labels.append(lbl)
        tk.Button(row, text="切换", width=5, command=lambda p=i: toggle_port(p)).pack(side="left")

    # 中：运动控制按钮（WASD布局）
    mid = ttk.LabelFrame(win, text="运动控制 (W/A/S/D)")
    mid.pack(side="left", padx=10, pady=10)

    # 用网格摆出小方向键
    for r in range(3):
        mid.grid_rowconfigure(r, minsize=38)
        mid.grid_columnconfigure(r, minsize=60)

    tk.Button(mid, text="W", width=6, command=lambda: (send_raw(CMD_FORWARD), set_status("前进"))).grid(row=0, column=1, padx=6, pady=6)
    tk.Button(mid, text="A", width=6, command=lambda: (send_raw(CMD_LEFT), set_status("左转"))).grid(row=1, column=0, padx=6, pady=6)
    tk.Button(mid, text="S", width=6, command=lambda: (send_raw(CMD_BACK), set_status("后退"))).grid(row=1, column=1, padx=6, pady=6)
    tk.Button(mid, text="D", width=6, command=lambda: (send_raw(CMD_RIGHT), set_status("右转"))).grid(row=1, column=2, padx=6, pady=6)
    tk.Button(mid, text="STOP", width=6, command=lambda: (send_raw(CMD_STOP), set_status("停止"))).grid(row=2, column=1, padx=6, pady=6)

    # 右：串口收发日志
    right = ttk.LabelFrame(win, text="串口收发")
    right.pack(side="right", fill="both", expand=True, padx=10, pady=10)

    text_box = tk.Text(right, height=20, width=50)
    text_box.pack(fill="both", expand=True, padx=6, pady=6)

    # 绑定键盘
    win.bind("<KeyPress>", on_keypress)

    # 底部退出
    tk.Button(win, text="退出", command=win.quit).pack(side="bottom", pady=6)

    return win

if __name__ == "__main__":
    app = build_gui()   # 先创建 GUI 和 text_box
    open_serial()       # 再尝试打开串口
    init_all_low()      # 上电置零

    # 启动读线程
    Thread(target=read_serial_worker, daemon=True).start()

    app.mainloop()

    if ser and ser.is_open:
        ser.close()
