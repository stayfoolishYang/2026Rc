"""Presentation helpers for the RC tuning console; no serial or file writes."""
from __future__ import annotations

import tkinter as tk
from tkinter import ttk

INK = "#172B46"
MUTED = "#66768C"
BLUE = "#2463EB"
CANVAS = "#F1F4F8"
LINE = "#DDE5EF"
NAV = "#111E32"


def configure_style(app):
    style = ttk.Style(app)
    style.theme_use("clam")
    app.configure(background=CANVAS)
    app.option_add("*Font", ("Microsoft YaHei UI", 10))
    style.configure(".", font=("Microsoft YaHei UI", 10), background="white", foreground=INK)
    style.configure("TFrame", background="white")
    style.configure("Surface.TFrame", background=CANVAS)
    style.configure("TLabel", background="white", foreground=INK)
    style.configure("Muted.TLabel", foreground=MUTED)
    style.configure("Title.TLabel", font=("Microsoft YaHei UI", 22, "bold"))
    style.configure("Status.TLabel", font=("Microsoft YaHei UI", 10, "bold"))
    style.configure("Safe.TLabel", foreground="#18745A")
    style.configure("Warn.TLabel", foreground="#996014")
    style.configure("TLabelframe", background="white", bordercolor=LINE, relief="solid", borderwidth=1)
    style.configure("TLabelframe.Label", background="white", foreground=INK,
                    font=("Microsoft YaHei UI", 10, "bold"), padding=(4, 0))
    style.configure("TButton", padding=(12, 8), relief="flat", background="#EEF3FA", borderwidth=0,
                    foreground=INK, focuscolor=BLUE)
    style.map("TButton", background=[("disabled", "#F2F4F7"), ("pressed", "#D3E0F5"),
                                     ("active", "#E0EAF8")], foreground=[("disabled", "#9AA8B9")])
    for name, bg, hover in (("Accent", BLUE, "#164FCB"), ("Danger", "#BA3347", "#9C2536")):
        style.configure(f"{name}.TButton", background=bg, foreground="white",
                        font=("Microsoft YaHei UI", 10, "bold"), padding=(14, 9))
        style.map(f"{name}.TButton", background=[("disabled", "#D6DEE9"), ("active", hover),
                                               ("pressed", hover)], foreground=[("disabled", "#7D8BA0"),
                                                                                 ("!disabled", "white")])
    style.configure("Small.TButton", padding=(7, 5), font=("Microsoft YaHei UI", 9))
    style.configure("TEntry", fieldbackground="white", bordercolor="#CFD9E7", padding=7,
                    lightcolor="#CFD9E7", darkcolor="#CFD9E7")
    style.map("TEntry", bordercolor=[("focus", BLUE)])
    style.configure("TSpinbox", fieldbackground="white", bordercolor="#CFD9E7", padding=6,
                    arrowsize=13, lightcolor="#CFD9E7", darkcolor="#CFD9E7")
    style.map("TSpinbox", bordercolor=[("focus", BLUE)])
    style.configure("TCombobox", fieldbackground="white", background="#EEF3FA", padding=7,
                    bordercolor="#CFD9E7", arrowsize=13)
    style.map("TCombobox", fieldbackground=[("readonly", "white")], foreground=[("readonly", INK)])
    style.configure("TCheckbutton", background="white", padding=(3, 6), foreground=INK)
    style.map("TCheckbutton", background=[("active", "#F4F7FC")])
    style.configure("Console.TNotebook", background=CANVAS, borderwidth=0, tabmargins=0)
    style.layout("Console.TNotebook.Tab", [])
    style.configure("Vertical.TScrollbar", background="#C4CFDE", troughcolor="#F0F4F8",
                    borderwidth=0, arrowsize=12, width=11)
    style.configure("Treeview", rowheight=30, borderwidth=0)


def build_shell(app, title, port, baud):
    """Keep the Notebook API while showing page navigation in a persistent rail."""
    app.geometry("1380x900")
    app.minsize(1000, 680)
    rail = tk.Frame(app, background=NAV, width=194)
    rail.pack(side="left", fill="y")
    rail.pack_propagate(False)
    brand = tk.Frame(rail, background=NAV)
    brand.pack(fill="x", padx=22, pady=(28, 28))
    tk.Label(brand, text="RC", background=BLUE, foreground="white", padx=10, pady=6,
             font=("Bahnschrift", 24, "bold")).pack(anchor="w")
    tk.Label(brand, text="硬件调参台", background=NAV, foreground="#F1F6FF",
             font=("Microsoft YaHei UI", 15, "bold")).pack(anchor="w", pady=(13, 3))
    tk.Label(brand, text="CONTROL STUDIO", background=NAV, foreground="#8094B0",
             font=("Bahnschrift", 9)).pack(anchor="w")
    tk.Label(rail, text="工作区", background=NAV, foreground="#7186A3",
             font=("Microsoft YaHei UI", 9)).pack(anchor="w", padx=24, pady=(0, 9))

    nav = tk.Frame(rail, background=NAV)
    nav.pack(fill="x", padx=12)
    footer = tk.Frame(rail, background=NAV)
    footer.pack(side="bottom", fill="x", padx=18, pady=20)
    stop = tk.Button(footer, text="停止元器件测试", command=app._stop_component_test,
                     background="#482737", foreground="#FFACB8", activebackground="#633040",
                     activeforeground="white", relief="flat", borderwidth=0, pady=11,
                     font=("Microsoft YaHei UI", 10, "bold"), cursor="hand2")
    stop.pack(fill="x")
    tk.Label(footer, text="仅元器件测试 · 关闭舵机信号\n不能中断比赛任务和底盘试跑",
             background=NAV, foreground="#AFBDCF", justify="left",
             font=("Microsoft YaHei UI", 8)).pack(anchor="w", pady=(8, 20))
    tk.Label(footer, text="STM32F103ZE", background=NAV, foreground="#D5DFEC",
             font=("Bahnschrift", 11)).pack(anchor="w")
    tk.Label(footer, text="本地设备 / 串口调试", background=NAV, foreground="#8094B0",
             font=("Microsoft YaHei UI", 9)).pack(anchor="w", pady=(3, 0))

    main = tk.Frame(app, background=CANVAS)
    main.pack(side="left", fill="both", expand=True, padx=22, pady=18)
    head = tk.Frame(main, background=CANVAS)
    head.pack(fill="x", pady=(0, 14))
    app.console_title = tk.StringVar(value="参数标定")
    app.console_subtitle = tk.StringVar()
    tk.Label(head, textvariable=app.console_title, background=CANVAS, foreground=INK,
             font=("Microsoft YaHei UI", 22, "bold")).pack(anchor="w")
    tk.Label(head, textvariable=app.console_subtitle, background=CANVAS, foreground=MUTED,
             font=("Microsoft YaHei UI", 10)).pack(anchor="w", pady=(4, 0))

    connection = ttk.Frame(main, padding=(14, 10))
    connection.pack(fill="x", pady=(0, 10))
    ttk.Label(connection, text="设备连接", style="Status.TLabel").grid(row=0, column=0, sticky="w")
    ttk.Label(connection, text=f"{port}  /  {baud}", style="Muted.TLabel").grid(row=0, column=1, padx=14)
    app.connect_button = ttk.Button(connection, text="连接", command=app._toggle_connection,
                                   style="Accent.TButton", width=7)
    app.connect_button.grid(row=0, column=2, padx=(0, 7))
    ttk.Button(connection, text="扫描", command=app._refresh_ports, width=5).grid(row=0, column=3)
    app.console_connection = tk.Label(connection, textvariable=app.connection_var,
                                      background="#F0F3F7", foreground=MUTED, padx=10, pady=5,
                                      font=("Microsoft YaHei UI", 9, "bold"))
    app.console_connection.grid(row=0, column=4, padx=(12, 0), sticky="e")
    connection.columnconfigure(4, weight=1)
    scan = ttk.Label(connection, textvariable=app.port_scan_var, style="Muted.TLabel", wraplength=900,
                     font=("Microsoft YaHei UI", 8))
    scan.grid(row=1, column=0, columnspan=5, sticky="w", pady=(5, 0))
    scan_width = [0]
    def resize_scan(event):
        width = max(300, event.width - 30)
        if width != scan_width[0]:
            scan_width[0] = width
            scan.configure(wraplength=width)
    connection.bind("<Configure>", resize_scan)

    safety = ttk.Frame(main, padding=(12, 7))
    safety.pack(fill="x", pady=(0, 14))
    for col, (text, variable) in enumerate((
        ("运动范围已清空\n允许本次会话机械动作", app.action_armed_var),
        ("升降已人工放到底\n软件零点与实际位置同步", app.lift_synced_var),
        ("转台区域无干涉\n机械臂处于安全高度", app.turntable_safe_var),
    )):
        ttk.Checkbutton(safety, text=text, variable=variable).grid(row=0, column=col, sticky="w", padx=4)
        safety.columnconfigure(col, weight=1, uniform="safe")

    app.notebook = ttk.Notebook(main, style="Console.TNotebook")
    app.notebook.pack(fill="both", expand=True)
    specs = (
        ("参数标定", "参数标定", "设备参数", "按器件调整标定值，修改后预览并保存。"),
        ("任务时序（二级）", "任务等待时间", "真实步骤与时间来源", "按真实动作顺序查看专用等待、公共等待和当前累计时间。"),
        ("任务试跑", "任务试跑", "组合动作", "核对机械状态后，执行单个任务或独立动作。"),
        ("元器件测试", "元器件测试", "逐个检查", "先检测测试固件，再逐个检查电机、舵机和输入。"),
        ("保存 / 编译 / 烧录", "构建与烧录", "保存与部署", "保存参数、编译检查、导出固件，再进行烧录。"),
        ("通信与构建日志", "通信日志", "收发与诊断", "查看串口收发、动作反馈和完整构建记录。"),
    )
    pages, buttons = [], []
    for index, (tab, label, caption, description) in enumerate(specs):
        page = ttk.Frame(app.notebook, padding=12)
        app.notebook.add(page, text=tab)
        pages.append(page)
        button = tk.Button(nav, text=f"{index+1:02d}   {label}", anchor="w", padx=13, pady=13,
                           background=NAV, foreground="#AFBDD0", activebackground="#213452",
                           activeforeground="white", relief="flat", borderwidth=0,
                           font=("Microsoft YaHei UI", 11), cursor="hand2",
                           command=lambda i=index: app.notebook.select(i))
        button.pack(fill="x", pady=3)
        buttons.append(button)
    app.console_nav_buttons = buttons

    def selected(_event=None):
        index = app.notebook.index(app.notebook.select())
        app.console_title.set(specs[index][1])
        app.console_subtitle.set(specs[index][3])
        for i, button in enumerate(buttons):
            button.configure(background="#263D61" if i == index else NAV,
                             foreground="white" if i == index else "#AFBDD0")
    app.notebook.bind("<<NotebookTabChanged>>", selected)
    selected()
    status = tk.Frame(main, background=CANVAS)
    status.pack(fill="x", pady=(10, 0))
    tk.Label(status, textvariable=app.status_var, background=CANVAS, foreground=MUTED,
             font=("Microsoft YaHei UI", 9), anchor="w").pack(side="left")
    tk.Label(status, textvariable=app.dirty_var, background=CANVAS, foreground="#996014",
             font=("Microsoft YaHei UI", 9)).pack(side="right")

    def connection_color(*_args):
        connected = app.connection_var.get().startswith("已连接")
        app.console_connection.configure(background="#E7F5EF" if connected else "#F0F3F7",
                                          foreground="#147353" if connected else MUTED)
    app.connection_var.trace_add("write", connection_color)
    install_scrolling(app)
    return pages


def scroll_surface(parent):
    """Canvas body with working wheel scrolling over child controls."""
    host = ttk.Frame(parent)
    host.pack(fill="both", expand=True)
    canvas = tk.Canvas(host, highlightthickness=0, background="white")
    scroll = ttk.Scrollbar(host, orient="vertical", command=canvas.yview)
    canvas.configure(yscrollcommand=scroll.set)
    scroll.pack(side="right", fill="y")
    canvas.pack(side="left", fill="both", expand=True)
    body = ttk.Frame(canvas, padding=(0, 0, 10, 4))
    window = canvas.create_window((0, 0), window=body, anchor="nw")
    layout = {"width": None, "bounds": None}
    def update_bounds(_event):
        bounds = canvas.bbox("all")
        if bounds != layout["bounds"]:
            layout["bounds"] = bounds
            canvas.configure(scrollregion=bounds)
    def resize_canvas(event):
        if event.width != layout["width"]:
            layout["width"] = event.width
            canvas.itemconfigure(window, width=event.width)
    body.bind("<Configure>", update_bounds)
    canvas.bind("<Configure>", resize_canvas)
    body.console_scroll_canvas = canvas
    return canvas, body


def install_scrolling(app):
    def wheel(event):
        node = event.widget
        if isinstance(node, (tk.Text, ttk.Combobox)):
            return None
        while node is not None:
            canvas = getattr(node, "console_scroll_canvas", None)
            if canvas is not None:
                if canvas.yview() != (0.0, 1.0):
                    canvas.yview_scroll(int(-event.delta / 120), "units")
                return "break"
            node = getattr(node, "master", None)
    app.bind_class("ConsoleScroll", "<MouseWheel>", wheel)


def finish_widgets(app, parent=None):
    """Add wheel routing before class bindings so scrolling cannot nudge a value."""
    root = parent or app
    for widget in root.winfo_children():
        if "ConsoleScroll" not in widget.bindtags():
            widget.bindtags(("ConsoleScroll",) + widget.bindtags())
        finish_widgets(app, widget)
