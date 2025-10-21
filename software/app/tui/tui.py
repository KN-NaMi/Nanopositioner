import os
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
import rich.box

from NaMi_protocol import NamiConnection, discover_devices

console = Console()

# --- Helper and Display Functions ---
# Clears the terminal screen depending on the OS
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

# Displays a table with found devices
def display_device_table(devices):
    table = Table(title="Found DAC Devices", title_style="bold yellow", box=rich.box.ROUNDED)
    table.add_column("No.", style="white", justify="center")
    table.add_column("Device Name", style="magenta")
    table.add_column("IP Address", style="green3")
    for i, device in enumerate(devices, start=1):
        table.add_row(str(i), device.get('device_id', 'N/A'), device.get('ip_address', 'N/A'))
    console.print(table)
    
# Displays the main help table when the generator is not running
def display_help_table():
    clear_screen()
    table = Table(title="Help and Available Commands", title_style="bold yellow", show_header=True, box=rich.box.ROUNDED)
    table.add_column("#", style="white", width=3)
    table.add_column("Command", style="cyan", width=12)
    table.add_column("Description", style="white")
    
    table.add_row("1", "start", "Starts the configuration and runs the generator.")
    table.add_row("2", "status", "Displays this help screen.")
    table.add_row("3", "clear", "Clears the terminal screen.")
    table.add_row("4", "disconnect", "Disconnects from the current device.")
    console.print(table)
    _display_param_description_table()

# Internal function, displays a table with descriptions of all config parameters
def _display_param_description_table():
    param_table = Table(title="Configuration Parameters Description", title_style="bold yellow", box=rich.box.ROUNDED)
    param_table.add_column("Parameter", style="cyan", width=20)
    param_table.add_column("Description", style="white")
    param_table.add_row("Direction", "Sawtooth signal direction [1=forward, 0=reverse]")
    param_table.add_row(r"Max Voltage \[V]", "Maximum voltage value in a cycle")
    param_table.add_row(r"Rise Time \[ms]", "Duration of the 'stick' phase")
    param_table.add_row(r"Slip Time \[ms]", "Duration of the 'slip' phase")
    param_table.add_row(r"Pause \[ms]", "Pause time between cycles")
    param_table.add_row(r"Curve Shape \[%]", "Curvature of the 'stick' phase [0-100]")
    param_table.add_row("Steps", "Number of cycles to execute [0=continuous]")
    console.print(param_table)

# Displays status and parameters when the generator is running
def display_running_status(active_config):
    config_table = Table(title="Active Configuration", title_style="bold yellow", box=rich.box.ROUNDED)
    config_table.add_column("Parameter", style="cyan", width=25)
    config_table.add_column("Set Value", style="white")

    # Map of parameter names for display in the table
    friendly_names = {
        'dir': 'Direction', 'vmax': r'Max Voltage \[V]', 'rise_time_ms': r'Rise Time \[ms]',
        'slip_time_ms': r'Slip Time \[ms]', 'pause_time_ms': r'Pause \[ms]',
        'curve_percent': r'Curve Shape \[%]', 'steps': 'Steps'
    }

    if active_config:
        for key, value in active_config.items():
            display_value = "Forward" if key == 'dir' and value == 1 else ("Reverse" if key == 'dir' else str(value))
            config_table.add_row(friendly_names.get(key, key), display_value)
    
    _display_param_description_table()
    console.print(config_table)

# Guides the user through the configuration process, asking questions and validating answers
def get_config_from_user():
    panel_text = "Generator Configuration\n\n[dim]Type 'stop' at any time to cancel.[/dim]"
    console.print(Panel(panel_text, style="bright_cyan", title_align="left", box=rich.box.ROUNDED))
    config = {}
    questions = {
        'dir': ("Direction [1=forward, 0=reverse]", int, lambda v: v in [0, 1]),
        'vmax': (r"Max Voltage \[V] (e.g. 2.5)", float, lambda v: 0.0 < v <= 3.3),
        'rise_time_ms': (r"Rise Time \[ms] (e.g. 200)", int, lambda v: v > 0),
        'slip_time_ms': (r"Slip Time \[ms] (e.g. 50)", int, lambda v: v > 0),
        'pause_time_ms': (r"Pause \[ms] (e.g. 100)", int, lambda v: v >= 0),
        'curve_percent': (r"Curve Shape (0-100) \[%]", int, lambda v: 0 <= v <= 100),
        'steps': (r"Number of steps \[0=continuous]", int, lambda v: v >= 0)
    }

    for key, (prompt, type_func, validation_func) in questions.items():
        while True:
            try:
                val_str = console.input(f"[bright_cyan]{prompt}: [/bright_cyan]")
                if val_str.lower() == 'stop':
                    console.print("[yellow]Configuration cancelled.[/yellow]")
                    return None
                
                value = type_func(val_str)
                if validation_func(value):
                    config[key] = 1 if (key == 'dir' and value == 1) else (-1 if key == 'dir' else value)
                    break
                else:
                    console.print("[red]Value is out of the allowed range.[/red]")
            except ValueError:
                console.print(f"[red]Invalid format. Please enter a value of type {type_func.__name__}.[/red]")
                
    console.print("[bold green]Configuration successfully collected.[/bold green]")
    return config

# --- Main Application Loops ---
# Main interaction loop with the connected device
def session_loop(connection, device_info):
    device_id = device_info.get('device_id', 'N/A')
    console.print(Panel(f"✅ Connected to [bold magenta]{device_id}[/bold magenta]", style="green", box=rich.box.ROUNDED))
    is_running = False
    active_config = None 

    # Dynamic menu building depending on the generator's state
    while True:
        menu_table = Table(title="Main Menu", title_style="bold yellow", box=rich.box.ROUNDED)
        menu_table.add_column("#", style="white", width=3)
        menu_table.add_column("Command", style="cyan", width=12)
        menu_table.add_column("Description", style="white")
        
        available_commands = []
        idx = 1
        
        if is_running:
            menu_table.add_row(str(idx), "stop", "Stops the running generator"); idx+=1
            available_commands.append("stop")
            menu_table.add_row(str(idx), "status", "Shows the active configuration"); idx+=1
            available_commands.append("status")
        else:
            menu_table.add_row(str(idx), "start", "Starts the configuration"); idx+=1
            available_commands.append("start")
            menu_table.add_row(str(idx), "status", "Displays the help screen"); idx+=1
            available_commands.append("status")
            menu_table.add_row(str(idx), "disconnect", "Disconnects from the device"); idx+=1
            available_commands.append("disconnect")
        
        console.print(menu_table)
        
        cmd = console.input(f"({device_id}) Enter command: ").lower().strip()
        
        # Check if the entered command is allowed in the current state
        always_available = ['clear']
        if cmd not in available_commands and cmd not in always_available:
            console.print(f"[red]Unknown command '{cmd}' or not available in the current state.[/red]")
            continue

        # Logic for handling individual commands
        if cmd == 'start':
            config = get_config_from_user()
            if config:
                response = connection.send_command({"cmd": "config", "params": config})
                if response and response.get("status") == "OK":
                    is_running = True
                    active_config = config
                    console.print(f"[green]✔️ Success! Generator started.[/green]")
                else:
                    console.print(f"[red]❌ Error! Could not start the generator.[/red]")

        elif cmd == 'stop':
            response = connection.send_command({"cmd": "stop"})
            if response and response.get("status") == "OK":
                is_running = False
                active_config = None
                console.print("[green]✔️ Success! Generator stopped.[/green]")
        
        elif cmd == 'status':
            if is_running:
                display_running_status(active_config)
            else:
                display_help_table()
        
        elif cmd == 'disconnect':
            break

        elif cmd == 'clear':
            clear_screen()
            console.print(Panel(f"Connected to [bold magenta]{device_id}[/bold magenta]", style="green", box=rich.box.ROUNDED))

    return 'continue'

# Main function that runs the application and manages state (disconnected/connected)
def run_app():
    clear_screen()

    while True:
        devices = discover_devices(device_type_filter="dac")
        
        if not devices:
            console.print("[yellow]No DAC devices found.[/yellow]\n")
        else:
            display_device_table(devices)

        console.print("\n[bold]Available options:[/bold]")
        console.print("  1. To connect, enter the device number from the list.")
        console.print("  2. Type [cyan]'refresh'[/cyan] to scan again.")
        console.print("  3. Type [cyan]'exit'[/cyan] to close the application.")
        cmd = console.input("> ").lower().strip()
        
        if cmd == 'exit':
            break
        elif cmd == 'refresh':
            clear_screen()
            continue

        try:
            choice_int = int(cmd)
            if devices and 1 <= choice_int <= len(devices):
                selected_device = devices[choice_int - 1]
                conn = NamiConnection()
                if conn.connect(selected_device['ip_address']):
                    clear_screen()
                    result = session_loop(conn, selected_device)
                    conn.disconnect()
                    
                    if result == 'exit':
                        break
                    else:
                        clear_screen()
                continue
            else:
                console.print("[red]Device number is out of range.[/red]")
        except ValueError:
            console.print(f"[red]Unknown command: '{cmd}'[/red]")
            
    console.print("[bold]Goodbye![/bold]")