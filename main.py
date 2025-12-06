import flet as ft
import math
import json
import os
import serial
import threading
from datetime import datetime

class RobotModel:
    def __init__(self):
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.z = 100.0
        self.x = 0.0
        self.y = 0.0
        self.l1 = 90.0  # Longitud del primer segmento (mm)
        self.l2 = 220.0  # Longitud del segundo segmento (mm)
        self.speed = 500
        self.acceleration = 500
        self.positions = []
        self.program_running = False
        
    def update_angles(self, a1, a2, z):
        """Actualizar ángulos y calcular posición forward kinematics"""
        self.angle1 = a1
        self.angle2 = a2
        self.z = z
        self.calculate_forward_kinematics()
    
    def calculate_forward_kinematics(self):
        """Calcular posición X, Y a partir de los ángulos (Forward Kinematics)"""
        rad1 = math.radians(self.angle1)
        rad2 = math.radians(self.angle2)
        
        self.x = self.l1 * math.cos(rad1) + self.l2 * math.cos(rad1 + rad2)
        self.y = self.l1 * math.sin(rad1) + self.l2 * math.sin(rad1 + rad2)
    
    def inverse_kinematics(self, x, y):
        """Calcular ángulos a partir de X, Y (Inverse Kinematics)"""
        try:
            # Cálculo de theta2
            cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            
            if abs(cos_theta2) > 1:
                return False  # Posición inalcanzable
            
            theta2 = math.acos(cos_theta2)
            
            # Cálculo de theta1
            k1 = self.l1 + self.l2 * math.cos(theta2)
            k2 = self.l2 * math.sin(theta2)
            theta1 = math.atan2(y, x) - math.atan2(k2, k1)
            
            self.angle1 = math.degrees(theta1)
            self.angle2 = math.degrees(theta2)
            
            return True
        except:
            return False
    
    def save_position(self):
        """Guardar posición actual"""
        pos = {
            'angle1': round(self.angle1, 2),
            'angle2': round(self.angle2, 2),
            'z': round(self.z, 2),
            'x': round(self.x, 2),
            'y': round(self.y, 2),
        }
        self.positions.append(pos)
        return len(self.positions) - 1
    
    def clear_positions(self):
        """Limpiar todas las posiciones guardadas"""
        self.positions = []
    
    def save_to_file(self, filename='robot_program.json'):
        """Guardar programa a archivo"""
        try:
            with open(filename, 'w') as f:
                json.dump({
                    'positions': self.positions,
                    'speed': self.speed,
                    'acceleration': self.acceleration,
                }, f, indent=2)
            return True
        except:
            return False
    
    def load_from_file(self, filename='robot_program.json'):
        """Cargar programa desde archivo"""
        try:
            if os.path.exists(filename):
                with open(filename, 'r') as f:
                    data = json.load(f)
                    self.positions = data.get('positions', [])
                    self.speed = data.get('speed', 500)
                    self.acceleration = data.get('acceleration', 500)
                return True
        except:
            pass
        return False


# ==================== SERIAL COMMUNICATION ====================

ser = None
serial_connected = False

def init_serial(port='COM11', baudrate=115200):
    """Inicializar comunicación serial"""
    global ser, serial_connected
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        serial_connected = True
        print(f"Serial connected to {port} at {baudrate} baud")
        return True
    except Exception as e:
        print(f"Serial connection error: {e}")
        serial_connected = False
        return False

def send_serial(command):
    """Enviar comando al puerto serial"""
    global ser, serial_connected
    if serial_connected and ser and ser.is_open:
        try:
            ser.write(command.encode('utf-8'))
            ser.flush()
            print(f"Sent: {command}")
        except Exception as e:
            print(f"Send error: {e}")

def read_serial_thread(callbacks):
    """Thread para leer datos del puerto serial"""
    global ser, serial_connected
    while serial_connected:
        try:
            if ser and ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    print(f"Received: {data}")
                    for callback in callbacks:
                        callback(data)
        except Exception as e:
            print(f"Read error: {e}")
        
def close_serial():
    """Cerrar puerto serial"""
    global ser, serial_connected
    serial_connected = False
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed")


def main(page: ft.Page):
    page.title = "SCARA Robot Control"
    page.window_width = 1400
    page.window_height = 900
    page.theme_mode = ft.ThemeMode.LIGHT
    
    robot = RobotModel()
    robot.load_from_file()
    
    # ==================== SERIAL COMMUNICATION SETUP ====================
    
    # Intentar conectar al puerto serial
    serial_port = 'COM11'  # Cambiado de COM3 a COM11
    init_serial(serial_port, 115200)
    
    # Variable para estado del serial
    serial_status_text = ft.Text("Serial: Desconectado", size=12, color=ft.colors.RED)
    
    def update_serial_status():
        """Actualizar estado del serial en la UI"""
        if serial_connected:
            serial_status_text.value = "Serial: Conectado (COM)"
            serial_status_text.color = ft.colors.GREEN
        else:
            serial_status_text.value = "Serial: Desconectado"
            serial_status_text.color = ft.colors.RED
        page.update()
    
    # Callback para datos recibidos del serial
    def on_serial_data(data):
        """Procesar datos recibidos del puerto serial"""
        print(f"Processing serial data: {data}")
        
        # Parsear comandos del serial
        if data.startswith("J1:"):
            try:
                val = float(data.split(":")[1])
                j1_slider.value = val
                update_sliders_and_info()
            except:
                pass
        elif data.startswith("J2:"):
            try:
                val = float(data.split(":")[1])
                j2_slider.value = val
                update_sliders_and_info()
            except:
                pass
        elif data.startswith("Z:"):
            try:
                val = float(data.split(":")[1])
                z_slider.value = val
                update_sliders_and_info()
            except:
                pass
        elif data == "SAVE_POS":
            save_position_teach(None)
        elif data == "RUN_PROG":
            run_program(None)
        elif data == "CLEAR_PROG":
            clear_program(None)
    
    # Iniciar thread de lectura serial
    threading.Thread(
        target=read_serial_thread,
        args=([on_serial_data],),
        daemon=True
    ).start()
    
    update_serial_status()
    
    # ==================== FORWARD KINEMATICS SECTION ====================
    
    j1_slider = ft.Slider(min=-90, max=266, value=0, width=300, label="J1")
    j2_slider = ft.Slider(min=-150, max=150, value=0, width=300, label="J2")
    z_slider = ft.Slider(min=0, max=150, value=100, width=300, label="Z")
    
    j1_value = ft.TextField(value="0", read_only=True, width=80)
    j2_value = ft.TextField(value="0", read_only=True, width=80)
    z_value = ft.TextField(value="100", read_only=True, width=80)
    
    j1_jog_val = ft.TextField(value="1", width=50, input_filter=ft.NumbersOnlyInputFilter())
    j2_jog_val = ft.TextField(value="1", width=50, input_filter=ft.NumbersOnlyInputFilter())
    z_jog_val = ft.TextField(value="1", width=50, input_filter=ft.NumbersOnlyInputFilter())
    
    # ==================== INVERSE KINEMATICS SECTION ====================
    
    x_input = ft.TextField(label="X", width=100, input_filter=ft.NumbersOnlyInputFilter())
    y_input = ft.TextField(label="Y", width=100, input_filter=ft.NumbersOnlyInputFilter())
    z_input = ft.TextField(label="Z", width=100, value="100", input_filter=ft.NumbersOnlyInputFilter())
    
    # ==================== TEACH MODE SECTION ====================
    
    positions_list = ft.ListView(expand=True, spacing=5)
    last_position_text = ft.Text("Ultima posición guardada: Ninguna", size=14)
    
    # ==================== INFO DISPLAY ====================
    
    x_display = ft.Text("X: 0.00", size=14)
    y_display = ft.Text("Y: 0.00", size=14)
    z_display = ft.Text("Z: 100.00", size=14)
    
    def update_sliders_and_info():
        """Actualizar información después de cambios en sliders"""
        robot.update_angles(
            float(j1_slider.value),
            float(j2_slider.value),
            float(z_slider.value)
        )
        
        j1_value.value = f"{robot.angle1:.1f}"
        j2_value.value = f"{robot.angle2:.1f}"
        z_value.value = f"{robot.z:.1f}"
        
        x_display.value = f"X: {robot.x:.2f}"
        y_display.value = f"Y: {robot.y:.2f}"
        z_display.value = f"Z: {robot.z:.2f}"
        
        page.update()
    
    def on_j1_change(e):
        update_sliders_and_info()
        send_serial(f"J1:{robot.angle1:.2f}\n")  # Agregar \n para terminar comando
    
    def on_j2_change(e):
        update_sliders_and_info()
        send_serial(f"J2:{robot.angle2:.2f}\n")  # Agregar \n para terminar comando
    
    def on_z_change(e):
        update_sliders_and_info()
        send_serial(f"Z:{robot.z:.2f}\n")  # Mantener para futuro uso
    
    j1_slider.on_change = on_j1_change
    j2_slider.on_change = on_j2_change
    z_slider.on_change = on_z_change
    
    def jog_minus(slider, jog_field):
        try:
            current = float(slider.value)
            jog_amount = float(jog_field.value)
            slider.value = max(slider.min, current - jog_amount)
            update_sliders_and_info()
        except:
            pass
    
    def jog_plus(slider, jog_field):
        try:
            current = float(slider.value)
            jog_amount = float(jog_field.value)
            slider.value = min(slider.max, current + jog_amount)
            update_sliders_and_info()
        except:
            pass
    
    # JOG buttons
    j1_minus_btn = ft.ElevatedButton("-", width=70, on_click=lambda e: jog_minus(j1_slider, j1_jog_val))
    j1_plus_btn = ft.ElevatedButton("+", width=70, on_click=lambda e: jog_plus(j1_slider, j1_jog_val))
    
    j2_minus_btn = ft.ElevatedButton("-", width=70, on_click=lambda e: jog_minus(j2_slider, j2_jog_val))
    j2_plus_btn = ft.ElevatedButton("+", width=70, on_click=lambda e: jog_plus(j2_slider, j2_jog_val))
    
    z_minus_btn = ft.ElevatedButton("-", width=70, on_click=lambda e: jog_minus(z_slider, z_jog_val))
    z_plus_btn = ft.ElevatedButton("+", width=70, on_click=lambda e: jog_plus(z_slider, z_jog_val))
    
    def move_to_position(e):
        """Mover a posición usando IK"""
        try:
            x = float(x_input.value)
            y = float(y_input.value)
            z = float(z_input.value)
            
            if robot.inverse_kinematics(x, y):
                j1_slider.value = robot.angle1
                j2_slider.value = robot.angle2
                z_slider.value = z
                update_sliders_and_info()
            else:
                page.snack_bar = ft.SnackBar(ft.Text("¡Posición inalcanzable!"))
                page.snack_bar.open = True
                page.update()
        except:
            page.snack_bar = ft.SnackBar(ft.Text("Error en valores ingresados"))
            page.snack_bar.open = True
            page.update()
    
    move_btn = ft.ElevatedButton("MOVE TO POSITION", width=300, on_click=move_to_position)
    
    def save_position_teach(e):
        """Guardar posición actual en modo Teach"""
        idx = robot.save_position()
        pos = robot.positions[idx]
        
        # Enviar al serial
        send_serial(f"POS_SAVED:{idx+1},J1={pos['angle1']},J2={pos['angle2']},Z={pos['z']}")
        
        # Agregar a lista visual
        pos_item = ft.Container(
            content=ft.Row([
                ft.Text(f"Pos {idx+1}: J1={pos['angle1']}° J2={pos['angle2']}° Z={pos['z']}", size=12),
                ft.IconButton(
                    ft.icons.DELETE,
                    on_click=lambda e, i=idx: delete_position(i)
                ),
            ]),
            padding=10,
            border=ft.border.all(1, ft.colors.BLUE_200),
            border_radius=5,
            bgcolor=ft.colors.BLUE_50,
        )
        positions_list.controls.append(pos_item)
        
        last_position_text.value = f"Ultima posición guardada: #{idx+1}"
        
        page.snack_bar = ft.SnackBar(ft.Text(f"¡Posición {idx+1} guardada!"))
        page.snack_bar.open = True
        page.update()
    
    def delete_position(idx):
        """Eliminar una posición"""
        if 0 <= idx < len(robot.positions):
            robot.positions.pop(idx)
            positions_list.controls.clear()
            
            for i, pos in enumerate(robot.positions):
                pos_item = ft.Container(
                    content=ft.Row([
                        ft.Text(f"Pos {i+1}: J1={pos['angle1']}° J2={pos['angle2']}° Z={pos['z']}", size=12),
                        ft.IconButton(
                            ft.icons.DELETE,
                            on_click=lambda e, idx=i: delete_position(idx)
                        ),
                    ]),
                    padding=10,
                    border=ft.border.all(1, ft.colors.BLUE_200),
                    border_radius=5,
                    bgcolor=ft.colors.BLUE_50,
                )
                positions_list.controls.append(pos_item)
            
            page.update()
    
    def run_program(e):
        """Ejecutar programa guardado"""
        if len(robot.positions) == 0:
            page.snack_bar = ft.SnackBar(ft.Text("¡No hay posiciones guardadas!"))
            page.snack_bar.open = True
            page.update()
            return
        
        send_serial("RUN_PROGRAM")
        for pos in robot.positions:
            j1_slider.value = pos['angle1']
            j2_slider.value = pos['angle2']
            z_slider.value = pos['z']
            update_sliders_and_info()
            page.update()
    
    def clear_program(e):
        """Limpiar programa"""
        robot.clear_positions()
        positions_list.controls.clear()
        last_position_text.value = "Ultima posición guardada: Ninguna"
        send_serial("CLEAR_PROGRAM")
        page.snack_bar = ft.SnackBar(ft.Text("¡Programa eliminado!"))
        page.snack_bar.open = True
        page.update()
    
    def save_program(e):
        """Guardar programa a archivo"""
        if robot.save_to_file():
            send_serial(f"SAVE_PROGRAM:{len(robot.positions)}")
            page.snack_bar = ft.SnackBar(ft.Text("¡Programa guardado!"))
            page.snack_bar.open = True
            page.update()
    
    save_position_btn = ft.ElevatedButton("Guardar posición", width=200, on_click=save_position_teach)
    run_program_btn = ft.ElevatedButton("Ejecutar programa", width=200, on_click=run_program)
    clear_btn = ft.ElevatedButton("Limpiar", width=100, on_click=clear_program)
    save_program_btn = ft.ElevatedButton("Guardar programa", width=200, on_click=save_program)
    
    # Speed and Acceleration controls with callbacks
    def on_speed_change(e):
        robot.speed = int(speed_slider.value)
        send_serial(f"SPEED:{robot.speed}\n")
        page.update()
    
    def on_accel_change(e):
        robot.acceleration = int(accel_slider.value)
        send_serial(f"ACCEL:{robot.acceleration}\n")
        page.update()
    
    speed_slider = ft.Slider(min=500, max=4000, value=500, width=250, label="Speed", on_change=on_speed_change)
    accel_slider = ft.Slider(min=500, max=4000, value=500, width=250, label="Acceleration", on_change=on_accel_change)
    
    # ==================== LAYOUT ====================
    
    forward_kinematics_section = ft.Container(
        content=ft.Column([
            ft.Text("Cinemática directa", size=18, weight="bold", color=ft.colors.BLUE_900),
            ft.Divider(),
            
            # J1
            ft.Row([ft.Text("J1", size=14, width=40, weight="bold"), j1_slider, j1_value], spacing=10),
            ft.Row([ft.Text("", size=14, width=40), j1_minus_btn, j1_jog_val, j1_plus_btn], spacing=5),
            
            # J2
            ft.Row([ft.Text("J2", size=14, width=40, weight="bold"), j2_slider, j2_value], spacing=10),
            ft.Row([ft.Text("", size=14, width=40), j2_minus_btn, j2_jog_val, j2_plus_btn], spacing=5),
            
            # Z
            ft.Row([ft.Text("Z", size=14, width=40, weight="bold"), z_slider, z_value], spacing=10),
            ft.Row([ft.Text("", size=14, width=40), z_minus_btn, z_jog_val, z_plus_btn], spacing=5),
        ]),
        padding=20,
        border=ft.border.all(2, ft.colors.BLUE_400),
        border_radius=10,
        bgcolor=ft.colors.BLUE_50,
    )
    
    inverse_kinematics_section = ft.Container(
        content=ft.Column([
            ft.Text("Cinemática inversa", size=18, weight="bold", color=ft.colors.GREEN_900),
            ft.Divider(),
            
            ft.Row([x_input, y_input, z_input], spacing=10),
            move_btn,
            
            ft.Text("Resultado de cinemática directa", size=14, weight="bold"),
            x_display,
            y_display,
            z_display,
        ]),
        padding=20,
        border=ft.border.all(2, ft.colors.GREEN_400),
        border_radius=10,
        bgcolor=ft.colors.GREEN_50,
    )
    
    teach_section = ft.Container(
        content=ft.Column([
            ft.Text("Modo Teach - Guardar posiciones", size=18, weight="bold", color=ft.colors.PURPLE_900),
            ft.Divider(),
            
            last_position_text,
            
            ft.Row([save_position_btn, run_program_btn], spacing=10),
            ft.Row([save_program_btn, clear_btn], spacing=10),
            
            ft.Text("Posiciones guardadas:", size=14, weight="bold"),
            ft.Container(
                content=positions_list,
                height=250,
                border=ft.border.all(1, ft.colors.GREY_300),
                border_radius=5,
            ),
        ]),
        padding=20,
        border=ft.border.all(2, ft.colors.PURPLE_400),
        border_radius=10,
        bgcolor=ft.colors.PURPLE_50,
        expand=True,
    )
    
    speed_section = ft.Container(
        content=ft.Column([
            ft.Text("Parámetros de movimiento", size=14, weight="bold"),
            ft.Divider(),
            ft.Row([ft.Text("Velocidad:", width=100), speed_slider]),
            ft.Row([ft.Text("Aceleración:", width=100), accel_slider]),
        ]),
        padding=15,
        border=ft.border.all(1, ft.colors.GREY_400),
        border_radius=8,
    )
    
    # Layout principal
    page.add(
        ft.AppBar(
            title=ft.Row([
                ft.Text("Control del Robot SCARA", size=24, weight="bold"),
                ft.VerticalDivider(),
                serial_status_text,
            ], spacing=20),
            bgcolor=ft.colors.BLUE_900,
            color=ft.colors.WHITE,
        ),
        ft.Row([
            ft.Column([
                forward_kinematics_section,
                inverse_kinematics_section,
                speed_section,
            ], width=650, scroll=ft.ScrollMode.AUTO),
            teach_section,
        ], expand=True),
    )
    
    # Inicializar información
    update_sliders_and_info()
    
    # Enviar valores iniciales a la placa
    send_serial(f"J1:{robot.angle1:.2f}\n")
    send_serial(f"J2:{robot.angle2:.2f}\n")
    send_serial(f"SPEED:{robot.speed}\n")
    send_serial(f"ACCEL:{robot.acceleration}\n")
    
    # Cargar posiciones guardadas si existen
    if robot.positions:
        for i, pos in enumerate(robot.positions):
            pos_item = ft.Container(
                content=ft.Row([
                    ft.Text(f"Pos {i+1}: J1={pos['angle1']}° J2={pos['angle2']}° Z={pos['z']}", size=12),
                    ft.IconButton(
                        ft.icons.DELETE,
                        on_click=lambda e, idx=i: delete_position(idx)
                    ),
                ]),
                padding=10,
                border=ft.border.all(1, ft.colors.BLUE_200),
                border_radius=5,
                bgcolor=ft.colors.BLUE_50,
            )
            positions_list.controls.append(pos_item)


if __name__ == "__main__":
    ft.app(target=main)


if __name__ == "__main__":
    ft.app(target=main)
