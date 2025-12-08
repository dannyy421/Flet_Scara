import flet as ft
import math
import json
import os
import serial
import threading
from datetime import datetime
import pygame

class RobotModel:
    def __init__(self):
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.z = 100.0
        self.x = 0.0
        self.y = 0.0
        self.l1 = 9.51  # Longitud del primer segmento (cm)
        self.l2 = 22.31  # Longitud del segundo segmento (cm)
        self.theta = math.radians(122.77133)  # Ángulo constante
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
        # Convertir angle1 (q2) y angle2 (q3) a radianes
        q2_rad = math.radians(self.angle1)
        q3_rad = math.radians(self.angle2)
        
        # Calcular r (radio en el plano XY)
        # De las ecuaciones inversas: D = cos(phi), phi = -(q3 + theta - pi)
        phi = -(q3_rad + self.theta - math.pi)
        D = math.cos(phi)
        
        # De D = (l1^2 + l2^2 - r^2)/(2*l1*l2), resolver para r
        r_squared = self.l1**2 + self.l2**2 - 2*self.l1*self.l2*D
        r = math.sqrt(max(0, r_squared))
        
        # Calcular beta usando la ley de cosenos
        if r > 0:
            A = (self.l1**2 + r**2 - self.l2**2)/(2*self.l1*r)
            A = max(-1, min(1, A))  # Limitar entre -1 y 1
            beta = math.atan2(math.sqrt(1-A**2), A)
            
            # Calcular alpha a partir de q2
            alpha = q2_rad + beta
            
            # Calcular x, y
            self.x = r * math.cos(alpha)
            self.y = r * math.sin(alpha)
        else:
            self.x = 0
            self.y = 0
    
    def inverse_kinematics(self, x, y):
        """Calcular ángulos a partir de X, Y (Inverse Kinematics)"""
        try:
            # Calcular r (radio en el plano XY)
            r = math.sqrt(x**2 + y**2)
            
            if r < abs(self.l1 - self.l2) or r > (self.l1 + self.l2):
                return False  # Posición inalcanzable
            
            # Calcular alpha primero para determinar el signo
            alpha = math.atan2(y, x)
            
            # Cálculo de q3 (angle2 en la interfaz)
            D = ((self.l1**2) + (self.l2**2) - r**2)/(2*self.l1*self.l2)
            D = max(-1, min(1, D))  # Limitar entre -1 y 1
            
            # Cálculo de q2 (angle1 en la interfaz)
            A = ((self.l1**2) + (r**2) - self.l2**2)/(2*self.l1*r)
            A = max(-1, min(1, A))  # Limitar entre -1 y 1
            
            # Calcular beta para determinar q2
            beta = math.atan2(math.sqrt(1-A**2), A)
            q2 = alpha - beta
            
            # Si q2 (angle1/q1 en interfaz) es negativo, cambiar signo en sqrt
            if q2 < 0:
                beta = math.atan2(-math.sqrt(1-A**2), A)
                phi = math.atan2(-math.sqrt(1-D**2), D)
            else:
                beta = math.atan2(math.sqrt(1-A**2), A)
                phi = math.atan2(math.sqrt(1-D**2), D)
            
            # Recalcular q2 con el beta correcto
            q2 = alpha - beta
            
            # Calcular q3 con el phi correcto
            q3 = -(phi + self.theta - math.pi)
            
            # Convertir a grados
            self.angle1 = math.degrees(q2)  # q2 -> angle1 (mostrado como q1 en UI)
            self.angle2 = math.degrees(q3)  # q3 -> angle2 (mostrado como q2 en UI)
            
            return True
        except Exception as e:
            print(f"Error en IK: {e}")
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
    
    x_input = ft.TextField(label="X", width=100, keyboard_type=ft.KeyboardType.NUMBER)
    y_input = ft.TextField(label="Y", width=100, keyboard_type=ft.KeyboardType.NUMBER)
    z_input = ft.TextField(label="Z", width=100, value="100", keyboard_type=ft.KeyboardType.NUMBER)
    
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
        
        # Actualizar también los campos de cinemática inversa
        x_input.value = f"{robot.x:.2f}"
        y_input.value = f"{robot.y:.2f}"
        z_input.value = f"{robot.z:.2f}"
        
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
    
    def bajar_plumon(e):
        """Bajar plumón - establecer Z en 0"""
        z_slider.value = 0
        robot.z = 0
        z_value.value = "0.0"
        z_display.value = "Z: 0.00"
        z_input.value = "0"
        send_serial(f"Z:0.00\n")
        page.snack_bar = ft.SnackBar(ft.Text("Plumón bajado"))
        page.snack_bar.open = True
        page.update()
    
    def subir_plumon(e):
        """Subir plumón - establecer Z en 150"""
        z_slider.value = 150
        robot.z = 150
        z_value.value = "150.0"
        z_display.value = "Z: 150.00"
        z_input.value = "150"
        send_serial(f"Z:150.00\n")
        page.snack_bar = ft.SnackBar(ft.Text("Plumón subido"))
        page.snack_bar.open = True
        page.update()
    
    # JOG buttons
    j1_minus_btn = ft.ElevatedButton("-", width=70, on_click=lambda e: jog_minus(j1_slider, j1_jog_val))
    j1_plus_btn = ft.ElevatedButton("+", width=70, on_click=lambda e: jog_plus(j1_slider, j1_jog_val))
    
    j2_minus_btn = ft.ElevatedButton("-", width=70, on_click=lambda e: jog_minus(j2_slider, j2_jog_val))
    j2_plus_btn = ft.ElevatedButton("+", width=70, on_click=lambda e: jog_plus(j2_slider, j2_jog_val))
    
    bajar_plumon_btn = ft.ElevatedButton("Bajar plumón", width=150, on_click=bajar_plumon)
    subir_plumon_btn = ft.ElevatedButton("Subir plumón", width=150, on_click=subir_plumon)
    
    def bajar_plumon_ik(e):
        """Bajar plumón en cinemática inversa - establecer Z en 0"""
        z_input.value = "0"
        page.update()
    
    def subir_plumon_ik(e):
        """Subir plumón en cinemática inversa - establecer Z en 150"""
        z_input.value = "150"
        page.update()
    
    bajar_plumon_ik_btn = ft.ElevatedButton("Bajar plumón", width=120, on_click=bajar_plumon_ik)
    subir_plumon_ik_btn = ft.ElevatedButton("Subir plumón", width=120, on_click=subir_plumon_ik)
    
    def move_to_position(e):
        """Mover a posición usando IK"""
        try:
            x = float(x_input.value)
            y = float(y_input.value)
            z = float(z_input.value)
            
            # Validar que Z esté dentro del rango del slider
            if z < 0:
                z = 0
            elif z > 150:
                z = 150
            
            if robot.inverse_kinematics(x, y):
                # Validar que los ángulos estén dentro del rango de los sliders
                angle1 = max(-90, min(266, robot.angle1))
                angle2 = max(-150, min(150, robot.angle2))
                
                j1_slider.value = angle1
                j2_slider.value = angle2
                z_slider.value = z
                
                # Actualizar el robot con los valores validados
                robot.angle1 = angle1
                robot.angle2 = angle2
                robot.z = z
                
                # Actualizar todos los campos
                j1_value.value = f"{robot.angle1:.1f}"
                j2_value.value = f"{robot.angle2:.1f}"
                z_value.value = f"{robot.z:.1f}"
                
                # Mantener los valores originales de X, Y, Z que el usuario ingresó
                x_display.value = f"X: {x:.2f}"
                y_display.value = f"Y: {y:.2f}"
                z_display.value = f"Z: {z:.2f}"
                
                # NO actualizar los campos de entrada para mantener los valores originales
                # Los campos de entrada ya tienen los valores correctos
                
                # Enviar comandos al serial
                send_serial(f"J1:{robot.angle1:.2f}\n")
                send_serial(f"J2:{robot.angle2:.2f}\n")
                send_serial(f"Z:{robot.z:.2f}\n")
                
                page.update()
            else:
                page.snack_bar = ft.SnackBar(ft.Text("¡Posición inalcanzable!"))
                page.snack_bar.open = True
                page.update()
        except:
            page.snack_bar = ft.SnackBar(ft.Text("Error en valores ingresados"))
            page.snack_bar.open = True
            page.update()
    
    move_btn = ft.ElevatedButton("MOVE TO POSITION", width=300, on_click=move_to_position)
    
    def move_to_home(e):
        """Mover a posición home (q1=0, q2=0, Z=0)"""
        try:
            # Establecer ángulos directamente en 0
            j1_slider.value = 0
            j2_slider.value = 0
            z_slider.value = 0
            
            # Actualizar el robot
            robot.update_angles(0, 0, 0)
            
            # Actualizar todos los campos
            j1_value.value = "0.0"
            j2_value.value = "0.0"
            z_value.value = "0.0"
            
            # Establecer las coordenadas específicas para home
            x_display.value = "X: -2.91"
            y_display.value = "Y: 18.50"
            z_display.value = "Z: 0.00"
            
            # Actualizar campos de entrada con las coordenadas de home
            x_input.value = "-2.9083"
            y_input.value = "18.4984"
            z_input.value = "0"
            
            # Enviar comandos al serial
            send_serial(f"J1:0.00\n")
            send_serial(f"J2:0.00\n")
            send_serial(f"Z:0.00\n")
            
            page.snack_bar = ft.SnackBar(ft.Text("¡Movido a posición HOME (q1=0, q2=0, Z=0)!"))
            page.snack_bar.open = True
            page.update()
        except Exception as ex:
            page.snack_bar = ft.SnackBar(ft.Text(f"Error al mover a HOME: {str(ex)}"))
            page.snack_bar.open = True
            page.update()
    
    home_btn = ft.ElevatedButton("MOVER A HOME", width=300, on_click=move_to_home, bgcolor=ft.colors.ORANGE_400, color=ft.colors.WHITE)
    
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
    
    # ==================== LAYOUT ====================
    
    forward_kinematics_section = ft.Container(
        content=ft.Column([
            ft.Text("Cinemática directa", size=18, weight="bold", color=ft.colors.BLUE_900),
            ft.Divider(),
            
            # J1
            ft.Row([ft.Text("q1", size=14, width=40, weight="bold"), j1_slider, j1_value], spacing=10),
            ft.Row([ft.Text("", size=14, width=40), j1_minus_btn, j1_jog_val, j1_plus_btn], spacing=5),
            
            # J2
            ft.Row([ft.Text("q2", size=14, width=40, weight="bold"), j2_slider, j2_value], spacing=10),
            ft.Row([ft.Text("", size=14, width=40), j2_minus_btn, j2_jog_val, j2_plus_btn], spacing=5),
            
            # Z
            ft.Row([ft.Text("Z", size=14, width=40, weight="bold"), z_slider, z_value], spacing=10),
            ft.Row([ft.Text("", size=14, width=40), bajar_plumon_btn, subir_plumon_btn], spacing=5),
            
            ft.Divider(),
            ft.Text("Resultado de cinemática directa", size=14, weight="bold"),
            x_display,
            y_display,
            z_display,
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
            
            ft.Row([x_input, y_input, z_input, bajar_plumon_ik_btn, subir_plumon_ik_btn], spacing=10),
            move_btn,
            home_btn,
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
            ], width=650, scroll=ft.ScrollMode.AUTO),
            teach_section,
        ], expand=True),
    )
    
    # Inicializar información
    update_sliders_and_info()
    
    # Enviar valores iniciales a la placa
    send_serial(f"J1:{robot.angle1:.2f}\n")
    send_serial(f"J2:{robot.angle2:.2f}\n")
    
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
