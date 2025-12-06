# SCARA Robot Desktop - Flet Edition

Simulador de un robot SCARA (Selective Compliance Articulated Robot Arm) construido con Flet.

## Requisitos

- Python 3.8+
- Flet
- NumPy
- Matplotlib

## Instalación

1. Instala las dependencias:
```bash
pip install -r requirements.txt
```

## Ejecución

```bash
python main.py
```

## Características

- **Visualización en tiempo real** del robot SCARA
- **Control de ángulos** mediante sliders
- **Cálculo de cinemática directa** (Forward Kinematics)
- **Visualización de posición (X, Y)** del efector final
- **Animación de secuencia** de movimientos
- **Reloj en tiempo real**
- **Interfaz intuitiva** con Flet

## Controles

- **Ángulo 1**: Controla la rotación del primer segmento
- **Ángulo 2**: Controla la rotación del segundo segmento
- **Animar Secuencia**: Ejecuta una secuencia de movimiento automática
- **Resetear**: Vuelve el robot a la posición inicial

## Estructura del Código

- `main.py`: Aplicación principal con la interfaz y lógica del robot
- `requirements.txt`: Dependencias del proyecto
- `README.md`: Este archivo

## Notas

Este es un simulador educativo del robot SCARA que demuestra:
- Cinemática directa
- Visualización de movimientos robóticos
- Programación con Flet para aplicaciones desktop