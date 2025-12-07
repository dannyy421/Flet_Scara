#include <AccelStepper.h>

// --- 1. Definición de Pines del Shield CNC (SIN CAMBIOS) ---
// Eje X
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5
// Eje Y (Comentado en el original)
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
// Eje Z (Comentado en el original)
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7

// Pin para el Final de Carrera del Eje X (+)
#define ENDSTOP_X_PIN 9
#define ENDSTOP_Y_PIN 10
#define ENDSTOP_Z_PIN 11

// --- 2. Configuración de Movimiento (SIN CAMBIOS) ---
const long MICROSTEPS_PER_REVOLUTION = 800; 
const float TARGET_REVOLUTIONS = 0.5; 
const long TARGET_MICROSTEPS = TARGET_REVOLUTIONS * MICROSTEPS_PER_REVOLUTION; // 400 micropasos
const long MAX_DISTANCE = 1000000; 

// --- 3. Configuración de Velocidad (SIN CAMBIOS) ---
const float MAX_SPEED = 500.0;
const float ACCELERATION = 10000.0; 
const float CRUISE_SPEED = 500.0; 


// 4. Crear los Objetos AccelStepper para cada eje (SIN CAMBIOS)
AccelStepper stepperX(AccelStepper::DRIVER, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

// 5. Variable de estado para controlar la dirección
// MODIFICACIÓN 1: Iniciar la bandera en 'false' (retroceder)
bool movingForward = false; 


void setup() {
    Serial.begin(9600);
    Serial.println("Inicializando Eje X en dirección contraria...");

    // Configurar el pin del final de carrera como entrada
    pinMode(ENDSTOP_Z_PIN, INPUT);
    
    // Configurar motores (SIN CAMBIOS)
    stepperZ.setMaxSpeed(MAX_SPEED);
    stepperZ.setAcceleration(ACCELERATION);
    stepperZ.setSpeed(CRUISE_SPEED); 
    stepperZ.setCurrentPosition(0); 

    // MODIFICACIÓN 2: Mover inicialmente a una posición muy NEGATIVA
    // Esto fuerza al motor a girar en dirección contraria (retroceder).
    stepperZ.moveTo(-MAX_DISTANCE); 
}

void loop() {
    
    bool endstopHit = digitalRead(ENDSTOP_Z_PIN);
    
    // MODIFICACIÓN 3A: Si el motor está RETROCEDIENDO (!movingForward) Y se detecta el final de carrera X+.
    // Esto es necesario si el final de carrera es solo en una posición (+).
    if (!movingForward && endstopHit == LOW) {
        
        Serial.println("¡FINAL DE CARRERA X+ DETECTADO (mientras retrocede)! Cambiando a avance...");
        
        // 1. Detener inmediatamente
        stepperZ.stop();
        
        // 2. Cambiar la bandera de estado para empezar a avanzar
        movingForward = true; // Empieza a avanzar
        
        // 3. Establecer la nueva posición objetivo (un valor grande en positivo para avanzar)
        long forwardPosition = stepperZ.currentPosition() + MAX_DISTANCE;
        
        stepperZ.moveTo(forwardPosition);

    } 
    // MODIFICACIÓN 3B: Si el motor está AVANZANDO (movingForward) y ha completado la distancia de avance (que era 400 pasos).
    // NOTA: Esta lógica asume que el avance debe ser de solo 400 micropasos antes de volver a retroceder.
    // Si quieres que AVANCE hasta otro end-stop, la lógica sería diferente.
    else if (movingForward && stepperZ.distanceToGo() == 0) {
        
        Serial.println("Avance (400 micropasos) completado. Retrocediendo de nuevo...");
        
        // 1. Cambiar la bandera de estado para retroceder
        movingForward = false;
        
        // 2. Establecer la nueva posición objetivo (un valor grande en negativo para retroceder)
        long backPosition = stepperZ.currentPosition() - MAX_DISTANCE;
        
        stepperZ.moveTo(backPosition);
    }

    // Mover los motores
    stepperZ.run();
}