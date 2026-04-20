import math
import random

# ==========================================
# 1. CONFIGURACIÓN DEL SIMULADOR Y VARIABLES
# ==========================================
# Constantes de Estado
ESTADO_ESPERA = 0
ESTADO_ASCENSO = 1
ESTADO_DESCENSO = 2
ESTADO_ATERRIZADO = 3
nombres_estados = ["ESPERA", "ASCENSO", "DESCENSO", "ATERRIZADO"]

estado_actual = ESTADO_ESPERA

# Variables del Filtro Alpha-Beta
h_est = 0.0
v_est = 0.0
alpha = 0.2
beta = 0.01

# Variables de telemetría y seguridad
altitud_maxima = 0.0
velocidad_ascenso_max = 0.0
velocidad_descenso_max = 0.0
contador_apogeo = 0
tiempo_despegue = 0.0
motor_encendido = False
umbral_burnout = 0.5

# Variables del Filtro de Media Móvil (MPU6050)
NUM_MUESTRAS_ACCEL = 5
buffer_accel = [0.0] * NUM_MUESTRAS_ACCEL
indice_accel = 0
accel_z_filtrada = 0.0

# Tiempos de backup
TIEMPO_BACKUP_APOGEO = 15.0
TIEMPO_BACKUP_PRINCIPAL = 60.0 # No se alcanzará en esta sim corta

# ==========================================
# 2. FUNCIÓN PRINCIPAL DE LA COMPUTADORA (Traducción del C)
# ==========================================
def procesar_telemetria(tiempo_actual, delta_t, altitud_cruda, accel_z_cruda):
    global estado_actual, h_est, v_est, altitud_maxima, velocidad_ascenso_max
    global velocidad_descenso_max, contador_apogeo, tiempo_despegue
    global motor_encendido, buffer_accel, indice_accel, accel_z_filtrada

    # === CAPA 2: FILTRADO DE DATOS ===
    # Filtro Media Móvil
    buffer_accel[indice_accel] = accel_z_cruda
    indice_accel = (indice_accel + 1) % NUM_MUESTRAS_ACCEL
    accel_z_filtrada = sum(buffer_accel) / NUM_MUESTRAS_ACCEL

    # === CAPA 1: RECHAZO DE IMPOSIBLES FÍSICOS ===
    v_bruta = (altitud_cruda - h_est) / delta_t
    
    if v_bruta > -343.0 and v_bruta < 343.0:
        # Filtro Alpha-Beta
        h_pred = h_est + (v_est * delta_t)
        v_pred = v_est
        error = altitud_cruda - h_pred
        
        h_est = h_pred + (alpha * error)
        v_est = v_pred + ((beta / delta_t) * error)

        # Actualización de telemetría máxima (solo en vuelo)
        if estado_actual != ESTADO_ESPERA:
            if h_est > altitud_maxima: altitud_maxima = h_est
        if estado_actual == ESTADO_ASCENSO:
            if v_est > velocidad_ascenso_max: velocidad_ascenso_max = v_est
        if estado_actual == ESTADO_DESCENSO:
            if v_est < velocidad_descenso_max: velocidad_descenso_max = v_est

        # === CAPA 3: MÁQUINA DE ESTADOS ===
        if estado_actual == ESTADO_ESPERA:
            if accel_z_filtrada > 2.0 or (h_est > 5.0 and v_est > 15.0):
                estado_actual = ESTADO_ASCENSO
                motor_encendido = True
                tiempo_despegue = tiempo_actual
                print(f"[{tiempo_actual:.2f}s] ¡DESPEGUE DETECTADO! Transición a ASCENSO.")

        elif estado_actual == ESTADO_ASCENSO:
            tiempo_vuelo = tiempo_actual - tiempo_despegue
            
            # Burnout
            if motor_encendido and accel_z_filtrada < umbral_burnout:
                motor_encendido = False
                print(f"[{tiempo_actual:.2f}s] BURNOUT DETECTADO (Fin de motor).")

            # Despliegue Drogue
            if (altitud_maxima - h_est > 3.0) and (v_est < -2.0):
                contador_apogeo += 1
                if contador_apogeo >= 3:
                    estado_actual = ESTADO_DESCENSO
                    print(f"[{tiempo_actual:.2f}s] APOGEO DETECTADO. DISPARANDO DROGUE. Altura: {h_est:.2f}m")
            else:
                contador_apogeo = 0

            # Backup Drogue
            if tiempo_vuelo >= TIEMPO_BACKUP_APOGEO and estado_actual == ESTADO_ASCENSO:
                estado_actual = ESTADO_DESCENSO
                print(f"[{tiempo_actual:.2f}s] BACKUP TIMER APOGEO. DISPARANDO DROGUE FORZADO.")

        elif estado_actual == ESTADO_DESCENSO:
            tiempo_vuelo = tiempo_actual - tiempo_despegue

            # Despliegue Principal a los 250m
            if h_est <= 250.0 and altitud_maxima > 250.0:
                estado_actual = ESTADO_ATERRIZADO
                print(f"[{tiempo_actual:.2f}s] ALTITUD CRÍTICA. DISPARANDO PRINCIPAL. Altura: {h_est:.2f}m")
            
            # Detección de suelo real (velocidad casi nula a baja altura)
            if h_est < 5.0 and abs(v_est) < 1.0 and altitud_maxima > 10.0:
                estado_actual = ESTADO_ATERRIZADO
                print(f"[{tiempo_actual:.2f}s] ATERRIZAJE DETECTADO.")

# ==========================================
# 3. GENERADOR DE VUELO Y BUCLE PRINCIPAL
# ==========================================
print("--- INICIANDO SIMULACIÓN DE VUELO ---")
delta_t = 0.05 # 20 Hz
tiempo_actual = 0.0

# Perfil de vuelo ideal simulado
altitud_real = 0.0
velocidad_real = 0.0

for paso in range(1600): # Simular 80 segundos
    tiempo_actual += delta_t
    
    # 1. Simulación de la física del cohete
    if tiempo_actual < 2.0:
        # En rampa
        accel_real = 0.0
    elif tiempo_actual < 3.5:
        # Motor encendido (T=2 a T=4)
        accel_real = 5.0 # 5G de aceleración
    elif altitud_real > 0:
        # Vuelo inercial y caída libre
        accel_real = -1.0 # Gravedad restando velocidad
    else:
        # En el piso
        accel_real = 0.0
        velocidad_real = 0.0
        altitud_real = 0.0

    # Si hay paracaídas abierto, frenamos la caída
    if estado_actual == ESTADO_DESCENSO and velocidad_real < -15.0:
        accel_real = 2.0 # Freno aerodinámico del drogue

    velocidad_real += accel_real * 9.81 * delta_t
    altitud_real += velocidad_real * delta_t
    if altitud_real < 0: altitud_real = 0

    # 2. Generar lecturas de "Sensores" (con ruido)
    ruido_barometro = random.uniform(-1.5, 1.5)
    ruido_accel = random.uniform(-0.2, 0.2)
    
    alt_sensor = altitud_real + ruido_barometro
    accel_sensor = accel_real + ruido_accel

    # ¡INYECCIÓN DE ERROR MASIVO (GLITCH)!
    if 12.0 < tiempo_actual < 12.1:
        alt_sensor += 500.0 # Falso pico de presión supersónica

    # 3. Alimentar a la computadora de vuelo
    procesar_telemetria(tiempo_actual, delta_t, alt_sensor, accel_sensor)

# ==========================================
# 4. REPORTE POST-VUELO
# ==========================================
print("\n--- REPORTE DE VUELO GENERADO ---")
print(f"Altitud Máxima (Apogeo): {altitud_maxima:.2f} m")
print(f"Velocidad de Ascenso Max: {velocidad_ascenso_max:.2f} m/s")
print(f"Velocidad de Descenso Max: {velocidad_descenso_max:.2f} m/s")
print("Estado Final de la Máquina:", nombres_estados[estado_actual])