import math
import random
import matplotlib.pyplot as plt

# ==========================================
# 1. CONFIGURACIÓN DEL SISTEMA Y ESTADOS
# ==========================================
ESTADO_ESPERA = 0
ESTADO_ASCENSO = 1
ESTADO_DESCENSO_DROGUE = 2
ESTADO_DESCENSO_PRINCIPAL = 3
ESTADO_ATERRIZADO = 4

nombres_estados = ["ESPERA (En Rampa)", "ASCENSO", "DESCENSO (Drogue)", "DESCENSO (Principal)", "ATERRIZADO"]

estado_actual = ESTADO_ESPERA
h_est = 0.0 # Altitud estimada (AGL)
v_est = 0.0 # Velocidad estimada
alpha = 0.2
beta = 0.01

altitud_maxima = 0.0
velocidad_ascenso_max = 0.0
velocidad_descenso_max = 0.0
contador_apogeo = 0
tiempo_despegue = 0.0
motor_encendido = False
umbral_burnout = 0.5

NUM_MUESTRAS_ACCEL = 5
buffer_accel = [0.0] * NUM_MUESTRAS_ACCEL
indice_accel = 0
accel_z_filtrada = 0.0

rechazos_consecutivos = 0 # El "salvavidas" anti-lockout

# Historial para el gráfico final
historial_tiempo = []
historial_alt_cruda = []
historial_alt_filtrada = []
historial_rechazados_x = []
historial_rechazados_y = []

# ==========================================
# 2. EL CEREBRO DEL COHETE (Filtro y Lógica)
# ==========================================
def procesar_telemetria(tiempo_actual, delta_t, altitud_cruda, accel_z_cruda):
    global estado_actual, h_est, v_est, altitud_maxima, velocidad_ascenso_max
    global velocidad_descenso_max, contador_apogeo, tiempo_despegue
    global motor_encendido, buffer_accel, indice_accel, accel_z_filtrada
    global rechazos_consecutivos

    # 2.1 Filtro simple para el acelerómetro
    buffer_accel[indice_accel] = accel_z_cruda
    indice_accel = (indice_accel + 1) % NUM_MUESTRAS_ACCEL
    accel_z_filtrada = sum(buffer_accel) / NUM_MUESTRAS_ACCEL

    # 2.2 Validación de la medición del barómetro
    v_bruta = (altitud_cruda - h_est) / delta_t
    dato_aceptado = False

    # Si la velocidad física es posible (< Mach 1) O si venimos ciegos hace mucho tiempo (Anti-Lockout)
    if (v_bruta > -343.0 and v_bruta < 343.0) or rechazos_consecutivos > 10:
        
        if rechazos_consecutivos > 10:
            print(f"[{tiempo_actual:.2f}s] ¡ALERTA! Demasiados errores seguidos. Resincronizando computadora con sensor...")
            h_est = altitud_cruda
            v_est = 0.0

        rechazos_consecutivos = 0
        dato_aceptado = True
        
        # Filtro Alpha-Beta (Suavizado de la curva)
        h_pred = h_est + (v_est * delta_t)
        v_pred = v_est
        error = altitud_cruda - h_pred
        
        h_est = h_pred + (alpha * error)
        v_est = v_pred + ((beta / delta_t) * error)

        # Registro de récords
        if estado_actual != ESTADO_ESPERA:
            if h_est > altitud_maxima: altitud_maxima = h_est
        if estado_actual == ESTADO_ASCENSO:
            if v_est > velocidad_ascenso_max: velocidad_ascenso_max = v_est
        if estado_actual in [ESTADO_DESCENSO_DROGUE, ESTADO_DESCENSO_PRINCIPAL]:
            if v_est < velocidad_descenso_max: velocidad_descenso_max = v_est

        # 2.3 Máquina de Estados (Control de Vuelo y Paracaídas)
        if estado_actual == ESTADO_ESPERA:
            if accel_z_filtrada > 2.0 or (h_est > 5.0 and v_est > 15.0):
                estado_actual = ESTADO_ASCENSO
                motor_encendido = True
                tiempo_despegue = tiempo_actual
                print(f"[{tiempo_actual:.2f}s] ¡DESPEGUE DETECTADO! Transición a ASCENSO.")

        elif estado_actual == ESTADO_ASCENSO:
            if motor_encendido and accel_z_filtrada < umbral_burnout:
                motor_encendido = False
                print(f"[{tiempo_actual:.2f}s] BURNOUT. Fin de empuje del motor.")

            # Detectar Apogeo: Ya no subimos (estamos 2m debajo del maximo) y la velocidad es negativa
            if (altitud_maxima - h_est > 2.0) and (v_est < -1.0):
                contador_apogeo += 1
                if contador_apogeo >= 3: # Confirmamos con 3 lecturas seguidas
                    estado_actual = ESTADO_DESCENSO_DROGUE
                    print(f"[{tiempo_actual:.2f}s] *** APOGEO CONFIRMADO *** Altura: {h_est:.2f}m")
                    print(f"[{tiempo_actual:.2f}s] ---> DISPARANDO PARACAÍDAS DROGUE")
            else:
                contador_apogeo = 0

        elif estado_actual == ESTADO_DESCENSO_DROGUE:
            # Detectar cruce de 250m hacia abajo
            if h_est <= 250.0 and altitud_maxima > 250.0:
                estado_actual = ESTADO_DESCENSO_PRINCIPAL
                print(f"[{tiempo_actual:.2f}s] *** ALTITUD CRÍTICA ALCANZADA *** Altura: {h_est:.2f}m")
                print(f"[{tiempo_actual:.2f}s] ---> DISPARANDO PARACAÍDAS PRINCIPAL")
        
        elif estado_actual == ESTADO_DESCENSO_PRINCIPAL:
            # Detectar el suelo (velocidad casi nula y altura cerca de 0)
            if h_est < 5.0 and abs(v_est) < 2.0 and altitud_maxima > 10.0:
                estado_actual = ESTADO_ATERRIZADO
                print(f"[{tiempo_actual:.2f}s] ATERRIZAJE DETECTADO. Fin de la misión.")
    else:
        # El dato es imposible y el contador es bajo: lo rechazamos.
        rechazos_consecutivos += 1
        dato_aceptado = False

    return dato_aceptado

# ==========================================
# 3. MOTOR FÍSICO Y ENTORNO DE PRUEBA
# ==========================================
print("--- INICIANDO SIMULACIÓN DE VUELO ---")
delta_t = 0.05 
tiempo_actual = 0.0
altitud_real = 0.0
velocidad_real = 0.0

for paso in range(1600): # Simular 80 segundos
    tiempo_actual += delta_t
    
    # Perfil del Cohete Real
    if tiempo_actual < 2.0: 
        accel_real = 0.0
    elif tiempo_actual < 5.1: 
        accel_real = 3.0 # Motor empujando a ~3G (Garantiza cruzar los 450m)
    elif altitud_real > 0: 
        accel_real = -1.0 # Caída libre (-1G)
    else:
        accel_real = 0.0
        velocidad_real = 0.0
        altitud_real = 0.0

    # Resistencia aerodinámica de los paracaídas
    if estado_actual == ESTADO_DESCENSO_DROGUE and velocidad_real < -20.0:
        accel_real = 2.0 # El drogue estabiliza la caída a -20 m/s
    elif estado_actual == ESTADO_DESCENSO_PRINCIPAL and velocidad_real < -5.0:
        accel_real = 10.0 # El principal frena brusco hasta caer a -5 m/s

    velocidad_real += accel_real * 9.81 * delta_t
    altitud_real += velocidad_real * delta_t
    if altitud_real < 0: altitud_real = 0

    # Sensor con ruido normal
    ruido_barometro = random.uniform(-2.5, 2.5) 
    alt_sensor = altitud_real + ruido_barometro
    accel_sensor = accel_real + random.uniform(-0.2, 0.2)

    # ====================================================
    # INYECCIÓN DE ERRORES CATASTRÓFICOS
    # ====================================================
    if 8.0 < tiempo_actual < 8.1:
        alt_sensor -= 400.0 # Falla profunda de sensor
    elif 18.0 < tiempo_actual < 19.5:
        alt_sensor += random.uniform(-150, 150) # Tormenta de ruido prolongada (Prueba el Anti-Lockout)
    elif 30.0 < tiempo_actual < 30.2:
        alt_sensor += 500.0 # Pico supersónico falso

    # Procesamos la telemetría con nuestro cerebro
    aceptado = procesar_telemetria(tiempo_actual, delta_t, alt_sensor, accel_sensor)

    # Guardamos datos para graficar
    historial_tiempo.append(tiempo_actual)
    historial_alt_cruda.append(alt_sensor)
    historial_alt_filtrada.append(h_est)
    
    if not aceptado:
        historial_rechazados_x.append(tiempo_actual)
        historial_rechazados_y.append(alt_sensor)

# ==========================================
# 4. REPORTE Y GRÁFICO
# ==========================================
print("\n--- REPORTE FINAL ---")
print(f"Altitud Máxima (Apogeo): {altitud_maxima:.2f} m")
print(f"Velocidad Max Ascenso: {velocidad_ascenso_max:.2f} m/s")
print(f"Velocidad Max Descenso: {velocidad_descenso_max:.2f} m/s")
print(f"Estado Final de la Computadora: {nombres_estados[estado_actual]}")

plt.figure(figsize=(12, 6))
plt.plot(historial_tiempo, historial_alt_cruda, label='Barómetro (Lectura Cruda)', color='red', alpha=0.3)
plt.plot(historial_tiempo, historial_alt_filtrada, label='Computadora (Altitud Estimada)', color='blue', linewidth=2.5)
plt.scatter(historial_rechazados_x, historial_rechazados_y, color='black', marker='X', s=40, label='Glitches Bloqueados')

# Líneas de referencia
plt.axhline(y=altitud_maxima, color='green', linestyle='--', alpha=0.5, label='Línea de Apogeo')
plt.axhline(y=250, color='purple', linestyle='--', alpha=0.5, label='Línea de 250m (Principal)')

plt.title('Simulación de Vuelo V4 - Filtro Alpha-Beta y Dual Deployment')
plt.xlabel('Tiempo de Vuelo (segundos)')
plt.ylabel('Altitud sobre el Nivel del Suelo [AGL] (metros)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()