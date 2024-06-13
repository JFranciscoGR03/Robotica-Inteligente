import numpy as np
import skfuzzy as sk
from matplotlib import pyplot as plt
import yaml

def encontrar_indice(valor, start, step, length):
    indice = round((valor - start) / step)
    if indice < 0:
        indice = 0
    elif indice >= length:
        indice = length - 1
        
    return indice

def save_to_yaml(filename, x, y, z_l, z_a):
    data = {
        'robot_controller': {
            'ros__parameters': {
                'X': x.tolist(),
                'Y': y.tolist(),
                'Z_l': z_l.tolist(),
                'Z_a': z_a.tolist()
            }
        }
    }
    with open(filename, 'w') as file:
        yaml.dump(data, file, default_flow_style=None)

dist_ab = np.arange(0.5, 7.05, 0.05)  # Distancia entre tortugas
vel_l = np.arange(0.0, 4.1, 0.04)  # Velocidad de la tortuga 2
desv_ab = np.arange(-2.5, 2.55, 0.05) # Diferencia de ángulo entre las dos tortugas
vel_a = np.arange(-3.5, 3.57, 0.07)

def fis(da, desva):
    # Conjuntos de la variable distancia
    dist_ab_short = sk.trapmf(dist_ab, [0.5, 0.5, 1.5, 2.5])
    dist_ab_mod = sk.trapmf(dist_ab, [2.0, 3.25, 4.50, 5.75])
    dist_ab_long = sk.trapmf(dist_ab, [5.25, 6.0, 7.05, 7.05])

    '''plt.figure(1)
    plt.subplot(221)
    plt.plot(dist_ab, dist_ab_short, label="corta")
    plt.plot(dist_ab, dist_ab_mod, label="moderada")
    plt.plot(dist_ab, dist_ab_long, label="larga")
    plt.title("Distancia entre tortugas")
    plt.xlabel("d [m]")
    plt.ylabel(r'$\mu$')
    plt.legend()'''

    # Conjuntos difusos para desv_ang
    desv_ab_left = sk.trapmf(desv_ab, [-2.5, -2.5, -1.5, -0.5])
    desv_ab_center = sk.trapmf(desv_ab, [-1.0, -0.5, 0.5, 1.0])
    desv_ab_right = sk.trapmf(desv_ab, [0.5, 1.5, 2.55, 2.55])

    '''plt.subplot(222)
    plt.plot(desv_ab, desv_ab_left, label="izquierda")
    plt.plot(desv_ab, desv_ab_center, label="centro")
    plt.plot(desv_ab, desv_ab_right, label="derecha")
    plt.title("Desviación entre tortugas")
    plt.xlabel(r'$\theta$' " [rad]")
    plt.ylabel(r'$\mu$')
    plt.legend()'''

    # Conjuntos de la variable velocidad lineal
    vel_l_low = sk.gbellmf(vel_l, 0.45, 5, 0.0)
    vel_l_mlow = sk.gbellmf(vel_l, 0.45, 5, 1.0)
    vel_l_med = sk.gbellmf(vel_l, 0.45, 5, 2.0)
    vel_l_mhigh = sk.gbellmf(vel_l, 0.45, 5, 3.0)
    vel_l_high = sk.gbellmf(vel_l, 0.45, 5, 4.0)

    '''plt.subplot(223)
    plt.plot(vel_l, vel_l_low, label="baja")
    plt.plot(vel_l, vel_l_mlow, label="media-baja")
    plt.plot(vel_l, vel_l_med, label="media")
    plt.plot(vel_l, vel_l_mhigh, label="media-alta")
    plt.plot(vel_l, vel_l_high, label="alta")
    plt.title("Velocidad lineal")
    plt.xlabel("v [m/s]")
    plt.ylabel(r'$\mu$')
    plt.legend()'''

    # Conjuntos de la variable velocidad angular
    high_cw = sk.gbellmf(vel_a, 0.8, 5, -3.5)
    medium_cw= sk.gbellmf(vel_a, 0.8, 5, -1.75)
    low = sk.gbellmf(vel_a, 0.8, 5, 0.0)
    medium_ccw = sk.gbellmf(vel_a, 0.8, 5, 1.75)
    high_ccw = sk.gbellmf(vel_a, 0.8, 5, 3.5)

    '''plt.subplot(224)
    plt.plot(vel_a, high_cw, label="cw alta")
    plt.plot(vel_a, medium_cw, label="cw media")
    plt.plot(vel_a, low, label="baja")
    plt.plot(vel_a, medium_ccw, label="ccw media")
    plt.plot(vel_a, high_ccw, label="ccw alta")
    plt.title("Velocidad angular")
    plt.xlabel(r'$\theta$' " [rad/s]")
    plt.ylabel(r'$\mu$')
    plt.legend()'''

    plt.show()
        
    da_idx = encontrar_indice(da, 0.5, 0.05, len(dist_ab))
    desva_idx = encontrar_indice(desva, -2.5, 0.05, len(desv_ab))

    # Reglas de inferencia
    R1 = min(dist_ab_short[da_idx], desv_ab_left[desva_idx])
    R2 = min(dist_ab_short[da_idx], desv_ab_center[desva_idx])
    R3 = min(dist_ab_short[da_idx], desv_ab_right[desva_idx])

    R4 = min(dist_ab_mod[da_idx], desv_ab_left[desva_idx])
    R5 = min(dist_ab_mod[da_idx], desv_ab_center[desva_idx])
    R6 = min(dist_ab_mod[da_idx], desv_ab_right[desva_idx])

    R7 = min(dist_ab_long[da_idx], desv_ab_left[desva_idx])
    R8 = min(dist_ab_long[da_idx], desv_ab_center[desva_idx])
    R9 = min(dist_ab_long[da_idx], desv_ab_right[desva_idx])

    # Salidas difusas para velocidad lineal
    max_vel_l_low = max(R2, 0)
    max_vel_l_mlow = max(R1, R3)
    max_vel_l_med = max(R4, R5, R6)
    max_vel_l_mhigh = max(R7, R9)
    max_vel_l_high = max(R8, 0)

    # Salidas difusas para velocidad angular
    max_high_cw = max(R4, R7)
    max_medium_cw = max(R1, 0)
    max_low = max(R2, R5, R8)
    max_medium_ccw = max(R6, R9)
    max_high_ccw = max(R3, 0)

    acc_vel_l = []
    acc_vel_a = []

    for i in range(len(vel_l)):
        vel_l_low[i] = min(vel_l_low[i], max_vel_l_low)
        vel_l_mlow[i] = min(vel_l_mlow[i], max_vel_l_mlow)
        vel_l_med[i] = min(vel_l_med[i], max_vel_l_med)
        vel_l_mhigh[i] = min(vel_l_mhigh[i], max_vel_l_mhigh)
        vel_l_high[i] = min(vel_l_high[i], max_vel_l_high)

        acc_vel_l.append(max(vel_l_low[i], vel_l_mlow[i], vel_l_med[i], vel_l_mhigh[i], vel_l_high[i]))
        
    for i in range(len(vel_a)):
        high_cw[i] = min(high_cw[i], max_high_cw)
        medium_cw[i] = min(medium_cw[i], max_medium_cw)
        low[i] = min(low[i], max_low)
        medium_ccw[i] = min(medium_ccw[i], max_medium_ccw)
        high_ccw[i] = min(high_ccw[i], max_high_ccw)

        acc_vel_a.append(max(high_cw[i], medium_cw[i], low[i], medium_ccw[i], high_ccw[i]))

    num_l = 0.0
    den_l = 0.0
    for i in range(len(vel_l)):
        num_l = num_l + vel_l[i] * acc_vel_l[i]
        den_l = den_l + acc_vel_l[i]
        
    defuzz_l = round(num_l / den_l, 2)
        
    num_a = 0.0
    den_a = 0.0
    for i in range(len(vel_a)):
        num_a = num_a + vel_a[i] * acc_vel_a[i]
        den_a = den_a + acc_vel_a[i]

    defuzz_a = round(num_a / den_a, 2)
    
    return defuzz_l, defuzz_a

############################################## Superficies de control ##############################################
#Entrada
s = 131,101
ctrl_surf_l = np.zeros(s)
ctrl_surf_a = np.zeros(s)

for j in range(len(desv_ab)):
    for i in range(len(dist_ab)):
        (ctrl_surf_l[i,j], ctrl_surf_a[i,j]) = fis(dist_ab[i], desv_ab[j])
        
X, Y = np.meshgrid(desv_ab, dist_ab)

# Velocidad lineal
Z_l = np.array(ctrl_surf_l)
plt.figure(1)

ax = plt.axes(projection = '3d')
ax.plot_surface(X, Y, Z_l, rstride=1, cstride=1, cmap='inferno', edgecolor='none')

ax.set_xlabel("Diferencia de ángulo [" r'$\theta$' "]")
ax.set_ylabel("Distancia [m]")
ax.set_zlabel("Velocidad lineal [m/s]")
plt.title("Superficie de control para velocidad lineal")


# Velocidad angular
Z_a = np.array(ctrl_surf_a)
plt.figure(2)

ax = plt.axes(projection = '3d')
ax.plot_surface(X, Y, Z_a, rstride=1, cstride=1, cmap='inferno', edgecolor='none')

ax.set_xlabel("Diferencia de ángulo [" r'$\theta$' "]")
ax.set_ylabel("Distancia [m]")
ax.set_zlabel("Velocidad angular [rad/s]")
plt.title("Superficie de control para velocidad angular")

plt.show()


################################################### Generar YAML ###################################################
X, Y = np.meshgrid(dist_ab, desv_ab)
Z_l = np.zeros(X.shape)
Z_a = np.zeros(X.shape)

for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        Z_l[i, j], Z_a[i, j] = fis(X[i, j], Y[i, j])

save_to_yaml('fis_control_surface.yaml', dist_ab, desv_ab, Z_l, Z_a)