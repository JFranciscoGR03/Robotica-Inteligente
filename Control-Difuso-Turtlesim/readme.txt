Alumnos: 
- Juan Francisco García Rodríguez
- Jennifer Avendaño Sánchez
- Juan Antonio Mancera Velasco

Implementar un control difuso de posición (controlando velocidades angular y lineal) de un robot basado el simulador de TurtleSim.

Tarea 1. Hacer que Turtle2 desde el centro de la pantalla gire hacia Turtle1 en todo momento. (Se puede desactivar la velocidad lineal).

Tarea 2. Harer que Turtle2 vaya al mismo punto (seguimiento) a donde quiera que vaya Turtle1.
Turtle1 se mantendrá trazando un rombo a lo largo del espacio de trabajo.

Recomendaciones:

    Utilizar conjuntos difusos curvos como entrada (cambiar los del ejemplo) y lineales como salida.
    Usar como variables de entrada 1. La distancia a la otra tortuga,  2. Qué tan desviado se encuentra el eje x (frontal de Turtle1) respecto a la recta que conecta con Turtle2.
    Usar como variables de salida 1. La velocidad lineal, 2. la velocidad angular

MUY IMPORTANTE: Se calcula la superficie de control una sola vez. Se guarda la matriz en un archivo. Después en ROS, se importa la matriz como lista o arreglo y se procede a seleccionar los valores de velocidades (lineales y/o angulares) según la coordenada que corresponda.
