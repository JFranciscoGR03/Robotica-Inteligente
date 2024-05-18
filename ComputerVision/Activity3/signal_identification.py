import cv2
from collections import Counter

#Cargar imágenes de referencia de las señales de tráfico
right_arrow_img = cv2.imread('turnright.jpg', cv2.IMREAD_GRAYSCALE)
left_arrow_img = cv2.imread('turnleft.jpg', cv2.IMREAD_GRAYSCALE)
give_way_img = cv2.imread('giveaway.jpg', cv2.IMREAD_GRAYSCALE)
work_in_progress_img = cv2.imread('workinprogress.jpg', cv2.IMREAD_GRAYSCALE)
forward_arrow_img = cv2.imread('straigth.jpg', cv2.IMREAD_GRAYSCALE)
turn_around_arrow_img = cv2.imread('turnaround.jpg', cv2.IMREAD_GRAYSCALE)
stop_img = cv2.imread('stop.jpg', cv2.IMREAD_GRAYSCALE)

#Detector SIFT
sift = cv2.SIFT_create()

#Keypoints y descriptores para las imágenes de referencia
kp1, des1 = sift.detectAndCompute(right_arrow_img, None)
kp2, des2 = sift.detectAndCompute(left_arrow_img, None)
kp3, des3 = sift.detectAndCompute(give_way_img, None)
kp4, des4 = sift.detectAndCompute(work_in_progress_img, None)
kp5, des5 = sift.detectAndCompute(forward_arrow_img, None)
kp6, des6 = sift.detectAndCompute(turn_around_arrow_img, None)
kp7, des7 = sift.detectAndCompute(stop_img, None)

#Inicializar la captura de video
cap = cv2.VideoCapture(0)

#Umbral de coincidencias mínimo para considerar que se ha detectado una señal
MIN_MATCH_COUNT = 20

#Buffer para almacenar los últimos N resultados
buffer_size = 7
results_buffer = []

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    #Conversión a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #Detectar keypoints y descriptores en cámara
    kp_frame, des_frame = sift.detectAndCompute(gray, None)
    
    #Contador de coincidencias para cada señal
    match_counts = {
        'turnright': 0,
        'turnleft': 0,
        'give_way': 0,
        'work_in_progress': 0,
        'straight': 0,
        'turnaround': 0,
        'stop': 0
    }
    
    #Comparar descriptores de cámara con los descriptores de las imágenes de referencia
    for name, (kp_ref, des_ref) in [('turnright', (kp1, des1)), ('turnleft', (kp2, des2)), ('give_way', (kp3, des3)), ('work_in_progress', (kp4, des4)), ('straight', (kp5, des5)), ('turn_around', (kp6, des6)), ('stop', (kp7, des7))]:
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des_ref, des_frame, k=2)
        
        good_matches = []
        for m, n in matches:
            if m.distance < 0.6 * n.distance:  #Umbral para el filtrado de coincidencias
                good_matches.append(m)
        
        #Actualiza el contador de coincidencias para la señal actual
        match_counts[name] = len(good_matches)
    
    #Elección de la señal con la mayor cantidad de coincidencias
    best_match_name = max(match_counts, key=match_counts.get)
    
    #Almacenar el resultado en el buffer
    results_buffer.append(best_match_name)
    if len(results_buffer) > buffer_size:
        results_buffer.pop(0)
    
    #Cálculo de la moda del buffer
    most_common_result = Counter(results_buffer).most_common(1)[0][0]
    
    #Verifica si se han detectado suficientes coincidencias para considerar que se ha encontrado una señal
    if match_counts[best_match_name] >= MIN_MATCH_COUNT:
        #Mostrar el texto de la señal de tráfico detectada
        cv2.putText(frame, most_common_result, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    #Mostrar la imagen con la señal de tráfico detectada
    cv2.imshow('Traffic Signs Detection', frame)
    
    #Cerrar la cámara si se presiona 'x'
    if cv2.waitKey(1) & 0xFF == ord('x'):
        break

#Cerrar todas las ventanas
cap.release()
cv2.destroyAllWindows()