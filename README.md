# ProyectoNoTripuladosP1
Repositorio para el proyecto de vehiculos no tripulados, en este repositorio se encuentra una breve explicación del código de los nodos utilizados para la implementación del algoritmo Follow the Gap en F1Tenth, utilizando ROS2.

## Instrucciones para la ejecución

1. Ingresar a la carpeta y ejecutar

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. Levantar simulador RViz

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

3. Realizar build del programa

```bash
colcon build
```

4. Iniciár el suscriptor para el conteo de vueltas

```bash
ros2 run controllers lap_node
```

5. Iniciár el algoritmo principal follow the gap

```bash
ros2 run controllers nodo_proyecto
```


## Explicación de código utilizado
En este proyecto se realiza una implementación del algoritmo Follow The Gap, este algoritmo consiste en enviar instrucciones al vehículo para su movimiento autónomo, en mi caso, trabajé con la pista de Oschersleben. A continuación se realiza una breve explicación del código implementado.

Mi implementación corresponde a dos nodos que se comunican mediante el tópico /lap_trigger, una vez el nodo de follow the gap envía el mensaje que el carro pasó la "meta", el nodo de conteo de vueltas imprime por pantalla el número de vuelta y el tiempo que le tomó.

1. **Nodo Follow The Gap**:

Este nodo se encuentra en [nodo_proyectoFTG.py](src/controllers/controllers/nodo_proyectoFTG.py).

El nodo llamado FollowGapCarrera permite que el vehículo se movilice de manera autónoma esquivando obstáculos de manera reactiva. Se implementó bajo la siguiente lógica:

- Procesar los datos recibidos por el sensor LiDAR mediante el tópico /scan.
- Identificar el espacio libre más grande disponible.
- Calcular la nueva dirección y velocidad necesarias para movilizarse hasta el centro de ese espacio.

Además, incluye la lógica para comunicarse con otro nodo para el conteo y temporización de vueltas, calculados por odometría mediante el tópico /ego_racecar/odom.

Se inicia con la definición de variables:

En esta sección, se definen las variables necesarias para la conducción, ángulo máximo de giro y factor de suavizado para evitar giros bruscos. Los parámetros del algoritmo para evitar los obstáculos, el alcance del LiDAR y el sesgo para seguir el camino seguro. Y la lógica para el conteo de vueltas, además de las suscripciones y publicadores.

```python
def __init__(self):
        super().__init__('follow_gap_carrera_node')
        self.VELOCIDAD_MAXIMA = 8.2
        self.VELOCIDAD_MINIMA = 3.5
        self.FACTOR_REDUCCION_CURVA = 1.2 
        self.DISTANCIA_MAX_LIDAR = 3.5
        self.RADIO_BURBUJA = 0.25
        self.ANGULO_MAX_DIRECCION = np.radians(20)
        self.PUNTO_LEJANO_BIAS = 0.8
        self.SUAVIZADO_DIRECCION = 0.9 
        self.ultimo_angulo_direccion = 0.0

        self.FINISH_LINE_X = 0.00
        self.FINISH_LINE_Y = 0.00
        self.FINISH_LINE_Y_RANGE = 5.0
        self.MIN_DISTANCE_TO_ARM = 0.2

        self.last_pose = None
        self.can_count_lap = True
        self.race_started = True

        self.lap_publisher = self.create_publisher(String, '/lap_trigger', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        msg_out = String()
        msg_out.data = "Nueva vuelta"
        self.lap_publisher.publish(msg_out)

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
```

Continunado con el procesamiento de los datos del LiDAR,
Se encuentra el punto más cercano al vehículo y se aplica una burbuja de seguridad a los obstáculos más cercanos.
```python
def lidar_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)

        #Limpieza de datos
        ranges[np.isinf(ranges) | np.isnan(ranges)] = self.DISTANCIA_MAX_LIDAR
        ranges = np.clip(ranges, 0.0, self.DISTANCIA_MAX_LIDAR)

        #Punto más cercano
        idx_closest = np.argmin(ranges)
        dist_closest = ranges[idx_closest]

        #Definición de burbuja 
        if dist_closest > 0.1:
            angle_span = 2 * np.arctan2(self.RADIO_BURBUJA, dist_closest)
            bubble_radius_idx = int(angle_span / msg.angle_increment)
        else:
            bubble_radius_idx = 30
        start_idx = max(0, idx_closest - bubble_radius_idx)
        end_idx = min(len(ranges) - 1, idx_closest + bubble_radius_idx)
        ranges[start_idx : end_idx + 1] = 0.0

```
Ahora la búsqueda del mayor gap,
El código se encarga de encontrar la secuencia más grande de mediciones que no se hayan puesto en 0 en el paso anterior

```python
        max_len = 0
        best_start_idx = 0
        best_end_idx = 0
        current_len = 0
        current_start_idx = 0
        for i in range(len(ranges)):
            if ranges[i] > 0.1:
                if current_len == 0:
                    current_start_idx = i
                current_len += 1
            else:
                if current_len > max_len:
                    max_len = current_len
                    best_start_idx = current_start_idx
                    best_end_idx = i - 1
                current_len = 0
        if current_len > max_len:
            max_len = current_len
            best_start_idx = current_start_idx
            best_end_idx = len(ranges) - 1
```

Una vez se ha definido los índices que corresponden al gap más grande, se realizan los cálculos de velocidad, distancia y ángulo para el movimiento del vehículo. Y se envía el mensaje a través de Ackermann.

```python
        drive_msg = AckermannDriveStamped()
        if max_len > 0:
            gap_ranges = ranges[best_start_idx : best_end_idx + 1]
            idx_punto_lejano_relativo = np.argmax(gap_ranges)
            idx_punto_lejano_absoluto = best_start_idx + idx_punto_lejano_relativo
            idx_centro_gap = (best_start_idx + best_end_idx) // 2
            target_idx = int(self.PUNTO_LEJANO_BIAS * idx_punto_lejano_absoluto + (1 - self.PUNTO_LEJANO_BIAS) * idx_centro_gap)
            target_angle = msg.angle_min + target_idx * msg.angle_increment
            
            steering_angle = self.SUAVIZADO_DIRECCION * self.ultimo_angulo_direccion + (1 - self.SUAVIZADO_DIRECCION) * target_angle
            self.ultimo_angulo_direccion = steering_angle
            steering_angle = np.clip(steering_angle, -self.ANGULO_MAX_DIRECCION, self.ANGULO_MAX_DIRECCION)
            
            severidad_curva = abs(target_angle) / self.ANGULO_MAX_DIRECCION
            
            severidad_curva = np.clip(severidad_curva, 0.0, 1.0)
            
            factor_reduccion = (1.0 - severidad_curva) ** self.FACTOR_REDUCCION_CURVA
            speed = self.VELOCIDAD_MINIMA + (self.VELOCIDAD_MAXIMA - self.VELOCIDAD_MINIMA) * factor_reduccion
            
            drive_msg.drive.steering_angle = float(steering_angle)
            drive_msg.drive.speed = float(speed)
        else:
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
            self.ultimo_angulo_direccion = 0.0
        
        self.drive_pub.publish(drive_msg)
```

Para el sistema de conteo de vueltas, se definió un margen para pasar la "meta" en la pista, si la odometría detecta que se ha pasado ese margen, envía el mensaje al tópico /lap_trigger

```python
def odom_callback(self, msg: Odometry):
        current_pose = msg.pose.pose

        if self.last_pose is None:
            self.last_pose = current_pose
            return

        last_x = self.last_pose.position.x
        current_x = current_pose.position.x
        current_y = current_pose.position.y

        crossed_finish_line = (last_x < self.FINISH_LINE_X and current_x >= self.FINISH_LINE_X)
        on_finish_straight = abs(current_y - self.FINISH_LINE_Y) < self.FINISH_LINE_Y_RANGE
        
        if crossed_finish_line and on_finish_straight and self.can_count_lap:
            msg_out = String()
            msg_out.data = "Nueva vuelta"
            self.lap_publisher.publish(msg_out)

            self.can_count_lap = False

        if not self.can_count_lap:
            dist_from_finish = math.sqrt((current_pose.position.x - self.FINISH_LINE_X)**2)
            if dist_from_finish > self.MIN_DISTANCE_TO_ARM:
                self.can_count_lap = True

        self.last_pose = current_pose
```

2. **Nodo Conteo de vueltas**:

El nodo LapTimeLogger se encarga de suscribirse al tópico /lap_trigger para el conteo de vueltas y cálculo de tiempo

Este nodo se encuentra en [controlador_tiempo.py](src/controllers/controllers/controlador_tiempo.py).

La función lap_callback se encarga de la lógica para mostrar por pantalla de la siguiente manera:

```python
def lap_callback(self, msg: String):
        current_time = self.clock.now()

        # Si es la primera vuelta, solo se inicia el cronómetro
        if self.last_trigger_time is None:
            self.last_trigger_time = current_time
            self.get_logger().info("Carrera iniciada")
            return

        # Calcular la duración de la vuelta en segundos
        lap_duration = current_time - self.last_trigger_time
        lap_time_s = lap_duration.nanoseconds / 1e9
        
        self.lap_count += 1

        # Comprobar si es la mejor vuelta hasta ahora
        if lap_time_s < self.best_lap_time_s:
            self.best_lap_time_s = lap_time_s
            best_lap_str = "¡MEJOR VUELTA!"
        else:
            best_lap_str = ""
        
        # Imprimir la información en la consola
        self.get_logger().info(f"Vuelta {self.lap_count} completada | "
                               f"Tiempo: {lap_time_s:.4f}s | {best_lap_str}")
        
        # Condición especial para finalizar después de 10 vueltas
        if self.lap_count == 10:
            self.get_logger().info(f"10 vueltas completadas. "
                                   f"Tiempo mejor vuelta:{self.best_lap_time_s:.2f}s")

        # Actualizar el tiempo de la última vuelta para la siguiente iteración
        self.last_trigger_time = current_time
```

### Video y vuelta más rápida

Aquí podrás encontrar el video donde se evidencia el funcionamiento del código.

[(./img/MiniaturaVideo.png)]([https://www.tu-pagina-externa.com](https://drive.google.com/file/d/1g_mgdxwOQ0wuOUyOpdmmXXkhU_XW1i-_/view?usp=sharing))

Como se puede ver, la velocidad más rápida por vuelta fue de 36.3199s
