"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Lidar, Motor, DistanceSensor # Importa as classes necessárias

# Cria a instância do robô
robot = Robot()

# Obtém o tempo de passo do mundo (simulador)
timestep = int(robot.getBasicTimeStep())

# --- Configuração dos Dispositivos do Robô ---

# 1. LIDAR
# Verifique o nome do nó do seu LIDAR no Webots (na árvore de nós do robô).
# Geralmente é algo como 'lidar' ou 'Sick LMS 291'.
try:
    lidar = robot.getDevice('lidar') # Ajuste 'lidar' para o nome correto do seu LIDAR
    lidar.enable(timestep)
    # Se precisar do campo de profundidade do LIDAR para a RNA:
    lidar.enablePointCloud()
except AttributeError:
    print("LIDAR não encontrado. Verifique o nome do dispositivo.")
    lidar = None # Define como None para evitar erros se não for encontrado

# 2. Motores das Rodas
# Os robôs Pioneer geralmente têm 'left wheel motor' e 'right wheel motor'.
try:
    back_left_motor = robot.getDevice('back left wheel')
    front_left_motor = robot.getDevice('front left wheel')
    back_right_motor = robot.getDevice('back right wheel')
    front_right_motor = robot.getDevice('front right wheel')
    

    # Configura os motores para controle de velocidade (posição infinita)
    back_left_motor.setPosition(float('inf'))
    front_left_motor.setPosition(float('inf'))
    back_right_motor.setPosition(float('inf'))
    front_right_motor.setPosition(float('inf'))

    # Define uma velocidade inicial zero
    back_left_motor.setVelocity(0.0)
    front_left_motor.setVelocity(0.0)
    back_right_motor.setVelocity(0.0)
    front_right_motor.setVelocity(0.0)   
    
except AttributeError:
    print("Motores das rodas não encontrados. Verifique os nomes dos dispositivos.")
    back_left_motor = None
    front_left_motor = None
    back_right_motor = None
    front_right_motor = None

# 3. Sensores de Distância (Ex: Sonares)
# Os robôs Pioneer frequentemente possuem vários sensores de distância (sonares ou IR).
# Você precisará listar todos eles e habilitá-los.
# Exemplo para 8 sensores típicos de um Pioneer:
distance_sensors = []
for i in range(15): # Ajuste o número de sensores que seu robô possui
    sensor_name = f'SO_{i} DistanceSensor' # Nome padrão para sonares em alguns Pioneers (ex: so0, so1, ...)
    try:
        ds = robot.getDevice(sensor_name)
        if ds:
            ds.enable(timestep)
            distance_sensors.append(ds)
    except AttributeError:
        print(f"Sensor de distância '{sensor_name}' não encontrado.")
        pass # Ignora se um sensor específico não for encontrado, útil para diferentes modelos

# Adicione a câmera aqui também, conforme o projeto exige
try:
    camera = robot.getDevice('camera') # Ajuste 'camera' para o nome correto da sua câmera
    camera.enable(timestep)
except AttributeError:
    print("Câmera não encontrada. Verifique o nome do dispositivo.")
    camera = None

# --- Loop Principal do Controlador ---
while robot.step(timestep) != -1:
    # --- Leitura dos Sensores ---

    # 1. Leitura do LIDAR
    if lidar:
        # getRangeImage() retorna uma lista de floats, onde cada valor é a distância
        # de um raio do LIDAR. O número de valores depende da resolução do LIDAR.
        # lidar_data = lidar.getRangeImage()
        # print(f"LIDAR Data (first 5 values): {lidar_data[:5]}")

        # Se habilitou enablePointCloud(), você pode obter a nuvem de pontos 3D
        # point_cloud = lidar.getPointCloud()
        # print(f"LIDAR Point Cloud (number of points): {len(point_cloud)}")

        pass # Remova este 'pass' para usar os dados do LIDAR

    # 2. Leitura dos Sensores de Distância
    # Percorre todos os sensores de distância que foram encontrados e habilitados
    # for i, ds in enumerate(distance_sensors):
    #    val = ds.getValue()
    #    print(f"Sensor {i} (so{i}): {val:.2f} m")

    pass # Remova este 'pass' para usar os dados dos sensores de distância

    # 3. Leitura da Câmera
    if camera:
        # A imagem é um array de bytes no formato RGB/RGBA.
        # camera_image = camera.getImage()
        # width = camera.getWidth()
        # height = camera.getHeight()
        # print(f"Camera Image (width: {width}, height: {height}, size: {len(camera_image)})")
        pass # Remova este 'pass' para usar os dados da câmera

    # --- Processamento dos Dados e Tomada de Decisão (Aqui entra sua RNA e Rede Bayesiana) ---
    # É aqui que você vai integrar:
    # 1. Sua RNA inferindo DistToObject e AngToTarget a partir da câmera e LIDAR.
    # 2. O mapeamento dessas saídas contínuas para probabilidades para a Rede Bayesiana.
    # 3. A inferência da Rede Bayesiana para decidir a 'Action' (seguir, virar esquerda, etc.).

    # Exemplo simples de movimento (apenas para teste inicial)
    # left_motor.setVelocity(2.0)
    # right_motor.setVelocity(2.0)


    # --- Envio de Comandos para Atuadores (Motores) ---
    # Com base na 'Action' decidida pela sua Rede Bayesiana, você controlará os motores.
    # if left_motor and right_motor: # Garante que os motores foram encontrados
    #     # Exemplo: mover para frente se nada for detectado ou se a Rede Bayesiana decidir "seguir"
    #     left_motor.setVelocity(2.0)
    #     right_motor.setVelocity(2.0)
    #     # Se 'Action' for 'virar esquerda':
    #     # left_motor.setVelocity(1.0)
    #     # right_motor.setVelocity(3.0)
    #     # ...e assim por diante para cada ação
