# Prerequisitos

- Tener [Webots](https://cyberbotics.com/) instalado.
- Tener el [paquete Webots ROS 2](https://github.com/cyberbotics/webots_ros2.git) instalado.
- Tutorial de instalación de simulador y paquete ROS 2 disponible en [Webots.md](../Tutoriales_Instalacion_Simuladores/Webots.md`).

# Uso del paquete
Después de esta sección se encuentra como ha sido desarrollado el paquete por completo. Para su uso simplemente se tendrá que descargar el paquete, añadirlo al repositorio propio de webots_ros2, hacer colcon build, source y lanzarlo:
```bash
cp -r Descargas/webots_ros2_example webots_ws/src/webots_ros2/
cd webots_ws
colcon build
source install/setup.bash
ros2 launch webots_ros2_example robot_launch.py rviz:=true nav:=true slam_toolbox:=true
```

# Desarrollo de un nuevo robot en Webots ROS 2 con Nav2

## 1. Creación de un nuevo paquete

En la carpeta `src/webots_ros2` de tu workspace de Webots ROS 2, crea un paquete:
```bash
ros2 pkg create --build-type ament_python webots_ros2_example
```

Dentro del nuevo paquete, crea las carpetas:
- `launch`
- `worlds`

Opcionalmente, se podría añadir una carpeta `protos` para almacenar modelos locales, pero en este ejemplo usaremos uno de los que ofrece Cyberbotics en su [cloud](https://webots.cloud/proto), el *Pioneer 3-AT*:

> [https://webots.cloud/run?version=R2025a&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwebots%2Fblob%2Freleased%2Fprojects%2Frobots%2Fadept%2Fpioneer3%2Fprotos%2FPioneer3at.proto](https://webots.cloud/run?version=R2025a\&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwebots%2Fblob%2Freleased%2Fprojects%2Frobots%2Fadept%2Fpioneer3%2Fprotos%2FPioneer3at.proto)

## 2. Estructura del proyecto

```text
webots_ros2_example/
├── launch/
├── resource/
├── test/
├── webots_ros2_example/
├── worlds/
└── package.xml
└── setup.cfg
└── setup.py
```

## 3. Carpeta `worlds`

En este ejemplo se hará uso del mundo que Webots ya ofrece para el robot *Pioneer 3-AT*. 

Primero descargamos el mundo de ejemplo y lo colocamos en la carpeta `worlds`:
```
wget https://raw.githubusercontent.com/cyberbotics/webots/released/projects/robots/adept/pioneer3/worlds/pioneer3at.wbt -P worlds/
```
Este mundo incluye el robot junto con un sensor lidar SICK equipado.

Una vez descargado, lo modificaremos para poder usarlo junto a ROS 2 y Nav2. Modificamos el controlador para que ahora sea externo y actualizamos el sensor LIDAR para que tenga como nombre `scan`. La definición del robot quedaría así:
```wbt
Pioneer3at {
  rotation 0 0 1 -0.25
  controller "<extern>"
  name "Pioneer3at"
  extensionSlot [
    SickLms291 {
      translation 0.136 0 0.35
      name "scan"
    }
  ]
}
```

También tendremos que modificar las rutas de los `EXTERNPROTO`, ajustándolas a las propias del ordenador o por su enlace en Github::
```wbt
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/appearances/protos/SandyGround.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/adept/pioneer3/protos/Pioneer3at.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/sick/protos/SickLms291.proto"
```

Y por último modificar de la misma manera los enlaces a las texturas y rocas:
```wbt
url [
        "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/default/worlds/textures/gray_brick_wall.jpg"
      ]
url [
        "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/default/worlds/textures/rock.jpg"
      ]
```

## 4. Carpeta `resource`

Esta carpeta contendrá los ficheros necesarios para el correcto funcionamiento de rviz, Nav2, SLAM y ROS 2 control además de el urdf del robot que se utiliza como archivo de descripción del robot para la simulación Webots.

### 4.1. RViz
Tendremos un archivo de configuración, `default.rviz`, para la visualización RViz que define los ajustes de visualización para el robot Pioneer 3AT, incluido el modelo del robot, los datos de los sensores, los mapas y los elementos de navegación. Se usará como base el del [ejemplo de TIAGo](https://github.com/cyberbotics/webots_ros2/blob/a5a9c3f82b0056809e89c976b9e9f45137c0fd57/webots_ros2_tiago/resource/default.rviz), modificando para adaptarlo a nuestro robot pioneer3at, cambiando los componenets del TIAGo:
- (Torso, base_cover_link, head_1_link, head_2_link, wheel_left_link, wheel_right_link, caster_front/back_left/right…)

Por los del pioneer:
- (base_link, front/back left/right wheel, Sick_LMS_291 (sensor láser), so0-so15 (sensores)).

ARCHIVO [default.rviz](resource/default.rviz)

### 4.2. Nav2
Necesitaremos un archivo `nav2_params.yaml` para la configuración de Nav2 que contiene todos los parámetros de la pila Navigation2 para la navegación autónoma, incluidos los ajustes para la localización (AMCL), la planificación de trayectorias, los controladores, los mapas de costes y los comportamientos. Este también lo obtendremos del [ejemplo de TIAGo](https://github.com/cyberbotics/webots_ros2/blob/a5a9c3f82b0056809e89c976b9e9f45137c0fd57/webots_ros2_tiago/resource/nav2_params.yaml) y no cambiaremos nada.

ARCHIVO [nav2_params.yaml](resource/nav2_params.yaml)

### 4.3. SLAM

Primeor necesitaremos instalar SLAM Toolbox para su posterior uso:
```bash
sudo apt install ros-<ros2-distro>-slam-toolbox
```

El archivo de SLAM que necesitaremos será el `slam_toolbox_params.yaml`, que contiene la configuración de los parámetros de SLAM utilizada para la localización y el mapeo simultáneos. Este también lo obtendremos del [ejemplo de TIAGo](https://github.com/cyberbotics/webots_ros2/blob/a5a9c3f82b0056809e89c976b9e9f45137c0fd57/webots_ros2_tiago/resource/slam_toolbox_params.yaml) y no cambiaremos nada.

ARCHIVO [slam_toolbox_params.yaml](resource/slam_toolbox_params.yaml)

### 4.4. ROS 2 Control

Tendremos un archivo para la configuración del framework ros2_control, `ros2_control.yml`, que define cómo se controlan las cuatro ruedas del robot Pioneer 3AT como un sistema de tracción diferencial. Se usará como base el del [ejemplo de TIAGo](https://github.com/cyberbotics/webots_ros2/blob/a5a9c3f82b0056809e89c976b9e9f45137c0fd57/webots_ros2_tiago/resource/ros2_control.yml) pero modificándolo para los nombres de las 4 ruedas de nuestro robot y sus medidas. Estos nombres se obtiene del archivo [proto de *Pioneer 3-AT*](https://github.com/cyberbotics/webots/blob/released/projects/robots/adept/pioneer3/protos/Pioneer3at.proto) y las medidas del [datasheet del robot](https://www.generationrobots.com/media/Pioneer3AT-P3AT-RevA-datasheet.pdf):
```yml
left_wheel_names: ["front left wheel", "back left wheel"]
right_wheel_names: ["front right wheel", "back right wheel"]
wheel_separation: 0.381
wheels_per_side: 2
wheel_radius: 0.111
```

ARCHIVO [ros2_control.yml](resource/ros2_control.yml)

### 4.5. URDF

Este urdf, `pioneer3at_webots.urdf`, contiene la descripción del modelo URDF del robot Pioneer 3AT para la simulación Webots, definiendo el sensor lidar y la configuración ros2_control para las interfaces de las ruedas. Se carga en el nodo pioneer3at_driver (WebotsController) como parámetro, donde sirve para:
- Definir la configuración del sensor lidar para la publicación de datos de escaneo a ROS2
- Configurar el plugin webots_ros2_control para la interconexión entre Webots y ROS2
- Configurar las interfaces de las articulaciones de las ruedas necesarias para el controlador de accionamiento diferencial

El URDF crea el puente entre el robot Pioneer 3AT simulado en Webots y el sistema de control ROS2, permitiendo la correcta publicación de datos del sensor y el control de las ruedas. Se usará como base el del [ejemplo de TIAGo](https://github.com/cyberbotics/webots_ros2/blob/a5a9c3f82b0056809e89c976b9e9f45137c0fd57/webots_ros2_tiago/resource/tiago_webots.urdf) el cual modificaremos.

Los cambios realizados son:
1. Nombre del robot:
```urdf
<robot name="Pioneer3at">
```

2. Sensor LIDAR:
```urdf
<device reference="scan" type="Lidar">
  <ros>
    <enabled>true</enabled>
    <updateRate>10</updateRate>
    <topicName>/scan</topicName>
    <alwaysOn>true</alwaysOn>
    <frameName>scan</frameName>
    </ros>
</device>
```

Se cambia el nombre del sensor de hokuyo a scan, el frameName a scan, más genérico y típico en ROS 2 y el resto de la configuración (topicName, updateRate, etc.) es igual(el sensor sigue publicando en /scan).

3. Articulaciones / Ruedas:
```urdf
<joint name="front left wheel">
    <state_interface name="position"/>
    <command_interface name="velocity"/>
</joint>
<joint name="front right wheel">
    <state_interface name="position"/>
    <command_interface name="velocity"/>
</joint>
<joint name="back left wheel">
    <state_interface name="position"/>
    <command_interface name="velocity"/>
</joint>
<joint name="back right wheel">
    <state_interface name="position"/>
    <command_interface name="velocity"/>
</joint>
```

Se cambió el nombre de las articulaciones a las del pioneer, estos nombres se obtiene del archivo [proto de *Pioneer 3-AT*](https://github.com/cyberbotics/webots/blob/released/projects/robots/adept/pioneer3/protos/Pioneer3at.proto).

ARCHIVO [pioneer3at_webots.urdf](resource/pioneer3at_webots.urdf)


## 5. Carpeta `launch`

En esta carpeta se creará el archivo de lanzamiento del robot, usando como base el [ejemplo de TIAGo](https://github.com/cyberbotics/webots_ros2/blob/a5a9c3f82b0056809e89c976b9e9f45137c0fd57/webots_ros2_tiago/launch/robot_launch.py).

En este tendremos que modificar:

- Actualizar  el directorio `package_dir`:
```python
package_dir = get_package_share_directory('webots_ros2_example')
```

- Ruta al URDF del robot:
```python
robot_description_path = os.path.join(package_dir, 'resource', 'pioneer3at_webots.urdf')
```

- Nodo `WebotsController`:
```python
pioneer3at_driver = WebotsController(
  robot_name='Pioneer3at',
  parameters=[
      {'robot_description': robot_description_path,
        'use_sim_time': use_sim_time,
        'set_robot_state_publisher': True},
      ros2_control_params
  ],
  remappings=mappings,
  respawn=True
)
```

- Ruta a configuración de Rviz:
```python
rviz_config = os.path.join(get_package_share_directory('webots_ros2_pioneer3at'), 'resource', 'default.rviz')
```

- El lanzamiento de la navegación para no necesitar de mapa:
```python
navigation_nodes = []
  nav2_params_file = 'nav2_params.yaml'
  nav2_params = os.path.join(package_dir, 'resource', nav2_params_file)
  if 'nav2_bringup' in get_packages_with_prefixes():
      navigation_nodes.append(IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(
              get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
          launch_arguments=[
              ('params_file', nav2_params),
              ('use_sim_time', use_sim_time),
          ],
          condition=launch.conditions.IfCondition(use_nav)))
```

- El nodo WaitForControllerConnection para especificar el driver de nuestro pioneer:
```python
waiting_nodes = WaitForControllerConnection(
    target_driver=pioneer3at_driver,
    nodes_to_start=[rviz] + navigation_nodes + ros_control_spawners
)
```

- Modificar el mundo por defecto lanzado:
```python
DeclareLaunchArgument(
    'world',
    default_value='pioneer3at.wbt',
    description='Choose one of the world files from `/webots_ros2_pioneer3at/world` directory'
)
```

- Cambiar el driver de tiago al de pioneer en nuestro LaunchDescription (pioneer3at_driver).
- Por último eliminar todo lo relacionado con slam_cartographer ya que haremos uso de slam_toolbox.

ARCHIVO [robot_launch.py](launch/robot_launch.py)

## 6. Archivos de configuración del paquete

### `package.xml`
Añadiremos las dependencias necesarias para el correcto funcionamiento y demás información.

ARCHIVO [package.xml](package.xml)

### `setup.py`
Incluiremos los archivos:
- Scripts de lanzamiento (launch/robot_launch.py)
- Archivos de configuración y URDF (resource/*.yaml, .urdf, .rviz)
- Mundo de simulación (worlds/*.wbt)
```python
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/pioneer3at_webots.urdf',
    'resource/ros2_control.yml',
    'resource/nav2_params.yaml',
    'resource/default.rviz',
    'resource/slam_toolbox_params.yaml',
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/pioneer3at.wbt',
]))
```

Esto nos permite que se instalen correctamente con colcon y se puedan usar con ros2 launch, RViz, etc.

También la dependencia de launch para usar launch_ros en ROS 2:
```python
install_requires=['setuptools', 'launch']
```

Y lo entry points actualizados, que permiten usar plugins de lanzamiento de ROS 2:
```python
'launch.frontend.launch_extension': ['launch_ros = launch_ros']
```

ARCHIVO [setup.py](setup.py)

## 7. Lanzamiento del ejemplo
Una vez configurado todo podemos realizar el lanzamiento. Primero tendremos que hacer un colcon build del workspace y su source:
```bash
cd webots_ws
colcon build
source install/setup.bash
```

Y podemos lanzar el ejemplo con navegación y rviz iniciada con:
```bash
ros2 launch webots_ros2_example robot_launch.py rviz:=true nav:=true slam_toolbox:=true
```

En RViz, usar Nav2 Goal para mover el robot, podemos ver como funciona en el siguiente video:

https://drive.google.com/file/d/1tttXQCAq6QeV7lREsmQbtxjoj_ssuI1X/view?usp=sharing

# Uso de otro mapa

Vamos ahora a probar con un mapa más grande, de exteriores, y comprobar que funciona de igual manera. Usaremos el [mundo del tractor boomer](https://github.com/cyberbotics/webots/blob/cf743661107268382087198169b17749bc511dd2/projects/vehicles/worlds/boomer.wbt).

```bash
wget https://raw.githubusercontent.com/cyberbotics/webots/cf743661107268382087198169b17749bc511dd2/projects/vehicles/worlds/boomer.wbt -P worlds/
```

Una vez descargado sustituimos el tractor por nuestro robot:
```wbt
Pioneer3at {
  rotation 0 0 1 -0.25
  controller "<extern>"
  name "Pioneer3at"
  extensionSlot [
    SickLms291 {
      translation 0.136 0 0.35
      name "scan"
    }
  ]
}
```

Y añadimos las dependencias a protos necesarias:
```wbt
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/adept/pioneer3/protos/Pioneer3at.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/sick/protos/SickLms291.proto"
```

ARCHIVO [boomer.wbt](worlds/boomer.wbt)

Y podemos lanzar el ejemplo usando otro mapa especificándolo por parámetros con:

```bash
ros2 launch webots_ros2_example robot_launch.py rviz:=true nav:=true slam_toolbox:=true world:=.../src/webots_ros2/webots_ros2_example/worlds/boomer.wbt
```

Podemos ver como funciona en el siguiente video:

https://drive.google.com/file/d/1z_qBT7xzMaenFWKZJ2qtONZG8yN0qx6t/view?usp=sharing

