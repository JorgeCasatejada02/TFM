import os
import pandas as pd

# Diccionario con los escenarios deseados
escenarios = {
    'cargando_gui_gpu': ("GUI Si", "GPU Si", "cargando"),
    'cargando_gui_no_gpu': ("GUI Si", "GPU No", "cargando"),
    'cargando_no_gui_no_gpu': ("GUI No", "GPU No", "cargando"),
    'cargando_no_gui_gpu': ("GUI No", "GPU Si", "cargando"),
    'sin_cargar_gui_gpu': ("GUI Si", "GPU Si", "sin_cargar"),
    'sin_cargar_gui_no_gpu': ("GUI Si", "GPU No", "sin_cargar"),
    'sin_cargar_no_gui_no_gpu': ("GUI No", "GPU No", "sin_cargar"),
    'sin_cargar_no_gui_gpu': ("GUI No", "GPU Si", "sin_cargar")
}

# Rutas a procesar
rutas_simuladores = {
    "Ignition Gazebo": os.path.expanduser("~/Escritorio/2_Cuatrimestre/TFM/turtlebot4_ws/mediciones"),
    "Webots": os.path.expanduser("~/Escritorio/2_Cuatrimestre/TFM/webots_ws/mediciones"),
    "Isaac Sim": os.path.expanduser("~/isaacsim/IsaacSim-ros_workspaces/humble_ws/mediciones")
}

# Archivos a filtrar desde la fila 181 en escenarios sin_cargar
archivos_filtrados = {"cpu_usage.csv", "gpu_usage.csv", "ram_usage.csv"}

# Función para determinar el escenario desde la carpeta
def determinar_escenario(nombre_carpeta):
    if nombre_carpeta in escenarios:
        return escenarios[nombre_carpeta]
    elif "gui" in nombre_carpeta.lower():  # Para Isaac Sim u otras carpetas similares
        tipo = "sin_cargar" if "sin_cargar" in nombre_carpeta else "cargando"
        gui = "GUI No" if "no_gui" in nombre_carpeta else "GUI Si"
        gpu = "GPU Si"
        return (gui, gpu, tipo)
    return None

def procesar_mediciones():
    resultados = {}  # {(GUI, GPU, Estado): lista de DataFrames}

    for simulador, base_path in rutas_simuladores.items():
        for nombre_carpeta in os.listdir(base_path):
            ruta_escenario = os.path.join(base_path, nombre_carpeta)
            if not os.path.isdir(ruta_escenario):
                continue

            escenario = determinar_escenario(nombre_carpeta)
            if escenario is None:
                continue

            gui, gpu, estado = escenario
            datos_escenario = []

            for repeticion in os.listdir(ruta_escenario):
                ruta_repeticion = os.path.join(ruta_escenario, repeticion)
                if not os.path.isdir(ruta_repeticion):
                    continue

                for archivo in os.listdir(ruta_repeticion):
                    if archivo.endswith(".csv"):
                        ruta_archivo = os.path.join(ruta_repeticion, archivo)

                        # Leer CSV, aplicando filtro si es necesario
                        if estado == "sin_cargar" and archivo in archivos_filtrados:
                            df = pd.read_csv(ruta_archivo, skiprows=range(1, 181))
                        else:
                            df = pd.read_csv(ruta_archivo)

                        df["Simulador"] = simulador
                        df["Escenario"] = nombre_carpeta
                        df["Medicion"] = archivo.replace(".csv", "")
                        df["Repeticion"] = repeticion.split("_")[-1]
                        df["GUI"] = gui
                        df["GPU"] = gpu
                        df["Estado"] = estado

                        datos_escenario.append(df)

            # Guardar por escenario
            if escenario in resultados:
                resultados[escenario].extend(datos_escenario)
            else:
                resultados[escenario] = datos_escenario

    # Crear carpeta de salida
    output_dir = "resultados_escenarios_2"
    os.makedirs(output_dir, exist_ok=True)

    # Guardar CSVs individuales por escenario
    for escenario, dataframes in resultados.items():
        gui, gpu, estado = escenario
        df_final = pd.concat(dataframes, ignore_index=True)
        nombre_archivo = f"{estado}_{gui.replace(' ', '').lower()}_{gpu.replace(' ', '').lower()}.csv"
        df_final.to_csv(os.path.join(output_dir, nombre_archivo), index=False)
        print(f"[✔] Guardado: {nombre_archivo}")

    # Guardar CSV general con todos los datos
    df_todos = pd.concat(
        [df for dfs in resultados.values() for df in dfs],
        ignore_index=True
    )
    df_todos.to_csv(os.path.join(output_dir, "todos_los_escenarios.csv"), index=False)
    print("[✔] Guardado: todos_los_escenarios.csv")

if __name__ == "__main__":
    procesar_mediciones()
