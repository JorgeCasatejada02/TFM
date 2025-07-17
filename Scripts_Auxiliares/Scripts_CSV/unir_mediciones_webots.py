import os
import pandas as pd

# Ruta raíz donde están las carpetas cargando/sin_cargar
base_dir = ""
# Lista de carpetas a procesar
folders = [
    "cargando_gui_gpu",
    "cargando_gui_no_gpu",
    "cargando_no_gui_no_gpu",
    "cargando_no_gui_gpu",
    "sin_cargar_gui_gpu",
    "sin_cargar_gui_no_gpu",
    "sin_cargar_no_gui_no_gpu",
    "sin_cargar_no_gui_gpu",
]

# Archivos CSV esperados
archivos_cargando = ["cpu_usage.csv", "gpu_usage.csv", "ram_usage.csv", "rtf.csv"]
archivos_sin_cargar = ["cpu_usage.csv", "gpu_usage.csv", "powerstat.csv", "ram_usage.csv", "rtf.csv"]

# Archivos que deben empezar desde la fila 181 (índice 180)
filtrar_desde_fila_181 = {"cpu_usage.csv", "gpu_usage.csv", "ram_usage.csv"}

# DataFrame final
df_final = pd.DataFrame()

for folder in folders:
    path_folder = os.path.join(base_dir, folder)

    # Detección precisa de condiciones
    bateria = "cargando" if "cargando" in folder else "sin_cargar"
    gui = "No" if "no_gui" in folder else "Si"
    gpu = "No" if "no_gpu" in folder else "Si"

    # Archivos que se deben buscar
    archivos = archivos_cargando if bateria == "cargando" else archivos_sin_cargar

    for repeticion in range(1, 6):  # repeticiones 1 a 5
        rep_folder_name = f"{folder}_{repeticion}"
        rep_folder_path = os.path.join(path_folder, rep_folder_name)

        for archivo in archivos:
            archivo_path = os.path.join(rep_folder_path, archivo)
            if os.path.exists(archivo_path):
                try:
                    # Leer normalmente o desde fila 181 si aplica
                    if bateria == "sin_cargar" and archivo in filtrar_desde_fila_181:
                        df = pd.read_csv(archivo_path, skiprows=range(1, 180))  # Mantener cabecera
                    else:
                        df = pd.read_csv(archivo_path)

                    # Añadir metadatos
                    df["source_file"] = archivo
                    df["Rep"] = repeticion
                    df["Battery"] = bateria
                    df["GUI"] = gui
                    df["GPU"] = gpu
                    df["Folder"] = folder

                    # Agregar al DataFrame final
                    df_final = pd.concat([df_final, df], ignore_index=True)

                except Exception as e:
                    print(f"❌ Error al leer {archivo_path}: {e}")
            else:
                print(f"⚠️ Archivo no encontrado: {archivo_path}")

# Guardar el CSV combinado
df_final.to_csv("mediciones_unificadas.csv", index=False)
print("✅ CSV unificado generado como 'mediciones_unificadas.csv'")
