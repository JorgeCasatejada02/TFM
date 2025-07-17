import os
import pandas as pd

# Ruta raíz donde están las carpetas
base_dir = ""

# Carpetas actuales
folders = [
    "cargando_gui",
    "cargando_no_gui",
    "sin_cargar_gui",
    "sin_cargar_no_gui",
]

# Archivos esperados por tipo de carpeta
archivos_cargando = ["cpu_usage.csv", "gpu_usage.csv", "ram_usage.csv", "rtf.csv"]
archivos_sin_cargar = ["cpu_usage.csv", "gpu_usage.csv", "powerstat.csv", "ram_usage.csv", "rtf.csv"]

# Archivos que deben saltarse hasta la fila 181 (cuando sin_cargar)
filtrar_desde_fila_181 = {"cpu_usage.csv", "gpu_usage.csv", "ram_usage.csv"}

# Inicializar DataFrame combinado
df_final = pd.DataFrame()

for folder in folders:
    path_folder = os.path.join(base_dir, folder)

    bateria = "cargando" if folder.startswith("cargando") else "sin_cargar"
    gui = "No" if "no_gui" in folder else "Si"
    gpu = "Si"  # GPU siempre

    archivos = archivos_cargando if bateria == "cargando" else archivos_sin_cargar

    for repeticion in range(1, 6):
        rep_folder = f"{folder}_{repeticion}"
        rep_path = os.path.join(path_folder, rep_folder)

        for archivo in archivos:
            file_path = os.path.join(rep_path, archivo)
            if os.path.exists(file_path):
                try:
                    if bateria == "sin_cargar" and archivo in filtrar_desde_fila_181:
                        df = pd.read_csv(file_path, skiprows=range(1, 180))  # mantiene cabecera, salta 1‑179
                    else:
                        df = pd.read_csv(file_path)

                    # Añadir columnas
                    df["archivo_origen"] = archivo
                    df["repeticion"] = repeticion
                    df["Batería"] = bateria
                    df["GUI"] = gui
                    df["GPU"] = gpu
                    df["carpeta"] = folder

                    # Concatenar
                    df_final = pd.concat([df_final, df], ignore_index=True)
                except Exception as e:
                    print(f"❌ Error leyendo {file_path}: {e}")
            else:
                print(f"⚠️ Archivo no encontrado: {file_path}")

# Guardar resultado combinado
df_final.to_csv("mediciones_unificadas.csv", index=False)
print("✅ CSV unificado generado: 'mediciones_unificadas.csv'")
