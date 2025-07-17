import os
import time
import csv
import psutil
import subprocess
import threading
import re

class MetricsCollector:
    def __init__(self, output_folder, interval=1):
        self.output_folder = output_folder
        self.interval = interval
        self.running = False
        self.collectors = []
        os.makedirs(output_folder, exist_ok=True)
        
    def start_all(self, duration=300, power_warmup=180):
        self.running = True
        
        # Crear y configurar colectores individuales
        cpu_collector = CPUCollector(self.output_folder, self.interval)
        ram_collector = RAMCollector(self.output_folder, self.interval)
        gpu_collector = GPUCollector(self.output_folder, self.interval)
        power_collector = PowerCollector(self.output_folder, duration, self.interval, power_warmup)
        
        # Iniciar todos los colectores en hilos separados
        self.collectors = [cpu_collector, ram_collector, gpu_collector, power_collector]
        self.threads = []
        
        for collector in self.collectors:
            thread = threading.Thread(target=collector.run)
            thread.daemon = True
            thread.start()
            self.threads.append(thread)
        
        # Devolver el colector de energía para poder esperar su warmup
        return power_collector
    
    def stop_all(self):
        self.running = False
        for collector in self.collectors:
            collector.stop()
        
        # Esperar a que todos los hilos terminen
        for thread in self.threads:
            thread.join(timeout=5)

class BaseCollector:
    def __init__(self, output_folder, interval):
        self.output_folder = output_folder
        self.interval = interval
        self.running = False
        self.csv_file = None
        self.csv_writer = None
    
    def setup(self):
        pass
    
    def collect_data(self):
        pass
    
    def cleanup(self):
        if self.csv_file:
            self.csv_file.close()
    
    def run(self):
        self.running = True
        self.setup()
        
        while self.running:
            try:
                self.collect_data()
                time.sleep(self.interval)
            except Exception as e:
                print(f"Error en {self.__class__.__name__}: {e}")
        
        self.cleanup()
    
    def stop(self):
        self.running = False

class CPUCollector(BaseCollector):
    def setup(self):
        cpu_csv_path = os.path.join(self.output_folder, "cpu_usage.csv")
        self.csv_file = open(cpu_csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "cpu_percent", "cpu_user_percent", "cpu_system_percent", 
            "cpu_idle_percent", "cpu_freq_MHz", "load_avg_1", "load_avg_5", 
            "load_avg_15", "total_tasks", "total_threads", "running_tasks"
        ])
    
    def collect_data(self):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # CPU info
        cpu_times = psutil.cpu_times_percent(interval=None)
        cpu_freq = psutil.cpu_freq()
        load1, load5, load15 = os.getloadavg()

        # Process info
        total_threads = 0
        running_tasks = 0
        all_procs = list(psutil.process_iter(['status', 'num_threads']))

        for p in all_procs:
            try:
                total_threads += p.info['num_threads']
                if p.info['status'] == psutil.STATUS_RUNNING:
                    running_tasks += 1
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        total_tasks = len(all_procs)

        self.csv_writer.writerow([
            timestamp,
            psutil.cpu_percent(interval=None),
            cpu_times.user,
            cpu_times.system,
            cpu_times.idle,
            round(cpu_freq.current, 2) if cpu_freq else 0,
            round(load1, 2), round(load5, 2), round(load15, 2),
            total_tasks, total_threads, running_tasks
        ])
        self.csv_file.flush()

class RAMCollector(BaseCollector):
    def setup(self):
        ram_csv_path = os.path.join(self.output_folder, "ram_usage.csv")
        self.csv_file = open(ram_csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "total_MB", "used_MB", "available_MB", "percent", 
            "free_MB", "active_MB", "inactive_MB", "buffers_MB", "cached_MB", 
            "swap_total_MB", "swap_used_MB", "swap_free_MB"
        ])
    
    def collect_data(self):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # RAM info
        vmem = psutil.virtual_memory()
        swap = psutil.swap_memory()
        
        self.csv_writer.writerow([
            timestamp,
            round(vmem.total / (1024*1024), 2),
            round(vmem.used / (1024*1024), 2),
            round(vmem.available / (1024*1024), 2),
            vmem.percent,
            round(vmem.free / (1024*1024), 2),
            round(getattr(vmem, "active", 0) / (1024*1024), 2),
            round(getattr(vmem, "inactive", 0) / (1024*1024), 2),
            round(getattr(vmem, "buffers", 0) / (1024*1024), 2),
            round(getattr(vmem, "cached", 0) / (1024*1024), 2),
            round(swap.total / (1024*1024), 2),
            round(swap.used / (1024*1024), 2),
            round(swap.free / (1024*1024), 2)
        ])
        self.csv_file.flush()

class GPUCollector(BaseCollector):
    def setup(self):
        gpu_csv_path = os.path.join(self.output_folder, "gpu_usage.csv")
        self.csv_file = open(gpu_csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "gpu_util_percent", "gpu_mem_used_MB", 
            "gpu_mem_total_MB", "gpu_power_W", "gpu_temp_C"
        ])
    
    def collect_data(self):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # GPU info
        try:
            output = subprocess.check_output([
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total,power.draw,temperature.gpu",
                "--format=csv,noheader,nounits"
            ]).decode("utf-8").strip()
            gpu_util, gpu_mem_used, gpu_mem_total, gpu_power, gpu_temp = output.split(", ")
        except subprocess.CalledProcessError:
            gpu_util = gpu_mem_used = gpu_mem_total = gpu_power = gpu_temp = "error"
        
        self.csv_writer.writerow([
            timestamp, gpu_util, gpu_mem_used, gpu_mem_total, gpu_power, gpu_temp
        ])
        self.csv_file.flush()

class RTFCollector(BaseCollector):
    def setup(self):
        rtf_csv_path = os.path.join(self.output_folder, "rtf.csv")
        self.csv_file = open(rtf_csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "real_time_factor", "iterations"])
        
        # Iniciar proceso de captura de RTF usando el nodo rtf_monitor
        self.rtf_process = subprocess.Popen(
            ["ros2", "run", "webots_ros2_turtlebot", "rtf_monitor"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT
        )
        
        # Iniciar hilo para leer RTF
        self.rtf_thread = threading.Thread(target=self._read_rtf, daemon=True)
        self.rtf_thread.start()
    
    def _read_rtf(self):
        """Lee continuamente la salida del proceso rtf_monitor"""
        for line in iter(self.rtf_process.stdout.readline, b''):
            if not self.running:
                break
                
            decoded = line.decode("utf-8").strip()
            # Busca una línea que contenga los datos de RTF y Steps
            match = re.search(r"RTF:\s*([\d.]+)\s*\|\s*Sim:.*?\|\s*Real:.*?\|\s*Steps:\s*(\d+)", decoded)
            if match:
                rtf = float(match.group(1))
                steps = int(match.group(2))
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                self.csv_writer.writerow([timestamp, rtf, steps])
                self.csv_file.flush()
    
    def collect_data(self):
        # El hilo _read_rtf se encarga de recopilar datos continuamente
        # Solo verificamos que el proceso siga ejecutándose
        if self.rtf_process.poll() is not None:
            print(f"⚠️ Proceso rtf_monitor terminó inesperadamente con código: {self.rtf_process.returncode}")
            self.running = False
    
    def cleanup(self):
        if self.rtf_process and self.rtf_process.poll() is None:
            self.rtf_process.terminate()
            try:
                self.rtf_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.rtf_process.kill()
        
        super().cleanup()
        
class PowerCollector(BaseCollector):
    def __init__(self, output_folder, duration, interval=1, warmup_delay=180):
        super().__init__(output_folder, interval)
        self.duration = duration
        self.warmup_delay = warmup_delay
        self.process = None
        # Evento para notificar cuando el período de warmup ha terminado
        self.warmup_complete_event = threading.Event()
        
    def setup(self):
        powerstat_log_path = os.path.join(self.output_folder, "powerstat.log")
        self.log_file = open(powerstat_log_path, "w")
        
        # Comando powerstat:
        # -n: sin color
        # -a: formato breve
        # <interval>: intervalo de muestreo
        # <count>: número de muestras
        powerstat_cmd = ["powerstat", "-n", "-a", str(self.interval), str(self.duration)]
        
        print(f"🔋 Iniciando monitorización de batería con PowerStat (warmup: {self.warmup_delay}s)")
        self.process = subprocess.Popen(
            powerstat_cmd, 
            stdout=self.log_file, 
            stderr=subprocess.STDOUT
        )
        
        # Iniciar un temporizador para marcar cuando termina el período de warmup
        threading.Timer(self.warmup_delay, self._notify_warmup_complete).start()
    
    def _notify_warmup_complete(self):
        print("✅ Período de warmup de PowerStat completado, listo para iniciar simulador")
        self.warmup_complete_event.set()
    
    def collect_data(self):
        # PowerStat recopila datos automáticamente en segundo plano
        # Solo necesitamos verificar si sigue funcionando
        if self.process and self.process.poll() is not None:
            print(f"⚠️ PowerStat terminó inesperadamente con código: {self.process.returncode}")
            self.running = False
    
    def wait_for_warmup(self, timeout=None):
        """Espera hasta que el período de warmup de PowerStat termine"""
        return self.warmup_complete_event.wait(timeout)
    
    def cleanup(self):
        if self.process and self.process.poll() is None:
            print("🛑 Deteniendo PowerStat...")
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        
        if hasattr(self, 'log_file'):
            self.log_file.close()
            
# Al final del archivo metrics_collector.py
__all__ = ['MetricsCollector', 'RTFCollector', 'PowerCollector']