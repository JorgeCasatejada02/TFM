import argparse
import time
import signal
import sys
from datetime import datetime

# Importar nuestros módulos
from metrics_collector import MetricsCollector, RTFCollector
from simulator_launcher import SimulatorLauncher
from navigation_controller import NavigationController

def parse_args():
    parser = argparse.ArgumentParser(description="Monitorización de simulación con ROS2 e Ignition")
    parser.add_argument('--folder', type=str, help="Nombre de la carpeta de salida", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument('--mode', type=str, choices=['sin_gpu_gui', 'con_gpu_gui', 'sin_gpu_sin_gui', 'con_gpu_sin_gui'], required=True, help="Modo de lanzamiento del simulador")
    parser.add_argument('--duration', type=int, help="Duración de la simulación en segundos", default=300)
    parser.add_argument('--interval', type=int, help="Intervalo de muestreo en segundos", default=1)
    parser.add_argument('--power-warmup', type=int, help="Tiempo de warmup para PowerStat (segundos)", default=180)
    parser.add_argument('--no-power', action='store_true', help="No medir consumo de batería")
    return parser.parse_args()

def main():
    # Iniciar el temporizador inmediatamente
    start_time = time.time()
    
    # Parsear argumentos
    args = parse_args()
    
    # Calcular la duración total (simulación + warmup si se usa powerstat)
    total_duration = args.duration
    if not args.no_power:
        total_duration += args.power_warmup
        
    print(f"⏳ Comenzando benchmark durante {total_duration}s totales...")
    if not args.no_power:
        print(f"   - {args.power_warmup}s de warmup para PowerStat")
        print(f"   - {args.duration}s de simulación activa")
    
    # Manejar señales para cerrar limpiamente
    def signal_handler(sig, frame):
        print("\n🛑 Recibida señal de interrupción. Finalizando...")
        cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Inicializar componentes
    metrics = MetricsCollector(args.folder, args.interval)
    simulator = SimulatorLauncher(args.mode)
    navigator = NavigationController()
    rtf_collector = None
    power_collector = None
    
    def cleanup():
        print("🧹 Limpiando recursos...")
        if rtf_collector:
            rtf_collector.stop()
        navigator.stop_navigation()
        simulator.stop()
        metrics.stop_all()
        print(f"✅ Benchmark finalizado. Datos guardados en: {args.folder}")
    
    try:
        # Iniciar recolección de métricas
        print("📊 Iniciando recolección de métricas...")
        if not args.no_power:
            power_collector = metrics.start_all(args.duration, args.power_warmup)
        else:
            metrics.start_all()
        
        # Si estamos utilizando PowerStat, esperar a que termine el período de warmup
        if not args.no_power:
            print(f"⏳ Esperando {args.power_warmup}s para que PowerStat complete su período de warmup...")
            if not power_collector.wait_for_warmup(timeout=args.power_warmup + 5):  # +5s de margen
                print("⚠️ Tiempo de espera para warmup de PowerStat agotado, continuando de todas formas")
        
        # Lanzar simulador
        simulator.launch()
        
        # Esperar a que el simulador esté listo
        print("⏳ Esperando a que el simulador esté listo...")
        if not simulator.wait_until_ready(timeout=60):
            print("❌ Tiempo de espera agotado. El simulador no está listo.")
            cleanup()
            return 1
            
        # Iniciar recolección de RTF
        print("📊 Iniciando recolección de RTF...")
        rtf_collector = RTFCollector(args.folder, args.interval)
        rtf_thread = threading.Thread(target=rtf_collector.run)
        rtf_thread.daemon = True
        rtf_thread.start()
        
        # Configurar posición inicial
        print("🤖 Configurando posición inicial...")
        if not navigator.set_initial_pose():
            print("❌ Error al configurar posición inicial")
            cleanup()
            return 1
        
        time.sleep(5)
        
        # Iniciar navegación
        print("🧭 Iniciando navegación a waypoints...")
        navigator.start_navigation()
        
        # Calcular el tiempo restante para la duración de la simulación
        # (no contar el tiempo de warmup de PowerStat que ya pasó)
        elapsed_time = time.time() - start_time
        sim_duration = args.duration if args.no_power else total_duration - args.power_warmup
        remaining_time = sim_duration - (elapsed_time - (0 if args.no_power else args.power_warmup))
        
        if remaining_time > 0:
            print(f"⏳ Esperando {remaining_time:.1f} segundos para completar la duración de la simulación...")
            time.sleep(remaining_time)
        
        print(f"🏁 Benchmark completado en {time.time() - start_time:.1f} segundos")
        
    except Exception as e:
        print(f"❌ Error durante la ejecución: {e}")
        return 1
        
    finally:
        cleanup()
        try:
            import rclpy
            if rclpy.ok():
                print("🔌 Cerrando ROS2...")
                rclpy.shutdown()
        except Exception as e:
            print(f"⚠️ Error al cerrar ROS2: {e}")
    
    return 0

if __name__ == "__main__":
    import threading  # Importado aquí para evitar problemas con el global
    sys.exit(main())