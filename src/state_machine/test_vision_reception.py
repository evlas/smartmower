#!/usr/bin/env python3
"""
Test specifico per verificare la ricezione MQTT dei moduli vision
Monitora i log in tempo reale per verificare se perimeter e obstacle detector
stanno ricevendo e processando le immagini dalla camera
"""

import subprocess
import threading
import time
import signal
import sys
from datetime import datetime

class VisionReceptionTest:
    def __init__(self):
        self.running = True
        self.processes = {}
        self.log_threads = {}
        
        # Configurazione servizi da monitorare
        self.services = {
            'grass_detector': {
                'service': 'grass_detector',
                'expected_logs': ['Frame received:', '[GRASS]', '[PUBLISH]']
            },
            'perimeter_detector': {
                'service': 'perimeter_detector', 
                'expected_logs': ['Frame ricevuto:', 'detect_perimeter', '[PERIMETER]']
            },
            'obstacle_detector': {
                'service': 'obstacle_detector',
                'expected_logs': ['[IMAGE] Frame received:', 'Processing frame', 'Valid obstacles']
            }
        }
        
        self.reception_status = {
            'grass_detector': {'received': False, 'processed': False, 'published': False},
            'perimeter_detector': {'received': False, 'processed': False, 'published': False},
            'obstacle_detector': {'received': False, 'processed': False, 'published': False}
        }
        
    def signal_handler(self, signum, frame):
        """Gestisce l'interruzione del test"""
        print(f"\n🛑 Test interrotto (signal {signum})")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def monitor_service_logs(self, service_name, service_config):
        """Monitora i log di un servizio specifico"""
        cmd = ['journalctl', '-u', service_config['service'], '-f', '--no-pager', '-n', '0']
        
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )
            
            self.processes[service_name] = process
            
            print(f"📡 Monitoraggio {service_name} avviato")
            
            while self.running and process.poll() is None:
                line = process.stdout.readline()
                if line:
                    line = line.strip()
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    # Verifica se la riga contiene log di ricezione
                    for expected_log in service_config['expected_logs']:
                        if expected_log in line:
                            print(f"[{timestamp}] 🎯 {service_name.upper()}: {line}")
                            
                            # Aggiorna lo status di ricezione
                            if 'Frame' in expected_log or 'IMAGE' in expected_log:
                                self.reception_status[service_name]['received'] = True
                            elif 'Processing' in expected_log or 'detect_' in expected_log:
                                self.reception_status[service_name]['processed'] = True
                            elif 'PUBLISH' in expected_log or 'PERIMETER' in expected_log or 'obstacles' in expected_log:
                                self.reception_status[service_name]['published'] = True
                            
                            break
                    else:
                        # Mostra anche altri log importanti
                        if any(keyword in line.lower() for keyword in ['mqtt', 'error', 'exception', 'failed']):
                            print(f"[{timestamp}] ℹ️  {service_name}: {line}")
                
        except Exception as e:
            print(f"❌ Errore monitoraggio {service_name}: {e}")
        finally:
            if service_name in self.processes:
                try:
                    self.processes[service_name].terminate()
                except:
                    pass
    
    def start_camera_simulator(self):
        """Avvia il simulatore camera"""
        print("🎬 Avvio simulatore camera...")
        
        try:
            process = subprocess.Popen([
                'python3', 'camera_simulator.py',
                '--fps', '0.5',
                '--duration', '2'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.processes['camera_simulator'] = process
            return True
            
        except Exception as e:
            print(f"❌ Errore avvio simulatore camera: {e}")
            return False
    
    def show_status_report(self):
        """Mostra il report dello status di ricezione"""
        print(f"\n📊 Report Status Vision Reception:")
        print("=" * 60)
        
        for service_name, status in self.reception_status.items():
            print(f"\n🔍 {service_name.upper().replace('_', ' ')}:")
            
            # Status ricezione
            if status['received']:
                print("   ✅ RICEVE immagini MQTT")
            else:
                print("   ❌ NON riceve immagini MQTT")
            
            # Status processamento
            if status['processed']:
                print("   ✅ PROCESSA immagini")
            else:
                print("   ❌ NON processa immagini")
            
            # Status pubblicazione
            if status['published']:
                print("   ✅ PUBBLICA risultati")
            else:
                print("   ❌ NON pubblica risultati")
            
            # Diagnosi complessiva
            if all(status.values()):
                print("   🎯 STATUS: COMPLETAMENTE FUNZIONANTE")
            elif status['received']:
                print("   ⚠️  STATUS: RICEVE MA NON PROCESSA/PUBBLICA")
            else:
                print("   🚫 STATUS: NON RICEVE IMMAGINI")
    
    def cleanup(self):
        """Pulisce i processi avviati"""
        print("\n🧹 Pulizia processi...")
        
        for name, process in self.processes.items():
            try:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
                    print(f"   ✅ {name} terminato")
            except Exception as e:
                print(f"   ⚠️  Errore terminazione {name}: {e}")
                try:
                    process.kill()
                except:
                    pass
    
    def run_test(self, duration_minutes=3):
        """Esegue il test completo"""
        print("🚀 Avvio Test Vision Reception")
        print(f"   Durata: {duration_minutes} minuti")
        print(f"   Servizi monitorati: {', '.join(self.services.keys())}")
        print("=" * 60)
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Avvia monitoraggio log per ogni servizio
        for service_name, service_config in self.services.items():
            thread = threading.Thread(
                target=self.monitor_service_logs,
                args=(service_name, service_config),
                daemon=True
            )
            thread.start()
            self.log_threads[service_name] = thread
            time.sleep(0.5)  # Piccola pausa tra avvii
        
        # Aspetta che i monitor si stabilizzino
        time.sleep(2)
        
        # Avvia il simulatore camera
        if not self.start_camera_simulator():
            print("❌ Impossibile avviare il simulatore camera")
            self.cleanup()
            return False
        
        print(f"\n⏱️  Test in esecuzione per {duration_minutes} minuti...")
        print("   Premi Ctrl+C per interrompere")
        
        # Aspetta per la durata specificata
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        try:
            while time.time() < end_time and self.running:
                time.sleep(1)
                
                # Mostra aggiornamento ogni 30 secondi
                if int(time.time() - start_time) % 30 == 0:
                    elapsed = int(time.time() - start_time)
                    remaining = int(end_time - time.time())
                    print(f"\n⏰ Tempo trascorso: {elapsed}s, rimanente: {remaining}s")
                    
        except KeyboardInterrupt:
            print("\n⏹️  Test interrotto dall'utente")
        
        # Mostra report finale
        self.show_status_report()
        
        # Cleanup
        self.cleanup()
        
        return True

def main():
    """Funzione principale"""
    duration = 3  # Default 3 minuti
    
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("Uso: python3 test_vision_reception.py [durata_minuti]")
            sys.exit(1)
    
    test = VisionReceptionTest()
    success = test.run_test(duration)
    
    if success:
        print("\n✅ Test completato con successo")
    else:
        print("\n❌ Test fallito")
        sys.exit(1)

if __name__ == "__main__":
    main()
