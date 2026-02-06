import requests
from bs4 import BeautifulSoup
import re
import time
from datetime import datetime
import csv
import json
import os  

class FanucWebScraperAdvanced:

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.url = f"http://{robot_ip}/MD/CURPOS.DG"
        self.data_history = []
        
    def get_current_position(self):
        """Holt aktuelle Position"""
        try:
            response = requests. get(self.url, timeout=5)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.text, 'html.parser')
            text = soup.get_text()
            
            data = {
                'timestamp': datetime.now().isoformat(),
                'unix_time': time.time(),
                'group': None,
                'joints': {}
            }
            
            # Group extrahieren
            group_match = re.search(r'Group #:\s*(\d+)', text)
            if group_match: 
                data['group'] = int(group_match.group(1))
            
            # Joints extrahieren
            joint_pattern = r'Joint\s+(\d+):\s*([-+]?\d+\.?\d*)'
            joints = re. findall(joint_pattern, text)
            
            for joint_num, value in joints:
                data['joints'][f'J{joint_num}'] = float(value)
            
            # Zu History hinzufügen
            self.data_history.append(data)
            
            return data
            
        except Exception as e:
            print(f"? Fehler:  {e}")
            return None
    
    def save_to_csv(self, filename="robot_positions.csv"):
        """Speichert Daten als CSV"""
        if not self. data_history:
            print("?? Keine Daten zum Speichern")
            return
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # Header
            writer.writerow(['Timestamp', 'Group', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
            
            # Daten
            for entry in self.data_history:
                row = [
                    entry['timestamp'],
                    entry['group'],
                    entry['joints']. get('J1', ''),
                    entry['joints']. get('J2', ''),
                    entry['joints'].get('J3', ''),
                    entry['joints'].get('J4', ''),
                    entry['joints'].get('J5', ''),
                    entry['joints']. get('J6', '')
                ]
                writer.writerow(row)
                # HIER DIE ÄNDERUNG: Absoluten Pfad anzeigen
        full_path = os.path.abspath(filename)
        print(f"? CSV gespeichert unter:\n   ?? {full_path}")
        
        
    
    def save_to_json(self, filename="robot_positions.json"):
        """Speichert Daten als JSON"""
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.data_history, f, indent=2, ensure_ascii=False)

        # HIER DIE ÄNDERUNG: Absoluten Pfad anzeigen
        full_path = os.path.abspath(filename)
        print(f"? JSON gespeichert unter:\n   ?? {full_path}")
        
        
    
    def get_statistics(self):
        """Berechnet Statistiken"""
        if not self.data_history:
            return None
        
        stats = {}
        
        # Für jedes Joint
        for joint in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
            values = [entry['joints'].get(joint) for entry in self.data_history 
                     if joint in entry['joints']]
            
            if values:
                stats[joint] = {
                    'min': min(values),
                    'max': max(values),
                    'avg': sum(values) / len(values),
                    'range': max(values) - min(values)
                }
        
        return stats
    
    def print_statistics(self):
        """Zeigt Statistiken an"""
        stats = self.get_statistics()
        
        if not stats:
            print("?? Keine Daten für Statistiken")
            return
        
        print(f"\n{'='*60}")
        print("?? STATISTIKEN")
        print(f"{'='*60}")
        print(f"Anzahl Messungen: {len(self.data_history)}")
        print(f"\n{'Joint':<8} {'Min': >10} {'Max':>10} {'Avg':>10} {'Range':>10}")
        print("-" * 60)
        
        for joint, data in stats.items():
            print(f"{joint:<8} {data['min']:>10.2f} {data['max']:>10.2f} "
                  f"{data['avg']:>10.2f} {data['range']:>10.2f}")
        
        print(f"{'='*60}\n")
    
    def monitor_with_recording(self, duration=60, interval=0.5):
        """
        Überwacht und zeichnet für eine bestimmte Dauer auf
        """
        print(f"?? Starte Aufzeichnung für {duration} Sekunden")
        print(f"??  Intervall: {interval} Sekunden")
        
        start_time = time.time()
        count = 0
        
        try:
            while True:
                data = self.get_current_position()
                
                if data:
                    count += 1
                    # Kompakte Ausgabe
                    joints_str = ", ".join([f"{k}:{v:.2f}" for k, v in data['joints']. items()])
                    
                    # HIER WURDE DER ZEITSTEMPEL HINZUGEFÜGT
                    print(f"#{count} | {data['timestamp']} | {joints_str}")
                
                time.sleep(interval)
            
            print(f"\n? Aufzeichnung beendet:  {count} Messungen")
            
            # Statistiken anzeigen
            self.print_statistics()
            
        except KeyboardInterrupt:
            print(f"\n\n?? Aufzeichnung abgebrochen nach {count} Messungen")
            self.print_statistics()
            self.save_to_csv()
            self.save_to_json()


# ============================================
# VERWENDUNG
# ============================================

if __name__ == "__main__": 
    robot_ip = "192.168.0.11"
    
    scraper = FanucWebScraperAdvanced(robot_ip)

    
    # 30 Sekunden aufzeichnen, alle 0.5 Sekunden
    scraper.monitor_with_recording(duration=30, interval=0.0001)
