#!/usr/bin/env python3
"""
NMEA GPS Message Generator and TCP Server
Simulates Starlink GPS NMEA output over TCP
"""

import socket
import threading
import time
import random
import math
from datetime import datetime, timezone
from typing import List, Tuple


def calculate_nmea_checksum(sentence: str) -> str:
    """Calculate NMEA checksum (XOR of all characters between $ and *)"""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"


def format_nmea_sentence(sentence: str) -> str:
    """Format a complete NMEA sentence with $ prefix and checksum"""
    checksum = calculate_nmea_checksum(sentence)
    return f"${sentence}*{checksum}"


class SatelliteSimulator:
    """Simulates GPS satellite positions and signal strengths"""
    
    def __init__(self, num_satellites: int = 14):
        self.satellites = []
        self.num_satellites = num_satellites
        self._generate_satellites()
    
    def _generate_satellites(self):
        """Generate initial satellite data"""
        # Common GPS satellite PRN numbers
        prns = [5, 6, 11, 12, 13, 15, 18, 20, 21, 23, 25, 29, 46, 48]
        random.shuffle(prns)
        
        for i in range(self.num_satellites):
            self.satellites.append({
                'prn': prns[i % len(prns)],
                'elevation': random.randint(5, 85),
                'azimuth': random.randint(0, 359),
                'snr': random.randint(33, 50),  # Signal-to-noise ratio
                'in_use': i < 11  # First 11 satellites are in use
            })
    
    def update(self):
        """Update satellite positions slightly for realism"""
        for sat in self.satellites:
            # Slowly drift azimuth
            sat['azimuth'] = (sat['azimuth'] + random.randint(-1, 1)) % 360
            # Slowly drift elevation
            sat['elevation'] = max(5, min(85, sat['elevation'] + random.randint(-1, 1)))
            # SNR fluctuates
            sat['snr'] = max(30, min(50, sat['snr'] + random.randint(-2, 2)))


class GPSSimulator:
    """Simulates GPS position and movement"""
    
    def __init__(self, lat: float = 26.90622325, lon: float = -82.57817535):
        self.latitude = lat  # Degrees
        self.longitude = lon  # Degrees
        self.altitude = 1.4  # Meters
        self.speed_knots = 10.0  # Knots
        self.course = 139.3  # Degrees true
        self.magnetic_variation = 3.0  # Degrees West
        self.hdop = 0.9
        self.vdop = 1.6
        self.pdop = 1.8
        self.geoid_separation = -22.0  # Meters
        
        self.satellite_sim = SatelliteSimulator()
    
    def update(self):
        """Update position based on speed and course"""
        # Convert speed to degrees per second (approximate)
        speed_deg_per_sec = (self.speed_knots * 1.852) / 111000 / 3600
        
        # Update position
        self.latitude += speed_deg_per_sec * math.cos(math.radians(self.course))
        self.longitude += speed_deg_per_sec * math.sin(math.radians(self.course))
        
        # Add small random variations
        self.speed_knots = max(0, self.speed_knots + random.uniform(-0.5, 0.5))
        self.course = (self.course + random.uniform(-2, 2)) % 360
        
        self.satellite_sim.update()
    
    def format_lat(self) -> Tuple[str, str]:
        """Format latitude as NMEA (DDMM.MMMMMM,N/S)"""
        direction = 'N' if self.latitude >= 0 else 'S'
        lat_abs = abs(self.latitude)
        degrees = int(lat_abs)
        minutes = (lat_abs - degrees) * 60
        return f"{degrees:02d}{minutes:09.6f}", direction
    
    def format_lon(self) -> Tuple[str, str]:
        """Format longitude as NMEA (DDDMM.MMMMMM,E/W)"""
        direction = 'E' if self.longitude >= 0 else 'W'
        lon_abs = abs(self.longitude)
        degrees = int(lon_abs)
        minutes = (lon_abs - degrees) * 60
        return f"{degrees:03d}{minutes:09.6f}", direction
    
    def get_utc_time(self) -> str:
        """Get current UTC time in NMEA format (HHMMSS.SS)"""
        now = datetime.now(timezone.utc)
        return now.strftime("%H%M%S.00")
    
    def get_utc_date(self) -> str:
        """Get current UTC date in NMEA format (DDMMYY)"""
        now = datetime.now(timezone.utc)
        return now.strftime("%d%m%y")


class NMEAGenerator:
    """Generates NMEA sentences from GPS simulator data"""
    
    def __init__(self, gps: GPSSimulator):
        self.gps = gps
    
    def generate_gpgsv(self) -> List[str]:
        """Generate GPGSV (Satellites in View) sentences"""
        sentences = []
        sats = self.gps.satellite_sim.satellites
        total_sats = len(sats)
        total_msgs = (total_sats + 3) // 4  # 4 satellites per message
        
        for msg_num in range(1, total_msgs + 1):
            start_idx = (msg_num - 1) * 4
            end_idx = min(start_idx + 4, total_sats)
            
            parts = [f"GPGSV,{total_msgs},{msg_num},{total_sats:02d}"]
            
            for i in range(start_idx, end_idx):
                sat = sats[i]
                # Some satellites might not have azimuth/elevation (like SBAS)
                if sat['prn'] >= 46:
                    parts.append(f"{sat['prn']:02d},,,{sat['snr']:02d}")
                else:
                    parts.append(f"{sat['prn']:02d},{sat['elevation']:02d},{sat['azimuth']:03d},{sat['snr']:02d}")
            
            # Add signal ID (1 = L1 C/A)
            sentence_body = ','.join(parts) + ",1"
            sentences.append(format_nmea_sentence(sentence_body))
        
        return sentences
    
    def generate_gpgga(self) -> str:
        """Generate GPGGA (Fix Data) sentence"""
        utc_time = self.gps.get_utc_time()
        lat, lat_dir = self.gps.format_lat()
        lon, lon_dir = self.gps.format_lon()
        
        # Count satellites in use
        sats_in_use = sum(1 for s in self.gps.satellite_sim.satellites if s['in_use'])
        
        sentence = (
            f"GPGGA,{utc_time},{lat},{lat_dir},{lon},{lon_dir},"
            f"1,{sats_in_use:02d},{self.gps.hdop:.1f},{self.gps.altitude:.1f},M,"
            f"{self.gps.geoid_separation:.1f},M,,"
        )
        return format_nmea_sentence(sentence)
    
    def generate_gpgll(self) -> str:
        """Generate GPGLL (Geographic Position) sentence"""
        utc_time = self.gps.get_utc_time()
        lat, lat_dir = self.gps.format_lat()
        lon, lon_dir = self.gps.format_lon()
        
        sentence = f"GPGLL,{lat},{lat_dir},{lon},{lon_dir},{utc_time},A,A"
        return format_nmea_sentence(sentence)
    
    def generate_gpvtg(self) -> str:
        """Generate GPVTG (Track and Ground Speed) sentence"""
        magnetic_course = (self.gps.course + self.gps.magnetic_variation) % 360
        speed_kmh = self.gps.speed_knots * 1.852
        
        sentence = (
            f"GPVTG,{self.gps.course:.1f},T,{magnetic_course:.1f},M,"
            f"{self.gps.speed_knots:.1f},N,{speed_kmh:.1f},K,A"
        )
        return format_nmea_sentence(sentence)
    
    def generate_gprmc(self) -> str:
        """Generate GPRMC (Recommended Minimum) sentence"""
        utc_time = self.gps.get_utc_time()
        utc_date = self.gps.get_utc_date()
        lat, lat_dir = self.gps.format_lat()
        lon, lon_dir = self.gps.format_lon()
        
        mag_var_dir = 'W' if self.gps.magnetic_variation >= 0 else 'E'
        
        sentence = (
            f"GPRMC,{utc_time},A,{lat},{lat_dir},{lon},{lon_dir},"
            f"{self.gps.speed_knots:.1f},{self.gps.course:.1f},{utc_date},"
            f"{abs(self.gps.magnetic_variation):.1f},{mag_var_dir},A,V"
        )
        return format_nmea_sentence(sentence)
    
    def generate_gpgsa(self) -> str:
        """Generate GPGSA (DOP and Active Satellites) sentence"""
        # Get PRNs of satellites in use
        active_prns = [s['prn'] for s in self.gps.satellite_sim.satellites if s['in_use']]
        
        # Pad to 12 slots
        prn_fields = [f"{prn:02d}" if i < len(active_prns) else "" 
                      for i, prn in enumerate(active_prns[:12] + [0] * 12)][:12]
        
        sentence = (
            f"GPGSA,A,3,{','.join(prn_fields)},"
            f"{self.gps.pdop:.1f},{self.gps.hdop:.1f},{self.gps.vdop:.1f},1"
        )
        return format_nmea_sentence(sentence)
    
    def generate_all(self) -> List[str]:
        """Generate all NMEA sentences in order"""
        sentences = []
        sentences.extend(self.generate_gpgsv())
        sentences.append(self.generate_gpgga())
        sentences.append(self.generate_gpgll())
        sentences.append(self.generate_gpvtg())
        sentences.append(self.generate_gprmc())
        sentences.append(self.generate_gpgsa())
        return sentences


class NMEATCPServer:
    """TCP server that broadcasts NMEA messages to connected clients"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 10110, update_rate: float = 1.0):
        self.host = host
        self.port = port
        self.update_rate = update_rate
        self.clients: List[socket.socket] = []
        self.clients_lock = threading.Lock()
        self.running = False
        
        self.gps = GPSSimulator()
        self.nmea_gen = NMEAGenerator(self.gps)
    
    def handle_client(self, client_socket: socket.socket, address: tuple):
        """Handle a new client connection"""
        print(f"New client connected: {address}")
        with self.clients_lock:
            self.clients.append(client_socket)
        
        try:
            while self.running:
                # Just keep connection alive, data is sent by broadcast thread
                time.sleep(1)
        except Exception as e:
            print(f"Client {address} error: {e}")
        finally:
            with self.clients_lock:
                if client_socket in self.clients:
                    self.clients.remove(client_socket)
            client_socket.close()
            print(f"Client disconnected: {address}")
    
    def broadcast_nmea(self):
        """Broadcast NMEA messages to all connected clients"""
        while self.running:
            self.gps.update()
            sentences = self.nmea_gen.generate_all()
            data = '\r\n'.join(sentences) + '\r\n'
            
            with self.clients_lock:
                disconnected = []
                for client in self.clients:
                    try:
                        client.sendall(data.encode('ascii'))
                    except Exception:
                        disconnected.append(client)
                
                for client in disconnected:
                    self.clients.remove(client)
                    try:
                        client.close()
                    except Exception:
                        pass
            
            time.sleep(self.update_rate)
    
    def start(self):
        """Start the TCP server"""
        self.running = True
        
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        server_socket.settimeout(1.0)
        
        print(f"NMEA TCP Server started on {self.host}:{self.port}")
        print(f"Update rate: {self.update_rate} seconds")
        
        # Start broadcast thread
        broadcast_thread = threading.Thread(target=self.broadcast_nmea, daemon=True)
        broadcast_thread.start()
        
        try:
            while self.running:
                try:
                    client_socket, address = server_socket.accept()
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, address),
                        daemon=True
                    )
                    client_thread.start()
                except socket.timeout:
                    continue
        except KeyboardInterrupt:
            print("\nShutting down server...")
        finally:
            self.running = False
            server_socket.close()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='NMEA GPS TCP Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=10110, help='Port to listen on')
    parser.add_argument('--rate', type=float, default=1.0, help='Update rate in seconds')
    parser.add_argument('--lat', type=float, default=26.906223, help='Initial latitude')
    parser.add_argument('--lon', type=float, default=-82.578175, help='Initial longitude')
    
    args = parser.parse_args()
    
    server = NMEATCPServer(host=args.host, port=args.port, update_rate=args.rate)
    server.gps.latitude = args.lat
    server.gps.longitude = args.lon
    
    server.start()


if __name__ == '__main__':
    main()
