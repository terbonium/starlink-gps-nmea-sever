#!/usr/bin/env python3
"""
Starlink GPS to NMEA TCP Server
Uses gRPC reflection to properly communicate with Starlink dish
"""

import socket
import threading
import time
import math
import logging
from datetime import datetime, timezone
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

import grpc
from grpc_reflection.v1alpha.proto_reflection_descriptor_database import ProtoReflectionDescriptorDatabase
from google.protobuf.descriptor_pool import DescriptorPool

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def calculate_nmea_checksum(sentence: str) -> str:
    """Calculate NMEA checksum"""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"


def format_nmea_sentence(sentence: str) -> str:
    """Format a complete NMEA sentence"""
    checksum = calculate_nmea_checksum(sentence)
    return f"${sentence}*{checksum}"


@dataclass
class GPSData:
    """Holds current GPS data from Starlink"""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    speed_mps: float = 0.0
    heading: float = 0.0
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    
    hdop: float = 0.9
    vdop: float = 1.6
    pdop: float = 1.8
    satellites_used: int = 11
    fix_quality: int = 1
    
    _prev_lat: float = field(default=0.0, repr=False)
    _prev_lon: float = field(default=0.0, repr=False)
    _prev_time: Optional[datetime] = field(default=None, repr=False)
    
    # Smoothing history
    _speed_history: List[float] = field(default_factory=list, repr=False)
    _heading_history: List[Tuple[float, float]] = field(default_factory=list, repr=False)  # Store as (sin, cos) for circular mean
    _smoothing_window: int = field(default=6, repr=False)  # Number of samples to average (at 0.5s poll = 3 seconds)
    
    def update_velocity(self):
        """Calculate speed and heading from position changes with smoothing"""
        if self._prev_time is not None and self._prev_lat != 0:
            dt = (self.timestamp - self._prev_time).total_seconds()
            if dt > 0.1:  # Minimum time delta
                R = 6371000  # Earth radius meters
                lat1, lat2 = math.radians(self._prev_lat), math.radians(self.latitude)
                dlat = lat2 - lat1
                dlon = math.radians(self.longitude - self._prev_lon)
                
                # Haversine distance
                a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                distance = R * c
                
                instant_speed = distance / dt
                
                # Heading calculation
                y = math.sin(dlon) * math.cos(lat2)
                x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
                instant_heading = (math.degrees(math.atan2(y, x)) + 360) % 360
                
                # Add to history for smoothing
                self._speed_history.append(instant_speed)
                # Store heading as sin/cos for proper circular averaging
                heading_rad = math.radians(instant_heading)
                self._heading_history.append((math.sin(heading_rad), math.cos(heading_rad)))
                
                # Trim history to window size
                if len(self._speed_history) > self._smoothing_window:
                    self._speed_history = self._speed_history[-self._smoothing_window:]
                if len(self._heading_history) > self._smoothing_window:
                    self._heading_history = self._heading_history[-self._smoothing_window:]
                
                # Calculate smoothed speed (simple average)
                self.speed_mps = sum(self._speed_history) / len(self._speed_history)
                
                # Calculate smoothed heading (circular mean)
                if self._heading_history:
                    avg_sin = sum(h[0] for h in self._heading_history) / len(self._heading_history)
                    avg_cos = sum(h[1] for h in self._heading_history) / len(self._heading_history)
                    self.heading = (math.degrees(math.atan2(avg_sin, avg_cos)) + 360) % 360
        
        self._prev_lat = self.latitude
        self._prev_lon = self.longitude
        self._prev_time = self.timestamp
    
    @property
    def speed_knots(self) -> float:
        return self.speed_mps * 1.94384
    
    @property
    def speed_kmh(self) -> float:
        return self.speed_mps * 3.6


class StarlinkGRPCClient:
    """gRPC client using reflection to communicate with Starlink"""
    
    def __init__(self, dish_address: str = "192.168.100.1:9200", smoothing_window: int = 6):
        self.dish_address = dish_address
        self.channel = None
        self.gps_data = GPSData()
        self.gps_data._smoothing_window = smoothing_window
        self.connected = False
        self.lock = threading.Lock()
        
        # Protobuf reflection components
        self.reflection_db = None
        self.desc_pool = None
        self.factory = None
        self.request_type = None
        self.response_type = None
        
    def connect(self):
        """Establish gRPC connection and set up reflection"""
        try:
            self.channel = grpc.insecure_channel(
                self.dish_address,
                options=[
                    ('grpc.keepalive_time_ms', 10000),
                    ('grpc.keepalive_timeout_ms', 5000),
                ]
            )
            
            # Wait for channel ready
            grpc.channel_ready_future(self.channel).result(timeout=10)
            logger.info(f"Connected to Starlink at {self.dish_address}")
            
            # Set up reflection
            self._setup_reflection()
            
            self.connected = True
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            raise
    
    def _setup_reflection(self):
        """Set up protobuf reflection to discover message types"""
        try:
            self.reflection_db = ProtoReflectionDescriptorDatabase(self.channel)
            self.desc_pool = DescriptorPool(self.reflection_db)
            
            # Get message descriptors
            request_desc = self.desc_pool.FindMessageTypeByName('SpaceX.API.Device.Request')
            response_desc = self.desc_pool.FindMessageTypeByName('SpaceX.API.Device.Response')
            
            # Use the appropriate API based on protobuf version
            from google.protobuf import message_factory as mf
            
            # protobuf 5.x+ uses GetMessageClass
            if hasattr(mf, 'GetMessageClass'):
                self.request_type = mf.GetMessageClass(request_desc)
                self.response_type = mf.GetMessageClass(response_desc)
            else:
                # Older protobuf uses MessageFactory
                from google.protobuf.message_factory import MessageFactory
                factory = MessageFactory(pool=self.desc_pool)
                self.request_type = factory.GetPrototype(request_desc)
                self.response_type = factory.GetPrototype(response_desc)
            
            logger.info("gRPC reflection initialized successfully")
            logger.info(f"Request type: {self.request_type.DESCRIPTOR.full_name}")
            logger.info(f"Response type: {self.response_type.DESCRIPTOR.full_name}")
            
        except Exception as e:
            logger.error(f"Reflection setup failed: {e}")
            raise
    
    def _create_location_request(self):
        """Create a GetLocation request using reflection"""
        request = self.request_type()
        request.get_location.SetInParent()
        return request
    
    def get_location(self) -> dict:
        """Get location from Starlink dish"""
        try:
            # Create the request
            request = self._create_location_request()
            
            # Make the RPC call
            handle_method = self.channel.unary_unary(
                '/SpaceX.API.Device.Device/Handle',
                request_serializer=lambda x: x.SerializeToString(),
                response_deserializer=self.response_type.FromString,
            )
            
            response = handle_method(request, timeout=5)
            
            # Check which oneof field is set
            which_response = response.WhichOneof('response')
            
            # Response field is 'get_location' (not 'dish_get_location')
            if which_response == 'get_location':
                loc = response.get_location
                
                # Check for lla field
                if loc.HasField('lla'):
                    result = {
                        'lat': loc.lla.lat,
                        'lon': loc.lla.lon,
                        'alt': loc.lla.alt,
                        'speed_mps': getattr(loc, 'horizontal_speed_mps', 0.0),
                    }
                    return result
            
            return {'lat': 0.0, 'lon': 0.0, 'alt': 0.0, 'speed_mps': 0.0}
            
        except Exception as e:
            logger.warning(f"Location request error: {e}")
            return {'lat': 0.0, 'lon': 0.0, 'alt': 0.0, 'speed_mps': 0.0}
    
    def start_polling(self, interval: float = 0.5):
        """Poll location data continuously"""
        logger.info(f"Starting GPS polling (interval: {interval}s)")
        
        poll_count = 0
        while self.connected:
            try:
                poll_count += 1
                location = self.get_location()
                
                if location['lat'] != 0 or location['lon'] != 0:
                    with self.lock:
                        self.gps_data.latitude = location['lat']
                        self.gps_data.longitude = location['lon']
                        self.gps_data.altitude = location['alt']
                        self.gps_data.timestamp = datetime.now(timezone.utc)
                        
                        # Use speed from Starlink if available
                        if location.get('speed_mps', 0) > 0:
                            self.gps_data.speed_mps = location['speed_mps']
                        
                        # Still update heading from position changes
                        self.gps_data.update_velocity()
                    
                    if poll_count <= 3 or poll_count % 20 == 0:
                        logger.info(
                            f"GPS: {location['lat']:.6f}, {location['lon']:.6f}, "
                            f"alt={location['alt']:.1f}m, speed={self.gps_data.speed_knots:.1f}kts"
                        )
                    
            except Exception as e:
                logger.warning(f"Polling error: {e}")
            
            time.sleep(interval)
    
    def get_gps_data(self) -> GPSData:
        """Get current GPS data (thread-safe)"""
        with self.lock:
            return GPSData(
                latitude=self.gps_data.latitude,
                longitude=self.gps_data.longitude,
                altitude=self.gps_data.altitude,
                speed_mps=self.gps_data.speed_mps,
                heading=self.gps_data.heading,
                timestamp=self.gps_data.timestamp,
                hdop=self.gps_data.hdop,
                vdop=self.gps_data.vdop,
                pdop=self.gps_data.pdop,
                satellites_used=self.gps_data.satellites_used,
                fix_quality=self.gps_data.fix_quality
            )
    
    def disconnect(self):
        """Close connection"""
        self.connected = False
        if self.channel:
            self.channel.close()
        logger.info("Disconnected from Starlink")


class NMEAGenerator:
    """Generates NMEA sentences from GPS data"""
    
    def __init__(self, gps_data: GPSData):
        self.gps = gps_data
        self.magnetic_variation = 3.0
        self.geoid_separation = -22.0
        self.satellites = self._generate_satellites()
    
    def _generate_satellites(self) -> List[dict]:
        """Generate simulated satellite data"""
        prns = [5, 6, 11, 12, 13, 15, 18, 20, 21, 23, 25, 29, 46, 48]
        satellites = []
        for i, prn in enumerate(prns):
            satellites.append({
                'prn': prn,
                'elevation': 30 + (i * 5) % 60,
                'azimuth': (i * 30) % 360,
                'snr': 40 + (i % 10),
                'in_use': i < 11
            })
        return satellites
    
    def format_lat(self) -> Tuple[str, str]:
        direction = 'N' if self.gps.latitude >= 0 else 'S'
        lat_abs = abs(self.gps.latitude)
        degrees = int(lat_abs)
        minutes = (lat_abs - degrees) * 60
        return f"{degrees:02d}{minutes:09.6f}", direction
    
    def format_lon(self) -> Tuple[str, str]:
        direction = 'E' if self.gps.longitude >= 0 else 'W'
        lon_abs = abs(self.gps.longitude)
        degrees = int(lon_abs)
        minutes = (lon_abs - degrees) * 60
        return f"{degrees:03d}{minutes:09.6f}", direction
    
    def get_utc_time(self) -> str:
        return self.gps.timestamp.strftime("%H%M%S.00")
    
    def get_utc_date(self) -> str:
        return self.gps.timestamp.strftime("%d%m%y")
    
    def generate_gpgsv(self) -> List[str]:
        sentences = []
        total_sats = len(self.satellites)
        total_msgs = (total_sats + 3) // 4
        
        for msg_num in range(1, total_msgs + 1):
            start_idx = (msg_num - 1) * 4
            end_idx = min(start_idx + 4, total_sats)
            
            parts = [f"GPGSV,{total_msgs},{msg_num},{total_sats:02d}"]
            
            for i in range(start_idx, end_idx):
                sat = self.satellites[i]
                if sat['prn'] >= 46:
                    parts.append(f"{sat['prn']:02d},,,{sat['snr']:02d}")
                else:
                    parts.append(f"{sat['prn']:02d},{sat['elevation']:02d},{sat['azimuth']:03d},{sat['snr']:02d}")
            
            sentence_body = ','.join(parts) + ",1"
            sentences.append(format_nmea_sentence(sentence_body))
        
        return sentences
    
    def generate_gpgga(self) -> str:
        lat, lat_dir = self.format_lat()
        lon, lon_dir = self.format_lon()
        
        sentence = (
            f"GPGGA,{self.get_utc_time()},{lat},{lat_dir},{lon},{lon_dir},"
            f"{self.gps.fix_quality},{self.gps.satellites_used:02d},{self.gps.hdop:.1f},"
            f"{self.gps.altitude:.1f},M,{self.geoid_separation:.1f},M,,"
        )
        return format_nmea_sentence(sentence)
    
    def generate_gpgll(self) -> str:
        lat, lat_dir = self.format_lat()
        lon, lon_dir = self.format_lon()
        
        sentence = f"GPGLL,{lat},{lat_dir},{lon},{lon_dir},{self.get_utc_time()},A,A"
        return format_nmea_sentence(sentence)
    
    def generate_gpvtg(self) -> str:
        magnetic_course = (self.gps.heading + self.magnetic_variation) % 360
        
        sentence = (
            f"GPVTG,{self.gps.heading:.1f},T,{magnetic_course:.1f},M,"
            f"{self.gps.speed_knots:.1f},N,{self.gps.speed_kmh:.1f},K,A"
        )
        return format_nmea_sentence(sentence)
    
    def generate_gprmc(self) -> str:
        lat, lat_dir = self.format_lat()
        lon, lon_dir = self.format_lon()
        mag_var_dir = 'W' if self.magnetic_variation >= 0 else 'E'
        
        sentence = (
            f"GPRMC,{self.get_utc_time()},A,{lat},{lat_dir},{lon},{lon_dir},"
            f"{self.gps.speed_knots:.1f},{self.gps.heading:.1f},{self.get_utc_date()},"
            f"{abs(self.magnetic_variation):.1f},{mag_var_dir},A,V"
        )
        return format_nmea_sentence(sentence)
    
    def generate_gpgsa(self) -> str:
        active_prns = [s['prn'] for s in self.satellites if s['in_use']]
        prn_fields = [f"{prn:02d}" if i < len(active_prns) else "" 
                      for i, prn in enumerate(active_prns[:12] + [0] * 12)][:12]
        
        sentence = (
            f"GPGSA,A,3,{','.join(prn_fields)},"
            f"{self.gps.pdop:.1f},{self.gps.hdop:.1f},{self.gps.vdop:.1f},1"
        )
        return format_nmea_sentence(sentence)
    
    def generate_all(self) -> List[str]:
        sentences = []
        sentences.extend(self.generate_gpgsv())
        sentences.append(self.generate_gpgga())
        sentences.append(self.generate_gpgll())
        sentences.append(self.generate_gpvtg())
        sentences.append(self.generate_gprmc())
        sentences.append(self.generate_gpgsa())
        return sentences


class NMEATCPServer:
    """TCP server that broadcasts NMEA messages"""
    
    def __init__(self, starlink_client: StarlinkGRPCClient,
                 host: str = '0.0.0.0', port: int = 10110,
                 update_rate: float = 1.0):
        self.starlink = starlink_client
        self.host = host
        self.port = port
        self.update_rate = update_rate
        self.clients: List[socket.socket] = []
        self.clients_lock = threading.Lock()
        self.running = False
    
    def handle_client(self, client_socket: socket.socket, address: tuple):
        logger.info(f"Client connected: {address}")
        with self.clients_lock:
            self.clients.append(client_socket)
        
        try:
            while self.running:
                time.sleep(1)
        except Exception as e:
            logger.error(f"Client {address} error: {e}")
        finally:
            with self.clients_lock:
                if client_socket in self.clients:
                    self.clients.remove(client_socket)
            client_socket.close()
            logger.info(f"Client disconnected: {address}")
    
    def broadcast_nmea(self):
        """Broadcast NMEA to all clients"""
        while self.running:
            gps_data = self.starlink.get_gps_data()
            
            # Only send if we have valid GPS data
            if gps_data.latitude != 0 or gps_data.longitude != 0:
                nmea_gen = NMEAGenerator(gps_data)
                sentences = nmea_gen.generate_all()
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
                
                if logger.isEnabledFor(logging.DEBUG):
                    logger.debug(f"Sent {len(sentences)} NMEA sentences to {len(self.clients)} clients")
            else:
                logger.debug("Waiting for valid GPS data...")
            
            time.sleep(self.update_rate)
    
    def start(self):
        self.running = True
        
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        server_socket.settimeout(1.0)
        
        logger.info(f"NMEA TCP Server started on {self.host}:{self.port}")
        logger.info(f"Update rate: {self.update_rate}s")
        
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
            logger.info("Shutting down...")
        finally:
            self.running = False
            server_socket.close()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Starlink GPS to NMEA TCP Server')
    parser.add_argument('--dish', default='192.168.100.1:9200',
                        help='Starlink dish gRPC address')
    parser.add_argument('--host', default='0.0.0.0',
                        help='TCP server host')
    parser.add_argument('--port', type=int, default=10110,
                        help='TCP server port')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='NMEA output rate in seconds')
    parser.add_argument('--poll-interval', type=float, default=0.5,
                        help='GPS polling interval in seconds')
    parser.add_argument('--smoothing', type=int, default=6,
                        help='Number of samples to average for speed/heading smoothing (default: 6)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    logger.info(f"Speed/heading smoothing: {args.smoothing} samples")
    starlink = StarlinkGRPCClient(dish_address=args.dish, smoothing_window=args.smoothing)
    
    try:
        starlink.connect()
        
        # Start GPS polling in background
        poll_thread = threading.Thread(
            target=starlink.start_polling,
            args=(args.poll_interval,),
            daemon=True
        )
        poll_thread.start()
        
        # Wait for first fix
        logger.info("Waiting for GPS fix...")
        for i in range(10):
            time.sleep(1)
            data = starlink.get_gps_data()
            if data.latitude != 0 or data.longitude != 0:
                logger.info(f"GPS fix acquired: {data.latitude:.6f}, {data.longitude:.6f}")
                break
        else:
            logger.warning("No GPS fix after 10 seconds, starting anyway...")
        
        # Start NMEA server
        server = NMEATCPServer(
            starlink, 
            host=args.host, 
            port=args.port,
            update_rate=args.rate
        )
        server.start()
        
    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        raise
    finally:
        starlink.disconnect()


if __name__ == '__main__':
    main()
