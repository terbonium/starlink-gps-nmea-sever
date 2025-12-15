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


def verify_nmea_checksum(sentence: str) -> bool:
    """Verify NMEA sentence checksum"""
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    try:
        # Remove $ prefix and split at *
        body = sentence[1:sentence.index('*')]
        expected_cs = sentence[sentence.index('*') + 1:].strip().upper()
        calculated_cs = calculate_nmea_checksum(body)
        return calculated_cs == expected_cs
    except Exception:
        return False


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two coordinates in meters"""
    R = 6371000  # Earth radius in meters
    lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
    dlat = lat2_rad - lat1_rad
    dlon = math.radians(lon2 - lon1)

    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


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

    # Compass data from Starlink alignment stats
    compass_heading: Optional[float] = None  # Magnetic heading (with offset applied)
    boresight_azimuth: Optional[float] = None  # Raw dish azimuth
    boresight_elevation: Optional[float] = None  # Dish elevation (for tilt calc)
    tilt: Optional[float] = None  # Degrees off vertical (90 - elevation)

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


class NMEAParser:
    """Parses incoming NMEA sentences to extract GPS data"""

    @staticmethod
    def parse_coordinate(value: str, direction: str, is_longitude: bool = False) -> Optional[float]:
        """Parse NMEA coordinate format (DDDMM.MMMM or DDMM.MMMM) to decimal degrees"""
        if not value or not direction:
            return None
        try:
            if is_longitude:
                degrees = int(value[:3])
                minutes = float(value[3:])
            else:
                degrees = int(value[:2])
                minutes = float(value[2:])

            decimal = degrees + minutes / 60.0
            if direction in ('S', 'W'):
                decimal = -decimal
            return decimal
        except (ValueError, IndexError):
            return None

    @staticmethod
    def parse_gpgga(fields: List[str]) -> dict:
        """Parse GPGGA sentence - Global Positioning System Fix Data"""
        result = {}
        if len(fields) < 15:
            return result

        # Fields: time, lat, lat_dir, lon, lon_dir, fix_qual, num_sats, hdop, alt, alt_unit, geoid, geoid_unit, age, station_id
        lat = NMEAParser.parse_coordinate(fields[2], fields[3], is_longitude=False)
        lon = NMEAParser.parse_coordinate(fields[4], fields[5], is_longitude=True)

        if lat is not None:
            result['latitude'] = lat
        if lon is not None:
            result['longitude'] = lon

        try:
            if fields[6]:
                result['fix_quality'] = int(fields[6])
            if fields[7]:
                result['satellites_used'] = int(fields[7])
            if fields[8]:
                result['hdop'] = float(fields[8])
            if fields[9]:
                result['altitude'] = float(fields[9])
            if fields[1]:
                # Parse time HHMMSS.SS
                time_str = fields[1]
                result['time'] = time_str
        except (ValueError, IndexError):
            pass

        return result

    @staticmethod
    def parse_gprmc(fields: List[str]) -> dict:
        """Parse GPRMC sentence - Recommended Minimum Navigation Information"""
        result = {}
        if len(fields) < 12:
            return result

        # Fields: time, status, lat, lat_dir, lon, lon_dir, speed_knots, course, date, mag_var, var_dir, mode
        if fields[2] != 'A':  # A=Active, V=Void
            return result

        lat = NMEAParser.parse_coordinate(fields[3], fields[4], is_longitude=False)
        lon = NMEAParser.parse_coordinate(fields[5], fields[6], is_longitude=True)

        if lat is not None:
            result['latitude'] = lat
        if lon is not None:
            result['longitude'] = lon

        try:
            if fields[7]:
                result['speed_knots'] = float(fields[7])
                result['speed_mps'] = float(fields[7]) / 1.94384
            if fields[8]:
                result['heading'] = float(fields[8])
            if fields[1]:
                result['time'] = fields[1]
            if fields[9]:
                result['date'] = fields[9]
        except (ValueError, IndexError):
            pass

        return result

    @staticmethod
    def parse_gpgsa(fields: List[str]) -> dict:
        """Parse GPGSA sentence - GPS DOP and Active Satellites"""
        result = {}
        if len(fields) < 18:
            return result

        try:
            if fields[2]:
                result['fix_mode'] = int(fields[2])  # 1=no fix, 2=2D, 3=3D
            if fields[15]:
                result['pdop'] = float(fields[15])
            if fields[16]:
                result['hdop'] = float(fields[16])
            if fields[17].split('*')[0]:  # May have checksum attached
                result['vdop'] = float(fields[17].split('*')[0])
        except (ValueError, IndexError):
            pass

        return result

    @staticmethod
    def parse_gpvtg(fields: List[str]) -> dict:
        """Parse GPVTG sentence - Track Made Good and Ground Speed"""
        result = {}
        if len(fields) < 9:
            return result

        try:
            if fields[1]:
                result['heading'] = float(fields[1])  # True heading
            if fields[5]:
                result['speed_knots'] = float(fields[5])
                result['speed_mps'] = float(fields[5]) / 1.94384
            if fields[7]:
                result['speed_kmh'] = float(fields[7])
        except (ValueError, IndexError):
            pass

        return result

    @staticmethod
    def parse_sentence(sentence: str) -> Tuple[str, dict]:
        """Parse any NMEA sentence, return (sentence_type, parsed_data)"""
        if not verify_nmea_checksum(sentence):
            return ('INVALID', {})

        try:
            # Remove $ and checksum
            body = sentence[1:sentence.index('*')]
            fields = body.split(',')
            sentence_type = fields[0].upper()

            if sentence_type == 'GPGGA':
                return (sentence_type, NMEAParser.parse_gpgga(fields))
            elif sentence_type == 'GPRMC':
                return (sentence_type, NMEAParser.parse_gprmc(fields))
            elif sentence_type == 'GPGSA':
                return (sentence_type, NMEAParser.parse_gpgsa(fields))
            elif sentence_type == 'GPVTG':
                return (sentence_type, NMEAParser.parse_gpvtg(fields))
            else:
                return (sentence_type, {})
        except Exception:
            return ('ERROR', {})


class ExternalGPSReceiver:
    """TCP server that receives NMEA data from external GPS devices"""

    def __init__(self, host: str = '0.0.0.0', port: int = 60660):
        self.host = host
        self.port = port
        self.gps_data = GPSData()
        self.lock = threading.Lock()
        self.running = False
        self.connected = False
        self.last_update: Optional[datetime] = None
        self.last_log: Optional[datetime] = None
        self.sentence_count = 0

    def handle_client(self, client_socket: socket.socket, address: tuple):
        """Handle incoming NMEA data from a connected GPS device"""
        logger.info(f"External GPS connected: {address}")
        self.connected = True
        buffer = ""

        try:
            while self.running:
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        break

                    buffer += data.decode('ascii', errors='ignore')

                    # Process complete sentences
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('$'):
                            self._process_sentence(line)

                except socket.timeout:
                    continue
                except Exception as e:
                    logger.warning(f"External GPS receive error: {e}")
                    break

        finally:
            self.connected = False
            client_socket.close()
            logger.info(f"External GPS disconnected: {address}")

    def _process_sentence(self, sentence: str):
        """Process a single NMEA sentence and update GPS data"""
        sentence_type, parsed = NMEAParser.parse_sentence(sentence)

        if not parsed:
            return

        with self.lock:
            if 'latitude' in parsed:
                self.gps_data.latitude = parsed['latitude']
            if 'longitude' in parsed:
                self.gps_data.longitude = parsed['longitude']
            if 'altitude' in parsed:
                self.gps_data.altitude = parsed['altitude']
            if 'speed_mps' in parsed:
                self.gps_data.speed_mps = parsed['speed_mps']
            if 'heading' in parsed:
                self.gps_data.heading = parsed['heading']
            if 'hdop' in parsed:
                self.gps_data.hdop = parsed['hdop']
            if 'vdop' in parsed:
                self.gps_data.vdop = parsed['vdop']
            if 'pdop' in parsed:
                self.gps_data.pdop = parsed['pdop']
            if 'satellites_used' in parsed:
                self.gps_data.satellites_used = parsed['satellites_used']
            if 'fix_quality' in parsed:
                self.gps_data.fix_quality = parsed['fix_quality']

            self.gps_data.timestamp = datetime.now(timezone.utc)
            self.last_update = self.gps_data.timestamp
            self.sentence_count += 1

            # Log external GPS status periodically (every 10 seconds)
            now = datetime.now(timezone.utc)
            if self.last_log is None or (now - self.last_log).total_seconds() > 10:
                if self.gps_data.latitude != 0 or self.gps_data.longitude != 0:
                    logger.info(
                        f"External GPS: {self.gps_data.latitude:.6f}, {self.gps_data.longitude:.6f}, "
                        f"alt={self.gps_data.altitude:.1f}m, hdop={self.gps_data.hdop:.1f}, "
                        f"sats={self.gps_data.satellites_used}, sentences={self.sentence_count}"
                    )
                    self.last_log = now

    def start(self):
        """Start the external GPS receiver server"""
        self.running = True

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)  # Only accept one GPS device connection
        server_socket.settimeout(1.0)

        logger.info(f"External GPS receiver listening on {self.host}:{self.port}")

        try:
            while self.running:
                try:
                    client_socket, address = server_socket.accept()
                    client_socket.settimeout(5.0)
                    # Handle in a separate thread
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, address),
                        daemon=True
                    )
                    client_thread.start()
                except socket.timeout:
                    continue
        finally:
            server_socket.close()

    def get_gps_data(self) -> Optional[GPSData]:
        """Get current GPS data if available and recent (thread-safe)"""
        with self.lock:
            # Return None if no recent data (older than 5 seconds)
            if self.last_update is None:
                return None
            age = (datetime.now(timezone.utc) - self.last_update).total_seconds()
            if age > 5.0:
                return None

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

    def is_available(self) -> bool:
        """Check if external GPS data is available and recent"""
        return self.get_gps_data() is not None

    def stop(self):
        """Stop the receiver"""
        self.running = False


class GPSDataFusion:
    """Fuses GPS data from multiple sources with quality-based weighting"""

    # Deviation thresholds in meters
    THRESHOLD_AVERAGE = 50.0     # Below this: average both sources
    THRESHOLD_PREFER = 200.0    # Below this: prefer better quality source
    # Above THRESHOLD_PREFER: use only best quality source

    def __init__(self):
        self.last_fusion_log: Optional[datetime] = None
        self.current_mode: str = "starlink_only"
        self.current_deviation: Optional[float] = None
        self.starlink_quality: Optional[float] = None
        self.external_quality: Optional[float] = None

    def fuse(self, starlink_data: Optional[GPSData], external_data: Optional[GPSData]) -> Optional[GPSData]:
        """
        Fuse GPS data from Starlink and external sources.

        Returns the best available GPS data based on:
        1. Position deviation between sources
        2. Quality metrics (HDOP, satellites)
        3. Data freshness
        """
        # Handle cases where only one source is available
        if starlink_data is None and external_data is None:
            self.current_mode = "no_data"
            self.current_deviation = None
            return None

        if starlink_data is None:
            self.current_mode = "external_only"
            self.current_deviation = None
            self._log_fusion("external_only", None, None, external_data)
            return external_data

        if external_data is None:
            self.current_mode = "starlink_only"
            self.current_deviation = None
            self._log_fusion("starlink_only", starlink_data, None, None)
            return starlink_data

        # Both sources available - check for valid positions
        starlink_valid = starlink_data.latitude != 0 or starlink_data.longitude != 0
        external_valid = external_data.latitude != 0 or external_data.longitude != 0

        if not starlink_valid and not external_valid:
            self.current_mode = "no_data"
            return None
        if not starlink_valid:
            self.current_mode = "external_only"
            self.current_deviation = None
            self._log_fusion("external_only", None, None, external_data)
            return external_data
        if not external_valid:
            self.current_mode = "starlink_only"
            self.current_deviation = None
            self._log_fusion("starlink_only", starlink_data, None, None)
            return starlink_data

        # Calculate position deviation
        deviation = haversine_distance(
            starlink_data.latitude, starlink_data.longitude,
            external_data.latitude, external_data.longitude
        )
        self.current_deviation = deviation

        # Calculate quality scores (lower HDOP is better, more satellites is better)
        # Score = satellites / hdop (higher is better)
        self.starlink_quality = starlink_data.satellites_used / max(starlink_data.hdop, 0.1)
        self.external_quality = external_data.satellites_used / max(external_data.hdop, 0.1)

        if deviation < self.THRESHOLD_AVERAGE:
            # Small deviation: weighted average based on quality
            self.current_mode = "fused"
            self._log_fusion("fused", starlink_data, external_data, None, deviation)
            return self._weighted_average(starlink_data, external_data, self.starlink_quality, self.external_quality)

        elif deviation < self.THRESHOLD_PREFER:
            # Medium deviation: prefer better quality source but log the discrepancy
            if self.starlink_quality >= self.external_quality:
                self.current_mode = "prefer_starlink"
                self._log_fusion("prefer_starlink", starlink_data, external_data, None, deviation)
                return self._merge_best_of_both(starlink_data, external_data, prefer_starlink=True)
            else:
                self.current_mode = "prefer_external"
                self._log_fusion("prefer_external", starlink_data, external_data, None, deviation)
                return self._merge_best_of_both(starlink_data, external_data, prefer_starlink=False)

        else:
            # Large deviation: use only the best quality source, ignore the other
            if self.starlink_quality >= self.external_quality:
                self.current_mode = "starlink_only_deviation"
                self._log_fusion("starlink_only_deviation", starlink_data, external_data, None, deviation)
                return starlink_data
            else:
                self.current_mode = "external_only_deviation"
                self._log_fusion("external_only_deviation", starlink_data, external_data, None, deviation)
                return external_data

    def get_source_label(self) -> str:
        """Get a human-readable label for current GPS source"""
        labels = {
            "no_data": "NO_FIX",
            "starlink_only": "STARLINK",
            "external_only": "EXTERNAL",
            "fused": "FUSED",
            "prefer_starlink": "STARLINK*",
            "prefer_external": "EXTERNAL*",
            "starlink_only_deviation": "STARLINK!",
            "external_only_deviation": "EXTERNAL!",
        }
        return labels.get(self.current_mode, "UNKNOWN")

    def _weighted_average(self, starlink: GPSData, external: GPSData,
                          starlink_score: float, external_score: float) -> GPSData:
        """Create weighted average of both GPS sources"""
        total_score = starlink_score + external_score
        w_starlink = starlink_score / total_score
        w_external = external_score / total_score

        return GPSData(
            latitude=starlink.latitude * w_starlink + external.latitude * w_external,
            longitude=starlink.longitude * w_starlink + external.longitude * w_external,
            altitude=starlink.altitude * w_starlink + external.altitude * w_external,
            speed_mps=starlink.speed_mps * w_starlink + external.speed_mps * w_external,
            heading=self._average_heading(starlink.heading, external.heading, w_starlink, w_external),
            timestamp=max(starlink.timestamp, external.timestamp),
            # Use the better values for metadata
            compass_heading=starlink.compass_heading,  # Only Starlink has compass
            boresight_azimuth=starlink.boresight_azimuth,
            boresight_elevation=starlink.boresight_elevation,
            tilt=starlink.tilt,
            hdop=min(starlink.hdop, external.hdop),
            vdop=min(starlink.vdop, external.vdop),
            pdop=min(starlink.pdop, external.pdop),
            satellites_used=max(starlink.satellites_used, external.satellites_used),
            fix_quality=max(starlink.fix_quality, external.fix_quality)
        )

    def _merge_best_of_both(self, starlink: GPSData, external: GPSData,
                            prefer_starlink: bool) -> GPSData:
        """Use position from preferred source but keep best metadata from both"""
        primary = starlink if prefer_starlink else external

        return GPSData(
            latitude=primary.latitude,
            longitude=primary.longitude,
            altitude=primary.altitude,
            speed_mps=primary.speed_mps,
            heading=primary.heading,
            timestamp=primary.timestamp,
            # Always use Starlink's compass data
            compass_heading=starlink.compass_heading,
            boresight_azimuth=starlink.boresight_azimuth,
            boresight_elevation=starlink.boresight_elevation,
            tilt=starlink.tilt,
            # Use the better quality indicators
            hdop=min(starlink.hdop, external.hdop),
            vdop=min(starlink.vdop, external.vdop),
            pdop=min(starlink.pdop, external.pdop),
            satellites_used=max(starlink.satellites_used, external.satellites_used),
            fix_quality=max(starlink.fix_quality, external.fix_quality)
        )

    def _average_heading(self, h1: float, h2: float, w1: float, w2: float) -> float:
        """Average two headings using circular mean"""
        # Convert to unit vectors
        sin1, cos1 = math.sin(math.radians(h1)), math.cos(math.radians(h1))
        sin2, cos2 = math.sin(math.radians(h2)), math.cos(math.radians(h2))

        # Weighted average of components
        avg_sin = sin1 * w1 + sin2 * w2
        avg_cos = cos1 * w1 + cos2 * w2

        # Convert back to angle
        return (math.degrees(math.atan2(avg_sin, avg_cos)) + 360) % 360

    def _log_fusion(self, mode: str, starlink: Optional[GPSData], external: Optional[GPSData],
                     result: Optional[GPSData], deviation: Optional[float] = None):
        """Log fusion decisions periodically with detailed source info"""
        now = datetime.now(timezone.utc)
        # Log at most once every 10 seconds
        if self.last_fusion_log is None or (now - self.last_fusion_log).total_seconds() > 10:
            parts = [f"GPS source: {self.get_source_label()}"]

            if deviation is not None:
                parts.append(f"deviation={deviation:.1f}m")

            if starlink is not None:
                parts.append(f"starlink[{starlink.latitude:.6f},{starlink.longitude:.6f} hdop={starlink.hdop:.1f} sats={starlink.satellites_used}]")

            if external is not None:
                parts.append(f"external[{external.latitude:.6f},{external.longitude:.6f} hdop={external.hdop:.1f} sats={external.satellites_used}]")

            logger.info(" | ".join(parts))
            self.last_fusion_log = now


class StarlinkGRPCClient:
    """gRPC client using reflection to communicate with Starlink"""

    def __init__(self, dish_address: str = "192.168.100.1:9200", smoothing_window: int = 6,
                 heading_offset: float = 0.0):
        self.dish_address = dish_address
        self.channel = None
        self.gps_data = GPSData()
        self.gps_data._smoothing_window = smoothing_window
        self.heading_offset = heading_offset  # Static offset: vessel_heading = boresight_azimuth + offset
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

    def _create_status_request(self):
        """Create a GetStatus request using reflection"""
        request = self.request_type()
        request.get_status.SetInParent()
        return request

    def get_status(self) -> dict:
        """Get dish status including alignment stats from Starlink dish"""
        try:
            request = self._create_status_request()

            handle_method = self.channel.unary_unary(
                '/SpaceX.API.Device.Device/Handle',
                request_serializer=lambda x: x.SerializeToString(),
                response_deserializer=self.response_type.FromString,
            )

            response = handle_method(request, timeout=5)
            which_response = response.WhichOneof('response')

            if which_response == 'dish_get_status':
                status = response.dish_get_status

                # Extract alignment stats
                result = {
                    'boresight_azimuth': None,
                    'boresight_elevation': None,
                }

                if hasattr(status, 'alignment_stats'):
                    alignment = status.alignment_stats
                    # These may be strings in some firmware versions
                    if hasattr(alignment, 'boresight_azimuth_deg'):
                        az = alignment.boresight_azimuth_deg
                        result['boresight_azimuth'] = float(az) if az else None
                    if hasattr(alignment, 'boresight_elevation_deg'):
                        el = alignment.boresight_elevation_deg
                        result['boresight_elevation'] = float(el) if el else None

                return result

            return {'boresight_azimuth': None, 'boresight_elevation': None}

        except Exception as e:
            logger.warning(f"Status request error: {e}")
            return {'boresight_azimuth': None, 'boresight_elevation': None}

    def start_polling(self, interval: float = 0.5):
        """Poll location and status data continuously"""
        logger.info(f"Starting GPS polling (interval: {interval}s)")
        if self.heading_offset != 0:
            logger.info(f"Compass heading offset: {self.heading_offset:.1f}°")

        poll_count = 0
        while self.connected:
            try:
                poll_count += 1
                location = self.get_location()
                status = self.get_status()

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

                        # Update compass data from alignment stats
                        self.gps_data.boresight_azimuth = status.get('boresight_azimuth')
                        self.gps_data.boresight_elevation = status.get('boresight_elevation')

                        # Calculate compass heading with offset
                        if self.gps_data.boresight_azimuth is not None:
                            raw_heading = self.gps_data.boresight_azimuth + self.heading_offset
                            self.gps_data.compass_heading = raw_heading % 360

                        # Calculate tilt (degrees off vertical)
                        if self.gps_data.boresight_elevation is not None:
                            self.gps_data.tilt = 90.0 - self.gps_data.boresight_elevation

                    if poll_count <= 3 or poll_count % 20 == 0:
                        compass_str = f", compass={self.gps_data.compass_heading:.1f}°" if self.gps_data.compass_heading is not None else ""
                        tilt_str = f", tilt={self.gps_data.tilt:.1f}°" if self.gps_data.tilt is not None else ""
                        logger.info(
                            f"GPS: {location['lat']:.6f}, {location['lon']:.6f}, "
                            f"alt={location['alt']:.1f}m, speed={self.gps_data.speed_knots:.1f}kts"
                            f"{compass_str}{tilt_str}"
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
                compass_heading=self.gps_data.compass_heading,
                boresight_azimuth=self.gps_data.boresight_azimuth,
                boresight_elevation=self.gps_data.boresight_elevation,
                tilt=self.gps_data.tilt,
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

    def generate_hchdm(self) -> Optional[str]:
        """Generate HDM - Heading Magnetic sentence
        Format: $HCHDM,x.x,M*cs
        """
        if self.gps.compass_heading is None:
            return None

        sentence = f"HCHDM,{self.gps.compass_heading:.1f},M"
        return format_nmea_sentence(sentence)

    def generate_hchdt(self) -> Optional[str]:
        """Generate HDT - Heading True sentence
        Format: $HCHDT,x.x,T*cs
        Note: We calculate true heading by applying magnetic variation to compass heading
        """
        if self.gps.compass_heading is None:
            return None

        # True heading = Magnetic heading - magnetic variation (west positive)
        true_heading = (self.gps.compass_heading - self.magnetic_variation) % 360
        sentence = f"HCHDT,{true_heading:.1f},T"
        return format_nmea_sentence(sentence)

    def generate_hchdg(self) -> Optional[str]:
        """Generate HDG - Heading, Deviation & Variation sentence
        Format: $HCHDG,x.x,x.x,a,x.x,a*cs
        Fields: heading, deviation, dev_dir, variation, var_dir
        """
        if self.gps.compass_heading is None:
            return None

        # We don't have deviation data, so leave it empty
        # Variation: positive = West, negative = East
        var_dir = 'W' if self.magnetic_variation >= 0 else 'E'
        sentence = f"HCHDG,{self.gps.compass_heading:.1f},,,{abs(self.magnetic_variation):.1f},{var_dir}"
        return format_nmea_sentence(sentence)

    def generate_hcxdr_tilt(self) -> Optional[str]:
        """Generate XDR - Transducer Measurement for tilt/pitch
        Format: $HCXDR,A,x.x,D,PTCH*cs
        A = Angular displacement, D = Degrees
        PTCH = Pitch transducer ID
        """
        if self.gps.tilt is None:
            return None

        # Tilt is reported as degrees off vertical (positive = tilted)
        sentence = f"HCXDR,A,{self.gps.tilt:.1f},D,PTCH"
        return format_nmea_sentence(sentence)

    def generate_all(self) -> List[str]:
        sentences = []
        sentences.extend(self.generate_gpgsv())
        sentences.append(self.generate_gpgga())
        sentences.append(self.generate_gpgll())
        sentences.append(self.generate_gpvtg())
        sentences.append(self.generate_gprmc())
        sentences.append(self.generate_gpgsa())

        # Compass heading sentences (if available)
        hdm = self.generate_hchdm()
        if hdm:
            sentences.append(hdm)
        hdt = self.generate_hchdt()
        if hdt:
            sentences.append(hdt)
        hdg = self.generate_hchdg()
        if hdg:
            sentences.append(hdg)

        # Tilt sensor data (if available)
        xdr = self.generate_hcxdr_tilt()
        if xdr:
            sentences.append(xdr)

        return sentences


class NMEATCPServer:
    """TCP server that broadcasts NMEA messages"""

    def __init__(self, starlink_client: StarlinkGRPCClient,
                 host: str = '0.0.0.0', port: int = 10110,
                 update_rate: float = 1.0,
                 external_gps: Optional[ExternalGPSReceiver] = None):
        self.starlink = starlink_client
        self.external_gps = external_gps
        self.fusion = GPSDataFusion() if external_gps else None
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
        broadcast_count = 0
        while self.running:
            broadcast_count += 1
            # Get GPS data from available sources
            starlink_data = self.starlink.get_gps_data()

            # If we have external GPS and fusion enabled, fuse the data
            if self.fusion and self.external_gps:
                external_data = self.external_gps.get_gps_data()
                gps_data = self.fusion.fuse(starlink_data, external_data)
                source_label = self.fusion.get_source_label()
                deviation = self.fusion.current_deviation
            else:
                gps_data = starlink_data
                source_label = "STARLINK"
                deviation = None

            # Only send if we have valid GPS data
            if gps_data and (gps_data.latitude != 0 or gps_data.longitude != 0):
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

                # Log GPS status periodically (every 10 broadcasts, ~10 seconds at 1Hz)
                if broadcast_count <= 3 or broadcast_count % 10 == 0:
                    compass_str = f", compass={gps_data.compass_heading:.1f}°" if gps_data.compass_heading is not None else ""
                    tilt_str = f", tilt={gps_data.tilt:.1f}°" if gps_data.tilt is not None else ""
                    deviation_str = f", dev={deviation:.1f}m" if deviation is not None else ""
                    logger.info(
                        f"GPS [{source_label}]: {gps_data.latitude:.6f}, {gps_data.longitude:.6f}, "
                        f"alt={gps_data.altitude:.1f}m, speed={gps_data.speed_knots:.1f}kts"
                        f"{compass_str}{tilt_str}{deviation_str}"
                    )

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
    parser.add_argument('--heading-offset', type=float, default=0.0,
                        help='Static compass heading offset in degrees (vessel_heading = dish_azimuth + offset)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')

    # External GPS options
    parser.add_argument('--external-gps-port', type=int, default=0,
                        help='Port to listen for external GPS NMEA data (0 = disabled, default: 0)')
    parser.add_argument('--external-gps-host', default='0.0.0.0',
                        help='Host to listen for external GPS NMEA data (default: 0.0.0.0)')

    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    logger.info(f"Speed/heading smoothing: {args.smoothing} samples")
    if args.heading_offset != 0:
        logger.info(f"Compass heading offset: {args.heading_offset}°")

    starlink = StarlinkGRPCClient(
        dish_address=args.dish,
        smoothing_window=args.smoothing,
        heading_offset=args.heading_offset
    )

    external_gps = None

    try:
        starlink.connect()

        # Start GPS polling in background
        poll_thread = threading.Thread(
            target=starlink.start_polling,
            args=(args.poll_interval,),
            daemon=True
        )
        poll_thread.start()

        # Start external GPS receiver if enabled
        if args.external_gps_port > 0:
            external_gps = ExternalGPSReceiver(
                host=args.external_gps_host,
                port=args.external_gps_port
            )
            external_gps_thread = threading.Thread(
                target=external_gps.start,
                daemon=True
            )
            external_gps_thread.start()
            logger.info(f"External GPS fusion enabled - listening on port {args.external_gps_port}")
            logger.info("Fusion thresholds: <50m=average, 50-200m=prefer better, >200m=use best only")

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
            update_rate=args.rate,
            external_gps=external_gps
        )
        server.start()

    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        raise
    finally:
        starlink.disconnect()
        if external_gps:
            external_gps.stop()


if __name__ == '__main__':
    main()
