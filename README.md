# Starlink GPS to NMEA TCP Server

Connects to your Starlink dish via gRPC and serves real-time GPS data as NMEA 0183 sentences over TCP.

## Features

- **Real-time GPS data** from Starlink dish via gRPC reflection API
- **Compass heading** from Starlink dish alignment stats (with configurable offset for dish-to-vessel alignment)
- **Tilt sensor data** derived from dish boresight elevation
- **Standard NMEA 0183 output** compatible with chart plotters, autopilots, and navigation software
- **Automatic velocity calculation** (speed and heading derived from position changes)
- **Configurable update rates** for both GPS polling and NMEA output
- **Multi-client support** - multiple devices can connect simultaneously

## Prerequisites

### 1. Enable Starlink Location Sharing

You must enable GPS location sharing in the **Starlink mobile app**:

1. Open the Starlink app and log into your account
2. Go to **Settings → Advanced → Debug Data**
3. Enable **"Allow access on local network"** under STARLINK LOCATION

> ⚠️ This allows any device on your local network to access GPS data from your dish.

### 2. Network Access

Ensure the device running this container can reach the Starlink dish at `192.168.100.1:9200`.

## Quick Start

```bash
# Clone or download the files
# Build and run with Docker Compose
docker compose up -d

# View logs
docker compose logs -f

# Test the NMEA output
nc localhost 10110
```

## Configuration

### Command Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--dish` | 192.168.100.1:9200 | Starlink dish gRPC address |
| `--host` | 0.0.0.0 | TCP server bind address |
| `--port` | 10110 | TCP server port |
| `--rate` | 1.0 | NMEA output rate (seconds) |
| `--poll-interval` | 0.5 | GPS polling interval (seconds) |
| `--smoothing` | 6 | Number of samples to average for speed/heading |
| `--heading-offset` | 0.0 | Static compass heading offset in degrees (see below) |
| `--debug` | false | Enable debug logging |

### Docker Compose Configuration

Edit `docker-compose.yml` to customize:

```yaml
services:
  starlink-nmea:
    build: .
    container_name: starlink-nmea-server
    network_mode: host
    command: >
      --dish 192.168.100.1:9200
      --host 0.0.0.0
      --port 10110
      --rate 1.0
      --poll-interval 0.5
      --smoothing 6
      --heading-offset 321.7
    restart: unless-stopped
```

### Faster Updates

For sub-second NMEA output (useful for marine autopilots):

```yaml
command: >
  --dish 192.168.100.1:9200
  --port 10110
  --rate 0.5
  --poll-interval 0.25
  --smoothing 8
```

### Speed/Heading Smoothing

The `--smoothing` parameter controls how many GPS samples are averaged to calculate speed and heading. This reduces noise and jitter from GPS position variations.

- Default: `6` samples (at 0.5s poll interval = 3 seconds of averaging)
- Higher values = smoother but slower to respond to changes
- Lower values = more responsive but noisier
- Set to `1` to disable smoothing

The heading uses circular averaging to properly handle the 0°/360° boundary.

### Compass Heading Offset

The Starlink dish reports its boresight azimuth (the direction the dish is pointing). Since the dish may not be installed aligned with your vessel's centerline, you can configure a static offset to convert the dish azimuth to vessel heading:

```
vessel_heading = dish_boresight_azimuth + heading_offset
```

**To calculate your offset:**

1. Point your vessel at a known heading (e.g., using a handheld compass or landmark)
2. Note the `boresightAzimuthDeg` value from the Starlink diagnostics
3. Calculate: `offset = known_heading - boresight_azimuth`

**Example:**
- Vessel heading: 281°
- Dish boresight azimuth: -40.73°
- Offset: 281 - (-40.73) = **321.73°**

```yaml
command: >
  --heading-offset 321.73
```

## Generated NMEA Sentences

### GPS Sentences

| Sentence | Description |
|----------|-------------|
| GPGSV | GPS Satellites in View (simulated satellite data) |
| GPGGA | GPS Fix Data (position, altitude, fix quality) |
| GPGLL | Geographic Position - Latitude/Longitude |
| GPVTG | Track Made Good and Ground Speed |
| GPRMC | Recommended Minimum Navigation Information |
| GPGSA | GPS DOP and Active Satellites |

### Compass/Heading Sentences (from Starlink alignment stats)

| Sentence | Description |
|----------|-------------|
| HCHDM | Heading, Magnetic (with offset applied) |
| HCHDT | Heading, True (magnetic heading adjusted for variation) |
| HCHDG | Heading with Deviation & Variation |

### Sensor Sentences

| Sentence | Description |
|----------|-------------|
| HCXDR | Transducer Measurement - Tilt/Pitch (degrees off vertical) |

## Sample Output

```
$GPGSV,4,1,14,05,30,000,40,06,35,030,41,11,40,060,42,12,45,090,43,1*64
$GPGSV,4,2,14,13,50,120,44,15,55,150,45,18,60,180,46,20,65,210,47,1*6B
$GPGSV,4,3,14,21,70,240,48,23,75,270,49,25,80,300,40,29,85,330,41,1*62
$GPGSV,4,4,14,46,,,42,48,,,43,1*6D
$GPGGA,120000.00,2654.373395,N,08234.690521,W,1,11,0.9,1.4,M,-22.0,M,,*5A
$GPGLL,2654.373395,N,08234.690521,W,120000.00,A,A*7E
$GPVTG,139.3,T,142.3,M,10.0,N,18.5,K,A*22
$GPRMC,120000.00,A,2654.373395,N,08234.690521,W,10.0,139.3,111225,3.0,W,A,V*44
$GPGSA,A,3,05,06,11,12,13,15,18,20,21,23,25,,1.8,0.9,1.6,1*28
$HCHDM,281.0,M*1A
$HCHDT,278.0,T*1B
$HCHDG,281.0,,,3.0,W*0C
$HCXDR,A,2.3,D,PTCH*5E
```

## Integration Examples

### OpenCPN / Chart Plotters

Configure a network data source:
- Connection type: TCP
- Address: `<server-ip>`
- Port: `10110`

### SignalK

Add a TCP connection:
```json
{
  "type": "tcp",
  "host": "localhost",
  "port": 10110
}
```

### gpsd

```bash
gpsd -n tcp://localhost:10110
```

### Navionics / Marine Apps

Most marine navigation apps support TCP NMEA input. Configure with your server's IP and port 10110.

## How It Works

1. **Connects** to Starlink dish at `192.168.100.1:9200` via gRPC
2. **Discovers** the API schema using gRPC reflection
3. **Polls** `GetLocation` request at configurable intervals (default 0.5s)
4. **Calculates** speed and heading from position changes
5. **Generates** NMEA sentences with real GPS data + simulated satellite info
6. **Broadcasts** to all connected TCP clients

## Network Requirements

The container uses `network_mode: host` to access the Starlink dish on your local network. If you need bridge networking, ensure proper routing to `192.168.100.1`.

## Troubleshooting

### No GPS data / "No location data received"

1. **Verify location sharing is enabled** in Starlink mobile app:
   - Settings → Advanced → Debug Data → Allow access on local network

2. **Test direct connection:**
   ```bash
   grpcurl -plaintext -d '{"getLocation":{}}' \
     192.168.100.1:9200 \
     SpaceX.API.Device.Device/Handle
   ```

3. **Check network connectivity:**
   ```bash
   ping 192.168.100.1
   nc -zv 192.168.100.1 9200
   ```

### Connection refused

- Ensure you're on the Starlink network (not using cellular or another WAN)
- Check if a firewall is blocking port 9200

### "gRPC reflection failed"

- Your Starlink firmware may be too old or have reflection disabled
- Try updating your Starlink dish firmware via the app

### Clients connect but receive no data

- Check logs for "GPS fix acquired" message
- Verify location sharing is enabled
- Run with `--debug` flag for more information

## Files

| File | Description |
|------|-------------|
| `starlink_nmea_server.py` | Main server using gRPC reflection |
| `Dockerfile` | Container build configuration |
| `docker-compose.yml` | Docker Compose deployment |
| `nmea_simulator.py` | Standalone NMEA simulator (no Starlink needed) |

## Technical Notes

- Uses **gRPC reflection** to dynamically discover Starlink's protobuf schema
- Compatible with **protobuf 5.x/6.x** APIs
- Satellite data (GPGSV, GPGSA) is simulated as Starlink doesn't expose raw satellite info
- Speed/heading calculated via haversine formula from position deltas
- **Smoothing** uses rolling average for speed and circular mean for heading to handle 0°/360° wraparound

## License

MIT License

## Acknowledgments

- [sparky8512/starlink-grpc-tools](https://github.com/sparky8512/starlink-grpc-tools) for Starlink API research
- SpaceX for enabling local network access to dish telemetry
