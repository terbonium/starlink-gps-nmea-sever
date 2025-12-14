FROM python:3.12-slim

LABEL maintainer="Starlink NMEA GPS Server"
LABEL description="Connects to Starlink gRPC API and serves real-time GPS as NMEA over TCP"

WORKDIR /app

# Install Python gRPC dependencies
RUN pip install --no-cache-dir \
    grpcio \
    grpcio-tools \
    grpcio-reflection

# Copy server file
COPY starlink_nmea_server.py .

RUN chmod +x starlink_nmea_server.py

EXPOSE 10110

ENV STARLINK_DISH=192.168.100.1:9200
ENV NMEA_HOST=0.0.0.0
ENV NMEA_PORT=10110

HEALTHCHECK --interval=30s --timeout=3s --start-period=15s --retries=3 \
    CMD python -c "import socket; s=socket.socket(); s.connect(('localhost', 10110)); s.close()" || exit 1

ENTRYPOINT ["python", "-u", "starlink_nmea_server.py"]
CMD ["--dish", "192.168.100.1:9200", "--host", "0.0.0.0", "--port", "10110", "--debug"]
