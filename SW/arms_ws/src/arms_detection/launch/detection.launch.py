# arms_detection runs inside a Docker container — NOT launched via ROS2.
#
# Start the detection container separately:
#
#   cd arms_ws/src/arms_detection/docker
#   docker compose up --build
#
# The container:
#   - Uses network_mode: host  (same DDS network as the host)
#   - Subscribes to /arms/image_raw
#   - Publishes   /arms/detections
#
# Environment variables (optional, set in docker-compose.yml or .env):
#   ROS_DOMAIN_ID  — must match the host (default: 0)
#   ARMS_MODEL     — path to .pt model inside container (default: /models/drone.pt)
#   ARMS_CONF      — YOLO confidence threshold (default: 0.5)
#   ARMS_IOU       — YOLO IOU threshold (default: 0.45)
raise RuntimeError(
    "arms_detection does not have a ROS2 launch target. "
    "Run: cd arms_detection/docker && docker compose up"
)
