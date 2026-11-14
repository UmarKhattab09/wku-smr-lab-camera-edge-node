# Camera Node Setup (Raspberry Pi)

How to bring up a new camera Pi for the WKU SMR lab: camera host service,
camera edge container, PLC trigger, and Swarm.

Repo: `https://github.com/UmarKhattab09/wku-smr-lab-camera-edge-node`, branch **`plc_update`**
Commands assume user `admin` and the repo cloned to `/home/admin/wku-smr-lab-camera-edge-node`.

---

## How a camera node works

```
 USB camera ──► camera_host (systemd, on the Pi)  ──Unix socket──►  camera_edge (Docker)  ──MQTT──► broker ──► middleware ──► Postgres
                owns /dev/video*, runs 24/7          /run/camera-hw/camera.sock   detects markers, saves runs
                                                                                   ▲
                                                             PLC tags (Data_Start / Db_Stop) via pycomm3
```

- **camera_host** runs directly on the Pi under systemd, because Swarm containers can't be given `/dev/video0`. It opens the camera and serves frames over a Unix socket.
- **camera_edge** runs in Docker. It reads frames from the socket, detects ArUco markers, saves each run to disk and publishes to MQTT.
- **PLC trigger:** with `CAMERA_PLC_TRIGGER=true`, the edge waits for `Data_Start` to go 0→1, sends until `Db_Stop` goes 0→1, then waits for the next run. `IMU_Tare` is ignored by cameras.

---

## 1. Get the code

```bash
cd ~
git clone https://github.com/UmarKhattab09/wku-smr-lab-camera-edge-node.git
cd wku-smr-lab-camera-edge-node
git checkout plc_update
cd camera-swarm-fusion
```

Repo already on the Pi? Switch branches instead:

```bash
cd ~/wku-smr-lab-camera-edge-node
git status                 # stash local edits first if any: git stash
git fetch origin plc_update
git checkout plc_update && git pull origin plc_update
```

If `checkout` says `did not match any file(s)`, the clone only tracks one branch:

```bash
git remote set-branches --add origin plc_update
git fetch origin plc_update
git checkout -b plc_update origin/plc_update
```

Check: `git branch --show-current` → `plc_update`.

---

## 2. Camera host (runs 24/7)

### 2.1 Install the host's Python environment

```bash
cd /home/admin/wku-smr-lab-camera-edge-node/camera-swarm-fusion
sudo apt update
sudo apt install -y python3-venv v4l-utils libgl1
python3 -m venv .venv
.venv/bin/pip install -r requirements-host.txt
```

### 2.2 Find the camera

```bash
v4l2-ctl --list-devices
ls /dev/v4l/by-id/
```

The default is `/dev/video0`. On a Pi the USB camera isn't always `video0`.
The `/dev/v4l/by-id/...-video-index0` path never changes, so use it if `video0` is wrong.

### 2.3 Test the host by hand (optional)

```bash
CAMERA_SOCKET_PATH=/tmp/camera.sock CAMERA_DEVICE=/dev/video0 .venv/bin/python -m camera_host
# second SSH window:
curl -s --unix-socket /tmp/camera.sock http://x/v1/readyz; echo
```

Ctrl+C when done.

### 2.4 Install it as a systemd service

The template in `deploy/systemd/camera-hw.service` has another Pi's path in it.
This copies it and puts in **this** folder's path. Run it from inside `camera-swarm-fusion`:

```bash
cd /home/admin/wku-smr-lab-camera-edge-node/camera-swarm-fusion
sudo cp deploy/systemd/camera-hw.service /etc/systemd/system/camera-hw.service
sudo sed -i \
  -e "s#^WorkingDirectory=.*#WorkingDirectory=$(pwd)#" \
  -e "s#^ExecStart=.*#ExecStart=$(pwd)/.venv/bin/python -m camera_host#" \
  /etc/systemd/system/camera-hw.service

grep -E "WorkingDirectory|ExecStart|CAMERA_DEVICE|User=" /etc/systemd/system/camera-hw.service
```

Both paths must be full paths, e.g. `/home/admin/wku-smr-lab-camera-edge-node/camera-swarm-fusion/.venv/bin/python`.
If the camera isn't `/dev/video0` or the user isn't `admin`, edit `CAMERA_DEVICE=` / `User=` / `Group=`:
`sudo nano /etc/systemd/system/camera-hw.service`.

Start it, and make it start on every boot:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now camera-hw
sleep 10; systemctl status camera-hw --no-pager
```

**Healthy:** `active (running) since ... 10s ago`, and the time keeps growing.
**Crash loop:** `since ... 10ms ago` and `Scheduled restart`, so check the logs (Troubleshooting below).

`Restart=always` brings it back within 3 s after a crash or unplug.

### 2.5 Confirm it sees the camera

```bash
ls -l /run/camera-hw/camera.sock
curl -s --unix-socket /run/camera-hw/camera.sock http://x/v1/readyz; echo
journalctl -u camera-hw -f          # live logs
```

---

## 3. Camera edge image

### 3.1 Install Docker (once)

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker admin
# log out and back in so the group takes effect
```

### 3.2 Build

The Dockerfile is already in the repo (`camera_edge/Dockerfile`), so you only build:

```bash
cd /home/admin/wku-smr-lab-camera-edge-node/camera-swarm-fusion
docker build -f camera_edge/Dockerfile -t camera-edge:latest .
docker images | grep camera-edge
```

The trailing `.` matters, so build from `camera-swarm-fusion`. The first build on a Pi takes several minutes.

**Built under another tag** (e.g. `camera-edge:plc_update`)? Point `latest` at it:

```bash
docker tag camera-edge:latest camera-edge:pre-plc        # backup of the old image (skip if none)
docker tag camera-edge:plc_update camera-edge:latest
```

Undo: `docker tag camera-edge:pre-plc camera-edge:latest`.

---

## 4. Run the edge (standalone test)

Each camera needs its **own** `DEVICE_ID` and `CAMERA_NAME`. It publishes to `camera/<DEVICE_ID>_<CAMERA_NAME>`.
Used so far: `113_front`, `114_side`.

```bash
docker run --rm -it --name camera-edge-test \
  -v /run/camera-hw:/run/camera-hw \
  -v /opt/camera-sensor-fusion/data:/app/data \
  -e MQTT_BROKER_IP=192.168.5.169 \
  -e MQTT_BROKER_PORT=1883 \
  -e DEVICE_ID=115 \
  -e CAMERA_NAME=cam3 \
  -e CAMERA_CALIBRATION_PATH=calib/PLACEHOLDER_front_1920x1080.yml \
  -e CAMERA_MARKER_LENGTH_M=0.0485 \
  -e CAMERA_PLC_TRIGGER=true \
  -e CAMERA_PLC_FAKE=false \
  -e CAMERA_PLC_PATH=192.168.4.248 \
  camera-edge:latest
```

- `--rm -it` runs it in the foreground; Ctrl+C stops and removes it.
- For background use: `-d --restart unless-stopped` instead of `--rm -it`, then `docker logs -f camera-edge-test`.
- Or put the `-e` values in a file (`KEY=value`, one per line, **no quotes**) and use `--env-file camera.env`.

**Expected log:**

```
starting camera_edge camera_name=cam3 device_id=115 broker=192.168.5.169:1883 ...
PLC trigger on: PLC 192.168.4.248, start on Data_Start, stop on Db_Stop
Initialization complete.
waiting for PLC tag Data_Start   [Data_Start=False Db_Stop=False]     <- every 5 s
PLC: Data_Start ON -> detecting and sending
frame=0 dets=... 
PLC: run ended (N frames sent ...); reason: Db_Stop turned ON
waiting for PLC tag Data_Start ...
```

The first `cannot reach PLC at ...: None` line right at startup is harmless (`None` = no error yet, still connecting).

**Don't run two edges for the same camera.** If this camera is also in the Swarm, scale that service to 0 first:
`docker service scale edge_camera-<name>=0` (on the manager); use `=1` to restore.

---

## 5. PLC settings

| Variable | Default | Notes |
|---|---|---|
| `CAMERA_PLC_TRIGGER` | `false` | `true` = wait for PLC; `false` = send as soon as the container starts |
| `CAMERA_PLC_PATH` | `192.168.1.103` | Real PLC. Use the UNO Q's IP for bench tests |
| `CAMERA_PLC_TAG_START` | `Data_Start` | |
| `CAMERA_PLC_TAG_STOP` | `Db_Stop` | |
| `CAMERA_PLC_POLL_S` | `0.05` | Tags read every 50 ms, so a 1 s pulse is easily caught |
| `CAMERA_PLC_MAX_RUN_S` | `600` | Ends a run if `Db_Stop` never comes (`0` = no limit) |
| `CAMERA_PLC_FAKE` | `false` | `true` = built-in fake PLC (start at 3 s, stop at 25 s), no network needed |

The PLC pulses each tag for ~1 s. Only the 0→1 change counts:

| PLC | Camera |
|---|---|
| `Data_Start` 0→1 | starts a run (new session folder, frames numbered from 0) |
| `Data_Start` 1→0 | keeps sending |
| `IMU_Tare` | ignored |
| `Db_Stop` 0→1 | ends the run, re-arms |

A tag that's already 1 when the container starts does **not** count.

### Bench testing with the Arduino UNO Q (no robot)

The UNO Q runs `plc_emulator.py`, which answers pycomm3 like the real Logix PLC. Set
`CAMERA_PLC_PATH=<unoq-ip>` (check with `hostname -I` on the UNO Q). On the UNO Q:

```bash
cd ~/unoq-plc
python3 plc_emulator.py --manual --tare-at 5 --stop-at 30
```

| Key (+ Enter) | Action |
|---|---|
| Enter | one full run: start, tare at 5 s, stop at 30 s |
| `s` / `t` / `x` | single `Data_Start` / `IMU_Tare` / `Db_Stop` pulse |
| `q` | quit |

Without `--manual` it runs on its own timer (`--first-delay`, `--gap`, `--runs`).
Check a Pi can reach it: `nc -zv <unoq-ip> 44818`.

---

## 6. Where the data goes

Only **during a run** (between `Data_Start` and `Db_Stop`).

**On the Pi** (via the `/opt/camera-sensor-fusion/data` mount), one folder per run:

```
/opt/camera-sensor-fusion/data/sessions/<name>_session_<date>_<time>/
    detections.csv    # marker poses, one row per marker per frame
    frames/           # raw frames (JPEG)
    annotated/        # frames with markers drawn
    manifest.json     # settings used
```

**In the database:** published to `camera/<DEVICE_ID>_<CAMERA_NAME>` and stored only if a session is
started on the dashboard. Otherwise the middleware logs `No current active session` and drops it.

Watch the broker: `mosquitto_sub -h 192.168.5.169 -t 'camera/#' -v`

**Disk space:** full-res frames + annotated copies fill an SD card fast. If you only need poses:

```bash
-e CAMERA_SAVE_FRAMES=false -e CAMERA_SAVE_ANNOTATED=false
```

Check space with `df -h /opt`.

---

## 7. Add the camera to the Swarm

On the **manager**:

```bash
docker swarm join-token worker                                  # run the printed command on the new Pi
docker node ls                                                  # new Pi should show Ready
docker node update --label-add camera=<name> <pi-hostname>      # e.g. camera=cam3 CameraPi3
```

The image must exist **on the Pi** (Swarm doesn't copy local images between machines).

In `wku-smr-lab-middleware/deploy/swarm-edge-nodes.yml`, copy the `camera-side` service as `camera-<name>` and change the IDs and placement.
The PLC lines follow the IMU settings, so one switch (`IMU_PLC_TRIGGER` / `IMU_PLC_PATH`) drives all nodes:

```yaml
  camera-cam3:
    image: camera-edge:latest
    environment:
      MQTT_BROKER_IP: ${MQTT_BROKER_IP}
      DEVICE_ID: "${CAMERA_CAM3_DEVICE_ID}"
      CAMERA_NAME: ${CAMERA_CAM3_NAME}
      CAMERA_CALIBRATION_PATH: ${CAMERA_CAM3_CALIBRATION_PATH}
      CAMERA_MARKER_LENGTH_M: "${CAMERA_MARKER_LENGTH_M}"
      CAMERA_EDGE_HEALTH_PATH: /tmp/camera-edge-health.json
      CAMERA_PLC_TRIGGER: "${IMU_PLC_TRIGGER:-false}"
      CAMERA_PLC_PATH: "${IMU_PLC_PATH:-192.168.1.103}"
      CAMERA_PLC_TAG_START: "${IMU_PLC_TAG_START:-Data_Start}"
      CAMERA_PLC_TAG_STOP: "${IMU_PLC_TAG_STOP:-Db_Stop}"
      CAMERA_PLC_POLL_S: "${IMU_PLC_POLL_S:-0.05}"
      CAMERA_PLC_MAX_RUN_S: "${IMU_PLC_MAX_RUN_S:-600}"
      CAMERA_PLC_FAKE: "${IMU_PLC_FAKE:-false}"
    deploy:
      replicas: 1
      labels:
        smr.category: camera
        smr.device: "${CAMERA_CAM3_DEVICE_ID}_${CAMERA_CAM3_NAME}"
      restart_policy:
        condition: any
      placement:
        constraints:
          - node.labels.camera == cam3
    volumes:
      - type: bind
        source: /run/camera-hw
        target: /run/camera-hw
      - type: bind
        source: /opt/camera-sensor-fusion/data
        target: /app/data
    healthcheck:
      test: [CMD, python3, -m, camera_edge, healthcheck]
      interval: 10s
      timeout: 5s
      retries: 3
      start_period: 30s
```

Add to `deploy/.env`:

```
CAMERA_CAM3_DEVICE_ID=115
CAMERA_CAM3_NAME=cam3
CAMERA_CAM3_CALIBRATION_PATH=calib/PLACEHOLDER_front_1920x1080.yml
```

Deploy from `deploy/`. `docker stack deploy` does **not** read `.env` by itself, so load it into the shell first:

```bash
cd ~/wku-smr-lab-middleware/deploy
set -a; source ./.env; set +a
docker stack deploy --resolve-image never -c swarm-edge-nodes.yml edge
```

Check:

```bash
docker service ps edge_camera-cam3
docker service inspect edge_camera-cam3 --format '{{range .Spec.TaskTemplate.ContainerSpec.Env}}{{println .}}{{end}}' | grep -E "PLC|MQTT"
docker service logs -f edge_camera-cam3 | grep -E "PLC|waiting|cannot reach"
```

After rebuilding or retagging the image on a Pi, restart its service: `docker service update --force edge_camera-<name>`.

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `camera-hw`: `status=203/EXEC`, `Unable to locate executable '/.venv/bin/python'` | Empty/wrong path in the service file. Redo 2.4 (the `sed` must run from inside `camera-swarm-fusion`) |
| `camera-hw`: `status=200/CHDIR` | `WorkingDirectory=` path wrong |
| `camera-hw`: `ModuleNotFoundError: cv2` / `libGL.so.1` | `.venv/bin/pip install -r requirements-host.txt` / `sudo apt install -y libgl1` |
| `camera-hw`: camera can't be opened | Wrong `CAMERA_DEVICE`; check `v4l2-ctl --list-devices`, use `/dev/v4l/by-id/...` |
| Edge: socket connection errors | `camera-hw` not running, or `/run/camera-hw` not mounted (`-v /run/camera-hw:/run/camera-hw`) |
| `docker: ... container name "/camera-edge-test" is already in use` | `docker rm -f camera-edge-test` (or use another `--name`) |
| `permission denied ... docker.sock` | `sudo usermod -aG docker admin`, log out and back in |
| `cannot reach PLC at <ip>: failed to open a connection` (keeps repeating) | Network. From the Pi: `nc -zv <plc-ip> 44818`. UNO Q on another subnet must be routable |
| `PLC tag problem: Data_Start: ...` | Connected, but tag name wrong on that PLC |
| Data arrives before `Data_Start` | `CAMERA_PLC_TRIGGER` isn't `true` in the container. Check with `docker service inspect` (section 7) |
| No PLC lines in the log at all | Old image. `docker images | grep camera-edge`, rebuild/retag, `docker service update --force` |
| Swarm task `Pending` / `no suitable node` | Node label missing or doesn't match `placement` (`docker node inspect <host> --format '{{.Spec.Labels}}'`) |
| SD card full | `CAMERA_SAVE_FRAMES=false`, `CAMERA_SAVE_ANNOTATED=false`; clean `/opt/camera-sensor-fusion/data/sessions/` |

Useful commands:

```bash
systemctl status camera-hw --no-pager          # host state
journalctl -u camera-hw -n 30 --no-pager       # host logs
docker ps -a                                   # containers on this Pi
docker logs -f <container>                     # standalone edge logs
docker service logs -f edge_camera-<name>      # Swarm edge logs (on the manager)
df -h /opt                                     # disk space
```
