#!/usr/bin/env python3
"""
IMU Guía Inercial — Visualizador 3D en Tiempo Real
Renderizado con pygame puro (sin OpenGL). Requisitos: pip install pygame pyserial

Uso:
  python imu_visualizer.py --demo               # modo demo sin hardware
  python imu_visualizer.py                      # detecta puerto automáticamente
  python imu_visualizer.py --port /dev/ttyACM0

Controles:
  Arrastrar (botón izq.) → orbitar cámara
  Rueda del ratón        → zoom
  R                      → reset cámara
  C                      → calibrar magnetómetro (durante calibración inicial, por USB)
  T                      → ocultar / mostrar ventana de estado y consola
  ESC                    → salir
"""
import sys
import os
import math
import threading
import time
import argparse
import socket
from collections import deque

# Fuerza Mesa software renderer — necesario cuando el driver NVIDIA
# tiene mismatch de versiones entre kernel y librería (hasta próximo reboot).
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE',       '1')
os.environ.setdefault('GALLIUM_DRIVER',              'softpipe')
os.environ.setdefault('__GLX_VENDOR_LIBRARY_NAME',   'mesa')
os.environ.setdefault('SDL_VIDEO_GL_DRIVER',
                      '/lib/x86_64-linux-gnu/libGLX_mesa.so.0')

import pygame

try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

# ─────────────────────────────────────────────────────────────────────────────
# Configuración
# ─────────────────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 1400, 920
FPS     = 60
BAUD     = 115200
UDP_PORT = 4210
PANEL_W  = 302                       # panel de datos (derecha)
MAP_W    = 318                       # franja de mapa / posición (izquierda)
VP_X0   = MAP_W                      # x donde empieza el viewport 3D
VP_W    = WIN_W - PANEL_W - MAP_W    # ancho del viewport 3D (centro)
VP_CX   = VP_X0 + VP_W // 2
VP_CY   = WIN_H // 2

# ─────────────────────────────────────────────────────────────────────────────
# Estado compartido (hilo serial ↔ hilo render)
# ─────────────────────────────────────────────────────────────────────────────
_lock  = threading.Lock()
_state = dict(roll=0.0, pitch=0.0, yaw=0.0,
              acc_x=0.0, acc_y=0.0, acc_z=1.0,
              rumbo=0.0, baro_alt=0.0, baro_press=101325.0,
              baro_temp=None, vel_z=0.0, connected=False)

_KEY_MAP = {
    'R': 'roll',  'P': 'pitch',  'Y': 'yaw',   'M': 'rumbo',
    'AX': 'acc_x', 'AY': 'acc_y', 'AZ': 'acc_z',
    # Claves que envía main.cpp (BMP180)
    'Alt': 'baro_alt', 'Pre': 'baro_press', 'Tmp': 'baro_temp',
    # Claves alternativas (protocolo original MPU9250)
    'BA': 'baro_alt', 'BP': 'baro_press', 'VZ': 'vel_z',
}

# ── Log de mensajes de estado (estilo terminal) y comando saliente ───────────
_msg_lock = threading.Lock()
_messages = deque(maxlen=200)          # mensajes >OK:/>CAL:/>ERR:/>ALIGN:… recibidos
_serial   = None                       # handle serie compartido (para enviar 'C')
_flags    = {'reset_origin': False}    # señales hilo lector → hilo render

def _get():
    with _lock: return dict(_state)

def _set(**kw):
    with _lock: _state.update(kw)

def _log(msg: str):
    """Añade una línea al log de estado con marca de tiempo."""
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    with _msg_lock:
        _messages.append(line)
    print(f"[Log] {msg}")

def _get_messages(n: int):
    with _msg_lock:
        return list(_messages)[-n:]

def _ingest_line(line: str):
    """Procesa una línea '>KEY:VAL'. Datos → estado; resto → log de estado."""
    line = line.strip()
    if not line.startswith('>') or ':' not in line:
        return
    key, _, val = line[1:].partition(':')
    if key in _KEY_MAP:
        try: _set(**{_KEY_MAP[key]: float(val)})
        except ValueError: pass
    else:
        # Mensaje de inicio / calibración / error / alineación / etc.
        _log(line[1:])
        # '>OK:1' = sistema listo tras calibración → fijar origen del mapa (0,0,0)
        if key == 'OK' and val.strip() == '1':
            _flags['reset_origin'] = True

def send_command(cmd: str):
    """Envía un comando al ESP por serie (la ventana de calibración usa USB)."""
    s = _serial
    if s is not None:
        try:
            s.write(cmd.encode())
            _log(f"→ Comando '{cmd}' enviado (calibrar magnetómetro)")
        except Exception as e:
            _log(f"Error enviando comando: {e}")
    else:
        _log("Comando 'C' requiere conexión USB serie (no disponible por UDP)")

# ─────────────────────────────────────────────────────────────────────────────
# Hilo lector serial
# ─────────────────────────────────────────────────────────────────────────────
def serial_reader(port: str):
    """Lee la telemetría por serie. Si se pierde la conexión (p.ej. botón de
    reset del ESP), reintenta abrir el puerto periódicamente hasta reconectar."""
    global _serial
    while True:
        ser = None
        try:
            ser = serial.Serial(port, BAUD, timeout=2)
            _serial = ser
            _set(connected=True)
            _log(f"Conectado a {port} @ {BAUD} baudios")
            while True:
                raw = ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                if line:
                    _ingest_line(line)
                # readline() vacío = timeout sin datos (normal durante
                # calibración/alineación); seguimos esperando.
        except Exception as e:
            _serial = None
            _set(connected=False)
            _log(f"Conexión perdida: {e} — reintentando…")
            try:
                if ser is not None: ser.close()
            except Exception:
                pass
            time.sleep(1.5)   # esperar a que el puerto reaparezca tras el reset

# ─────────────────────────────────────────────────────────────────────────────
# Hilo receptor UDP (WiFi)
# ─────────────────────────────────────────────────────────────────────────────
def udp_receiver():
    """Recibe telemetría por UDP. Detecta la pérdida de conexión (reset del ESP)
    por ausencia de paquetes y marca DESCONECTADO; reconecta solo al volver."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', UDP_PORT))
    sock.settimeout(1.0)
    _log(f"Escuchando UDP en puerto {UDP_PORT}")
    last_data = 0.0
    while True:
        try:
            data, addr = sock.recvfrom(512)
            if not _get()['connected']:
                _log(f"Datos UDP recibidos de {addr[0]}")
            _set(connected=True)
            last_data = time.time()
            for line in data.decode('utf-8', errors='ignore').splitlines():
                _ingest_line(line)
        except socket.timeout:
            # Sin paquetes: si llevábamos tiempo conectados, marcar pérdida
            if _get()['connected'] and (time.time() - last_data) > 3.0:
                _set(connected=False)
                _log("Sin datos UDP — esperando reconexión…")
        except Exception as e:
            _set(connected=False)
            _log(f"UDP error: {e} — reintentando…")
            time.sleep(1.0)

# ─────────────────────────────────────────────────────────────────────────────
# Hilo demo (movimiento sintético)
# ─────────────────────────────────────────────────────────────────────────────
def demo_runner():
    t0 = time.time()
    while True:
        t  = time.time() - t0
        # baro_alt oscila ±3 m / período ~52 s; vel_z es su derivada exacta
        _set(roll=40*math.sin(t*0.38),    pitch=22*math.sin(t*0.25+1.0),
             yaw=(t*16)%360-180,           rumbo=(t*22)%360,
             acc_x=0.14*math.sin(t*1.4),  acc_y=0.09*math.cos(t*0.95),
             acc_z=1.0+0.07*math.sin(t*2.2), connected=True,
             baro_alt=3.0*math.sin(t*0.12),
             baro_press=101325 - 34*math.sin(t*0.12),
             baro_temp=22.5 + 1.5*math.sin(t*0.04),
             vel_z=0.36*math.cos(t*0.12))
        time.sleep(1/50)

# ─────────────────────────────────────────────────────────────────────────────
# Álgebra 3D (sin numpy)
# ─────────────────────────────────────────────────────────────────────────────
def _norm(v):
    x,y,z = v
    n = math.sqrt(x*x+y*y+z*z)
    return (x/n,y/n,z/n) if n > 1e-12 else (0,0,1)

def _dot(a, b):   return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def _cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def _sub(a, b):   return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def _mm(A, B):
    """3×3 matrix multiply."""
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]

def _mv(M, v):
    x,y,z = v
    return (M[0][0]*x+M[0][1]*y+M[0][2]*z,
            M[1][0]*x+M[1][1]*y+M[1][2]*z,
            M[2][0]*x+M[2][1]*y+M[2][2]*z)

def _ry(a):
    c,s = math.cos(a),math.sin(a)
    return [[c,0,s],[0,1,0],[-s,0,c]]

def _rx(a):
    c,s = math.cos(a),math.sin(a)
    return [[1,0,0],[0,c,-s],[0,s,c]]

def _rz(a):
    c,s = math.cos(a),math.sin(a)
    return [[c,-s,0],[s,c,0],[0,0,1]]

def imu_matrix(roll, pitch, yaw):
    """Convención ZYX aeronáutica: Ryaw · Rpitch · Rroll(neg)."""
    return _mm(_mm(_ry(math.radians(yaw)),
                   _rx(math.radians(pitch))),
               _rz(math.radians(-roll)))

def look_at(eye, center=(0,0,0)):
    """Devuelve (right, up, fwd) para transformar puntos mundo→cámara."""
    fwd   = _norm(_sub(center, eye))
    right = _norm(_cross(fwd, (0,1,0)))
    up    = _cross(right, fwd)
    return right, up, fwd

def to_cam(p, right, up, fwd, eye):
    d = _sub(p, eye)
    return (_dot(d,right), _dot(d,up), _dot(d,fwd))

def project(p_cam, focal, cx, cy):
    x,y,z = p_cam
    if z < 0.05: return None
    f = focal/z
    return (int(cx + x*f), int(cy - y*f))

# ─────────────────────────────────────────────────────────────────────────────
# Geometría del avión
# ─────────────────────────────────────────────────────────────────────────────
# Coordenadas: X derecha, Y arriba, Z negativo = nariz, Z positivo = cola
def _build_airplane():
    faces = []

    def q(v0,v1,v2,v3,c): faces.append(((v0,v1,v2,v3), c))
    def t(v0,v1,v2,c):    faces.append(((v0,v1,v2), c))

    C_TOP  = (200,210,232); C_SIDE = (152,162,192); C_BOT = (120,128,158)
    C_WING = (90,133,232);  C_WBOT = (66,103,193)
    C_NOSE = (100,132,25);  C_FIN  = (234,50,50)    #  C_NOSE = (242,132,25);
    C_CAB  = (126,218,255); C_CSID = (98,192,232)

    # ── Fuselaje ─────────────────────────────────────────────────────────
    secs = [(-1.80, 0.14, 0.13),
            (-0.35, 0.23, 0.20),
            ( 1.75, 0.16, 0.14)]
    for i in range(len(secs)-1):
        z1,w1,h1 = secs[i]; z2,w2,h2 = secs[i+1]
        q((-w1, h1,z1),( w1, h1,z1),( w2, h2,z2),(-w2, h2,z2), C_TOP)
        q((-w1,-h1,z1),( w1,-h1,z1),( w2,-h2,z2),(-w2,-h2,z2), C_BOT)
        q((-w1, h1,z1),(-w1,-h1,z1),(-w2,-h2,z2),(-w2, h2,z2), C_SIDE)
        q(( w1, h1,z1),( w1,-h1,z1),( w2,-h2,z2),( w2, h2,z2), C_SIDE)

    # Tapa frontal
    nz,nw,nh = secs[0]
    q((-nw, nh,nz),( nw, nh,nz),( nw,-nh,nz),(-nw,-nh,nz), C_SIDE)

    # Cono de nariz
    #   tip = (0.0, 0.0, -2.60)
    #   corners = [(-nw,nh,nz),(nw,nh,nz),(nw,-nh,nz),(-nw,-nh,nz)]
    #   for j in range(4):
    #       t(tip, corners[j], corners[(j+1)%4], C_NOSE)

    # ── Alas principales ─────────────────────────────────────────────────
    for sx in (-1, 1):
        # x0,zf0,zb0 = 0.23, -0.38,  0.55
        # x1,zf1,zb1 = 3.10,  0.10,  1.35
        x0,zf0,zb0 = 0.20, -0,  0.8
        x1,zf1,zb1 = 1.50,  0.10,  0.8
        s0,s1 = sx*x0, sx*x1
        yw = -0.06
        q((s0, yw,      zf0),(s0, yw,      zb0),(s1, yw-0.02,zb1),(s1, yw-0.02,zf1), C_WING)
        q((s0, yw-0.05, zf0),(s0, yw-0.05, zb0),(s1, yw-0.05,zb1),(s1, yw-0.05,zf1), C_WBOT)
        q((s1, yw-0.02, zf1),(s1, yw-0.05, zf1),(s1, yw-0.05,zb1),(s1, yw-0.02,zb1), C_SIDE)

    # ── Estabilizador horizontal ──────────────────────────────────────────
    for sx in (-1, 1):
        #  x0,zf0,zb0 = 0.16, 1.30, 1.75
        #   x1,zf1,zb1 = 1.10, 1.50, 1.75

        x0,zf0,zb0 = 0.16, 1.30, 1.8
        x1,zf1,zb1 = 1.10, 1.50, 1.8
        s0,s1 = sx*x0, sx*x1
        q((s0,-0.04,zf0),(s0,-0.04,zb0),(s1,-0.04,zb1),(s1,-0.04,zf1), C_WING)

    # ── Timón de dirección (vertical) ─────────────────────────────────────
    _,tw,th = secs[2]
    q((-0.04, th, 1.25),(-0.04, 0.90, 1.70),(0.04, 0.90, 1.70),(0.04, th, 1.25), C_FIN)
    q((-0.04, th, 1.75),(-0.04, 0.90, 1.70),(0.04, 0.90, 1.70),(0.04, th, 1.75), C_FIN)
    t((-0.04,th,1.25),(0.04,th,1.25),(0.0,0.90,1.70), C_FIN)

    # ── Cabina ────────────────────────────────────────────────────────────
  # _,bw,bh = secs[1]
  # cb, ct = bh, bh+0.30
  # q((-bw*0.55,ct,-1.20),( bw*0.55,ct,-1.20),( bw*0.80,cb,-0.15),(-bw*0.80,cb,-0.15), C_CAB)
  # q((-bw*0.55,ct,-1.20),(-bw*0.55,cb,-1.20),(-bw*0.80,cb,-0.15),(-bw*0.80,ct,-0.65), C_CSID)
  # q(( bw*0.55,ct,-1.20),( bw*0.55,cb,-1.20),( bw*0.80,cb,-0.15),( bw*0.80,ct,-0.65), C_CSID)

    return faces

_AIRPLANE_FACES = _build_airplane()

# ─────────────────────────────────────────────────────────────────────────────
# Renderizador 3D por software (painter's algorithm)
# ─────────────────────────────────────────────────────────────────────────────
_LIGHT = _norm((0.35, 0.82, -0.45))   # dirección de luz (espacio mundo, fija)
_RIM   = _norm((-0.50, 0.30, -0.80))  # contraluz suave

def _shade(base, normal):
    diff    = max(0.0, _dot(normal, _LIGHT))
    rim     = max(0.0, _dot(normal, _RIM)) * 0.12
    intens  = 0.36 + 0.64 * diff + rim
    return tuple(min(255, int(c * intens)) for c in base)

def render_airplane(surf, R, cam_eye, cam_right, cam_up, cam_fwd, focal):
    rendered = []
    for verts_local, base_col in _AIRPLANE_FACES:
        verts_w  = [_mv(R, v) for v in verts_local]
        verts_c  = [to_cam(v, cam_right, cam_up, cam_fwd, cam_eye) for v in verts_w]
        avg_z    = sum(v[2] for v in verts_c) / len(verts_c)

        # normal en espacio mundo para iluminación
        if len(verts_w) >= 3:
            e1 = _sub(verts_w[1], verts_w[0])
            e2 = _sub(verts_w[2], verts_w[0])
            normal = _norm(_cross(e1, e2))
        else:
            normal = (0, 1, 0)

        pts = [project(vc, focal, VP_CX, VP_CY) for vc in verts_c]
        if any(p is None for p in pts): continue

        rendered.append((avg_z, pts, _shade(base_col, normal)))

    # Pintor: de atrás hacia adelante
    rendered.sort(key=lambda x: -x[0])
    for _, pts, col in rendered:
        if len(pts) >= 3:
            try:
                pygame.draw.polygon(surf, col, pts)
                edge = (max(0,col[0]-25), max(0,col[1]-25), max(0,col[2]-25))
                pygame.draw.polygon(surf, edge, pts, 1)
            except Exception:
                pass

def render_grid(surf, cam_eye, cam_right, cam_up, cam_fwd, focal):
    col = (35, 72, 35)
    y_g = -3.5
    for i in range(-12, 13, 2):
        for pa_w, pb_w in [
            ((-8, y_g, i*0.6), ( 8, y_g, i*0.6)),
            ((i*0.6, y_g, -15), (i*0.6, y_g,  15)),
        ]:
            pa_s = project(to_cam(pa_w, cam_right, cam_up, cam_fwd, cam_eye), focal, VP_CX, VP_CY)
            pb_s = project(to_cam(pb_w, cam_right, cam_up, cam_fwd, cam_eye), focal, VP_CX, VP_CY)
            if pa_s and pb_s:
                pygame.draw.line(surf, col, pa_s, pb_s, 1)

def render_mini_axes(surf, cam_right, cam_up):
    """Ejes de referencia 2D fijos en esquina inferior izquierda del viewport."""
    cx, cy = VP_X0 + 75, WIN_H - 75
    length = 48
    for axis, col, lbl in [((1,0,0),(210,55,55),'X'),
                            ((0,1,0),(55,210,55),'Y'),
                            ((0,0,1),(55,105,210),'Z')]:
        sx = int(cx + _dot(axis, cam_right) * length)
        sy = int(cy - _dot(axis, cam_up)    * length)
        pygame.draw.line(surf, col, (cx,cy), (sx,sy), 2)
        f = pygame.font.SysFont('monospace', 11)
        surf.blit(f.render(lbl, True, col), (sx-4, sy-8))


def render_g_meter(surf, acc_x, acc_y):
    """Indicador de carga G lateral/longitudinal — esquina inferior izq. del viewport."""
    cx, cy, r = VP_X0 + 75, WIN_H - 200, 52
    g_mag = math.sqrt(acc_x ** 2 + acc_y ** 2)

    # fondo
    pygame.draw.circle(surf, (10, 12, 28), (cx, cy), r)
    # anillos 0.5g y 1g
    pygame.draw.circle(surf, (35, 55, 100), (cx, cy), r,        1)
    pygame.draw.circle(surf, (35, 55, 100), (cx, cy), r // 2,   1)
    # cruces
    pygame.draw.line(surf, (35, 55, 100), (cx - r, cy), (cx + r, cy), 1)
    pygame.draw.line(surf, (35, 55, 100), (cx, cy - r), (cx, cy + r), 1)

    # punto G: accY = lateral (derecha+), accX = longitudinal (adelante+)
    scale = r * 0.8
    dot_x = int(cx + acc_y * scale)
    dot_y = int(cy - acc_x * scale)
    dot_x = max(cx - r + 6, min(cx + r - 6, dot_x))
    dot_y = max(cy - r + 6, min(cy + r - 6, dot_y))

    col = (255, 65, 65) if g_mag > 1.5 else (255, 200, 40) if g_mag > 0.5 else (0, 210, 90)
    pygame.draw.circle(surf, col, (dot_x, dot_y), 7)
    pygame.draw.circle(surf, (220, 220, 220), (cx, cy), 3)   # centro de referencia

    f_sm = pygame.font.SysFont('monospace', 12, bold=True)
    surf.blit(f_sm.render("G", True, (160, 185, 225)), (cx - 5, cy - r - 17))
    surf.blit(f_sm.render(f"{g_mag:.2f}g", True, col),  (cx - 24, cy + r + 5))

# ─────────────────────────────────────────────────────────────────────────────
# Panel de datos 2D (lado derecho)
# ─────────────────────────────────────────────────────────────────────────────
class Panel:
    def __init__(self):
        self.f_lg = pygame.font.SysFont('monospace', 21, bold=True)
        self.f_md = pygame.font.SysFont('monospace', 16)
        self.f_sm = pygame.font.SysFont('monospace', 13)
        self._y   = 0

    def _section(self, surf, label):
        px = WIN_W - PANEL_W
        pygame.draw.line(surf, (55,75,135), (px+8, self._y+10), (px+PANEL_W-8, self._y+10), 1)
        surf.blit(self.f_sm.render(label, True, (110,160,230)), (px+10, self._y))
        self._y += 22

    def _bar(self, surf, x, y, w, h, frac, color):
        bw = int(abs(frac) * w)
        rx = x - bw if frac < 0 else x
        # blend with dark bg instead of alpha
        bg = (10,12,30)
        a  = 0.45
        bc = tuple(int(c*a + bg[i]*(1-a)) for i,c in enumerate(color))
        pygame.draw.rect(surf, bc, (rx, y, max(1,bw), h))

    def _compass(self, surf, cx, cy, r, heading, f_sm):
        pygame.draw.circle(surf, (18,20,46), (cx,cy), r)
        pygame.draw.circle(surf, (75,115,205), (cx,cy), r, 2)
        for deg in range(0, 360, 10):
            a     = math.radians(deg - heading)
            inner = r - (9 if deg % 30 == 0 else 4)
            x1 = cx + int(r     * math.sin(a)); y1 = cy - int(r     * math.cos(a))
            x2 = cx + int(inner * math.sin(a)); y2 = cy - int(inner * math.cos(a))
            pygame.draw.line(surf, (120,140,200), (x1,y1), (x2,y2), 1)
        for lbl, ang in (('N',0),('E',90),('S',180),('O',270)):
            a  = math.radians(ang - heading)
            tx = cx + int((r-17)*math.sin(a)); ty = cy - int((r-17)*math.cos(a))
            s  = f_sm.render(lbl, True, (220,205,90))
            surf.blit(s, (tx - s.get_width()//2, ty - s.get_height()//2))
        a  = math.radians(-heading)
        nx = cx + int((r-12)*math.sin(a)); ny = cy - int((r-12)*math.cos(a))
        pygame.draw.line(surf, (255,55,55), (cx,ny), (nx,ny), 3)
        pygame.draw.circle(surf, (255,55,55), (nx,ny), 5)

    def _artificial_horizon(self, surf, cx, cy, r, roll, pitch):
        """Horizonte artificial circular con filtro circular por máscara alpha."""
        size = r * 3
        half = size // 2

        # Desplazamiento vertical por cabeceo (45 ° = radio completo)
        pitch_px = max(-r, min(r, int(pitch * r / 45.0)))
        horizon_y = half + pitch_px   # pitch>0 (nariz arriba) → horizonte baja

        # Contenido: cielo + tierra + línea de horizonte
        tmp = pygame.Surface((size, size))
        tmp.fill((28, 88, 180))                                              # cielo
        pygame.draw.rect(tmp, (100, 64, 18), (0, horizon_y, size, size))    # tierra
        pygame.draw.line(tmp, (255, 255, 255), (0, horizon_y), (size, horizon_y), 2)

        # Marcas de cabeceo cada 10 °
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            py = horizon_y - int(deg * r / 45.0)
            if 0 < py < size:
                w = r // 3 if abs(deg) == 10 else r // 2
                pygame.draw.line(tmp, (200, 200, 200), (half - w, py), (half + w, py), 1)

        # Rotación: alabeo positivo (derecha) → CW en pantalla → pygame rotate(-roll)
        rotated = pygame.transform.rotate(tmp, -roll)
        rc       = rotated.get_rect()
        crop     = pygame.Rect(rc.centerx - r, rc.centery - r, r * 2, r * 2)
        crop.clamp_ip(pygame.Rect(0, 0, rc.width, rc.height))

        cropped = pygame.Surface((r * 2, r * 2))
        cropped.blit(rotated, (0, 0), crop)

        # Máscara circular: BLEND_RGBA_MULT con stencil opaco dentro del círculo
        masked  = cropped.convert_alpha()
        stencil = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
        stencil.fill((255, 255, 255, 0))
        pygame.draw.circle(stencil, (255, 255, 255, 255), (r, r), r)
        masked.blit(stencil, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

        surf.blit(masked, (cx - r, cy - r))
        pygame.draw.circle(surf, (70, 100, 180), (cx, cy), r, 2)

        # Referencia fija del avión (alas doradas + centro)
        wing_inner = r // 3
        wing_outer = r // 3 + r // 3
        for sx in (-1, 1):
            x0 = cx + sx * wing_inner
            x1 = cx + sx * wing_outer
            pygame.draw.line(surf, (255, 215, 0), (x0, cy), (x1, cy), 3)
            pygame.draw.line(surf, (255, 215, 0), (x1, cy - 5), (x1, cy + 5), 2)
        pygame.draw.line(surf, (255, 215, 0), (cx - r // 8, cy), (cx + r // 8, cy), 3)
        pygame.draw.circle(surf, (255, 215, 0), (cx, cy), 3)

    def draw(self, surf, st, fps):
        px    = WIN_W - PANEL_W
        self._y = 14
        pygame.draw.rect(surf, (10,12,30), (px, 0, PANEL_W, WIN_H))
        pygame.draw.line(surf, (55,75,145), (px,0), (px,WIN_H), 2)

        # Título
        t = self.f_lg.render("IMU  Guía Inercial", True, (85,175,255))
        surf.blit(t, (px + (PANEL_W - t.get_width())//2, self._y)); self._y += 34

        # Estado
        ok  = st['connected']
        col = (0,215,95) if ok else (255,65,65)
        surf.blit(self.f_sm.render("● CONECTADO" if ok else "● DESCONECTADO", True, col),
                  (px+10, self._y)); self._y += 28

        # ── Ángulos ───────────────────────────────────────────────────────
        self._section(surf, "ÁNGULOS DE EULER")
        for lbl, val, unit, clr in [("Roll  X", st['roll'],  "°", (255,110,110)),
                                     ("Pitch Y", st['pitch'], "°", (110,230,110)),
                                     ("Yaw   Z", st['yaw'],   "°", (110,160,255))]:
            self._bar(surf, px+155, self._y+4, 118, 13, val/180.0, clr)
            surf.blit(self.f_md.render(f"{lbl}: {val:+7.2f}{unit}", True, clr),
                      (px+10, self._y))
            self._y += 24
        self._y += 8

        # ── Aceleraciones ─────────────────────────────────────────────────
        self._section(surf, "ACELERACIÓN (g)")
        for lbl, val, unit, clr in [("Acc X", st['acc_x'], "g", (255,95,95)),
                                     ("Acc Y", st['acc_y'], "g", (95,220,95)),
                                     ("Acc Z", st['acc_z'], "g", (95,135,255))]:
            frac = math.copysign(min(abs(val)/2.0, 1.0), val)
            self._bar(surf, px+162, self._y+4, 105, 13, frac, clr)
            surf.blit(self.f_md.render(f"{lbl}: {val:+7.4f}{unit}", True, clr),
                      (px+10, self._y))
            self._y += 24
        self._y += 10

        # ── Horizonte artificial ──────────────────────────────────────────
        self._section(surf, "HORIZONTE ARTIFICIAL")
        ah_r  = 68
        ah_cx = px + PANEL_W // 2
        ah_cy = self._y + ah_r + 4
        self._artificial_horizon(surf, ah_cx, ah_cy, ah_r, st['roll'], st['pitch'])
        self._y = ah_cy + ah_r + 14

        # ── Rumbo magnético ───────────────────────────────────────────────
        self._section(surf, "RUMBO MAGNÉTICO")
        r_c  = 68
        c_cx = px + PANEL_W//2
        c_cy = self._y + r_c + 6
        self._compass(surf, c_cx, c_cy, r_c, st['yaw'], self.f_sm)

        dirs  = ['N','NE','E','SE','S','SO','O','NO']
        hdg   = st['yaw'] % 360
        di    = int((hdg + 22.5) / 45) % 8
        deg_s = self.f_lg.render(f"{hdg:05.1f}°", True, (255,210,55))
        dir_s = self.f_lg.render(dirs[di],         True, (255,210,55))
        surf.blit(deg_s, (c_cx - deg_s.get_width()//2, c_cy + r_c + 6))
        surf.blit(dir_s, (c_cx - dir_s.get_width()//2, c_cy + r_c + 30))
        self._y = c_cy + r_c + 60

        # ── Altímetro barométrico (GY-68) ─────────────────────────────────
        self._section(surf, "ALTÍMETRO BAROMÉTRICO")

        # Altitud — número grande centrado en el panel
        alt = st['baro_alt']
        alt_surf = self.f_lg.render(f"ALT: {alt:+.2f} m", True, (75, 210, 255))
        surf.blit(alt_surf, (px + (PANEL_W - alt_surf.get_width()) // 2, self._y))
        self._y += 28

        # Barra de altitud centrada en 0 (escala ±20 m)
        frac_alt = math.copysign(min(abs(alt) / 20.0, 1.0), alt)
        cx_bar = px + PANEL_W // 2
        # Línea base tenue que muestra el rango completo
        pygame.draw.line(surf, (35, 60, 95),
                         (cx_bar - 122, self._y + 5), (cx_bar + 122, self._y + 5), 1)
        # Marca de cero
        pygame.draw.line(surf, (75, 210, 255),
                         (cx_bar, self._y - 1), (cx_bar, self._y + 11), 1)
        self._bar(surf, cx_bar, self._y, 122, 10, frac_alt, (75, 210, 255))
        self._y += 18

        # Velocidad vertical — cambia de color según sube/baja
        vz = st['vel_z']
        frac_vz = math.copysign(min(abs(vz) / 5.0, 1.0), vz)
        col_vz = (80, 240, 100) if vz >= 0 else (255, 115, 50)
        self._bar(surf, px + 155, self._y + 4, 118, 12, frac_vz, col_vz)
        surf.blit(self.f_md.render(f"VelZ: {vz:+.3f} m/s", True, col_vz),
                  (px + 10, self._y))
        self._y += 22

        # Presión barométrica
        press = st['baro_press']
        if press > 50000:
            surf.blit(self.f_sm.render(f"Press: {int(press)} Pa", True, (65, 115, 150)),
                      (px + 10, self._y))
        self._y += 18

        # Temperatura BMP180 (si disponible)
        temp = st['baro_temp']
        if temp is not None:
            t_col = (255, 160, 60) if temp >= 30 else (100, 200, 255) if temp < 15 else (160, 220, 140)
            surf.blit(self.f_sm.render(f"Temp:  {temp:.1f} °C", True, t_col),
                      (px + 10, self._y))
            self._y += 18

        # ── Pie ───────────────────────────────────────────────────────────
        self._y += 10
        pygame.draw.line(surf,(55,75,135),(px+8,self._y),(px+PANEL_W-8,self._y),1)
        self._y += 8
        surf.blit(self.f_sm.render(f"FPS: {fps:.0f}", True,(75,85,115)),(px+10,self._y))
        self._y += 18
        surf.blit(self.f_sm.render("[ESC]Salir [R]Cám [C]Calib.mag [T]Consola", True,(65,75,105)),
                  (px+10,self._y))

# ─────────────────────────────────────────────────────────────────────────────
# Overlay de estado + consola de mensajes (esquina superior izquierda)
# ─────────────────────────────────────────────────────────────────────────────
def _msg_color(m: str):
    u = m.upper()
    if 'ERR' in u:                       return (255, 95, 95)
    if 'CAL' in u or 'ALIGN' in u:       return (240, 210, 90)
    if 'OK' in u or 'AP:' in u:          return (90, 220, 120)
    return (155, 175, 215)

def render_status_overlay(surf, st, f_status, f_msg, messages):
    x, y   = VP_X0 + 10, 10
    w      = 636
    n      = len(messages)
    head_h = 56
    box_h  = head_h + n * 15 + 8

    # Fondo semitransparente para legibilidad sobre la escena 3D
    box = pygame.Surface((w, box_h), pygame.SRCALPHA)
    box.fill((8, 10, 24, 205))
    surf.blit(box, (x, y))
    pygame.draw.rect(surf, (55, 75, 135), (x, y, w, box_h), 1)

    # Línea de estado de conexión
    ok  = st['connected']
    col = (0, 215, 95) if ok else (255, 65, 65)
    surf.blit(f_status.render("● CONECTADO" if ok else "● DESCONECTADO", True, col),
              (x + 10, y + 8))

    # Línea de instrucciones (debajo del estado)
    surf.blit(f_msg.render("[C] Calibrar magnetómetro durante la calibración inicial",
                           True, (185, 205, 120)), (x + 10, y + 32))

    # Separador
    pygame.draw.line(surf, (45, 60, 110), (x + 8, y + 50), (x + w - 8, y + 50), 1)

    # Consola de mensajes (más antiguos arriba, más recientes abajo)
    my = y + 56
    for m in messages:
        surf.blit(f_msg.render(m[:108], True, _msg_color(m)), (x + 10, my))
        my += 15

# ─────────────────────────────────────────────────────────────────────────────
# Estima de posición por dead-reckoning (mapa de la izquierda)
# ─────────────────────────────────────────────────────────────────────────────
_G        = 9.80665   # m/s² por g
# ── Ajuste del peso de las aceleraciones en la posición ──────────────────────
# Baja ACC_GAIN si el avión "vuela" demasiado lejos de lo real; sube si se mueve
# de menos. VEL_DAMP < 1 sangra la velocidad cada frame para frenar la deriva.
ACC_GAIN  = 0.02      # peso de las aceleraciones (↓ = menos movimiento)
VEL_DAMP  = 0.90      # amortiguación de velocidad por frame (↓ = menos deriva)

class DeadReckoning:
    """Infiere la posición del avión integrando las aceleraciones.
    - X (Este) e Y (Norte): doble integración de acc_x/acc_y (cuerpo) rotadas
      por el rumbo (yaw) al marco mundo. Se usa solo el PRIMER DECIMAL de cada
      aceleración (deadband) → en reposo no hay deriva.
    - Z (altura): promedio de la altura barométrica y la doble integración de
      acc_z (componente vertical, quitada la gravedad).
    Origen (0,0,0) fijado en la fase de calibración."""

    def __init__(self):
        self.reset(0.0)

    def reset(self, z_ref=0.0):
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self._z_acc = 0.0
        self.z_ref  = z_ref          # altura barométrica en el instante del origen
        self.trail  = deque(maxlen=4000)
        self.trail.append((0.0, 0.0))

    def update(self, dt, ax, ay, az, baro_alt, yaw_deg):
        if dt <= 0.0 or dt > 0.5:    # ignora el primer frame y saltos largos
            return

        # Solo el primer decimal (deadband ~0.05 g): mata el ruido en reposo.
        # ACC_GAIN reduce el peso de las aceleraciones para que la posición sea realista.
        afx = round(ax, 1) * _G * ACC_GAIN   # longitudinal (morro)  cuerpo
        afy = round(ay, 1) * _G * ACC_GAIN   # lateral (ala dcha.)   cuerpo

        # Rotar cuerpo → mundo según rumbo (0°=Norte, horario)
        psi     = math.radians(yaw_deg)
        a_east  = afx * math.sin(psi) + afy * math.cos(psi)
        a_north = afx * math.cos(psi) - afy * math.sin(psi)

        self.vx = (self.vx + a_east  * dt) * VEL_DAMP
        self.vy = (self.vy + a_north * dt) * VEL_DAMP
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Vertical (filtro complementario): el BARÓMETRO da la altura absoluta y
        # el acelerómetro solo una pequeña corrección de alta frecuencia.
        a_up = (az + 1.0) * _G * ACC_GAIN
        self.vz = (self.vz + a_up * dt) * VEL_DAMP
        self._z_acc = (self._z_acc + self.vz * dt) * 0.99   # decae lento hacia 0
        baro    = baro_alt - self.z_ref
        self.z  = baro + self._z_acc

        # Añadir al rastro solo si se ha movido lo suficiente (mapa limpio)
        lx, ly = self.trail[-1]
        if (self.x - lx) ** 2 + (self.y - ly) ** 2 > 0.0004:   # >0.02 m
            self.trail.append((self.x, self.y))


def _nice_step(span):
    """Paso de rejilla 'bonito' (1,2,5×10ⁿ) ≈ span/5."""
    raw = max(span / 5.0, 1e-6)
    p   = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if m * p >= raw:
            return m * p
    return 10 * p


def _render_map(surf, x, y, w, h, dr, st, f_sm):
    surf.set_clip(pygame.Rect(x, y, w, h))
    pygame.draw.rect(surf, (6, 18, 12), (x, y, w, h))

    # Límites autoescala (rastro + actual + origen)
    pts  = list(dr.trail) + [(dr.x, dr.y), (0.0, 0.0)]
    xs   = [p[0] for p in pts]; ys = [p[1] for p in pts]
    minx, maxx = min(xs), max(xs); miny, maxy = min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 0.5) * 1.15   # escala mínima 0.5 m
    cxw  = (minx + maxx) / 2.0;  cyw = (miny + maxy) / 2.0
    pad  = 14
    scale = (min(w, h) - 2 * pad) / span
    cx, cy = x + w / 2.0, y + h / 2.0

    def P(wx, wy):
        return (int(cx + (wx - cxw) * scale), int(cy - (wy - cyw) * scale))

    # Rejilla
    step = _nice_step(span)
    half = span / 2.0 * 1.1
    k = math.floor((cxw - half) / step)
    while k * step <= cxw + half:
        sx = P(k * step, cyw)[0]
        pygame.draw.line(surf, (22, 42, 30), (sx, y), (sx, y + h), 1)
        k += 1
    k = math.floor((cyw - half) / step)
    while k * step <= cyw + half:
        sy = P(cxw, k * step)[1]
        pygame.draw.line(surf, (22, 42, 30), (x, sy), (x + w, sy), 1)
        k += 1

    # Origen (cruz ámbar)
    ox, oy = P(0.0, 0.0)
    pygame.draw.line(surf, (140, 100, 45), (ox - 6, oy), (ox + 6, oy), 1)
    pygame.draw.line(surf, (140, 100, 45), (ox, oy - 6), (ox, oy + 6), 1)

    # Rastro
    if len(dr.trail) >= 2:
        pygame.draw.lines(surf, (70, 205, 125), False, [P(px, py) for px, py in dr.trail], 2)

    # Posición actual + flecha de rumbo
    px, py = P(dr.x, dr.y)
    psi    = math.radians(st['yaw'])
    hx, hy = px + int(15 * math.sin(psi)), py - int(15 * math.cos(psi))
    pygame.draw.line(surf, (255, 210, 55), (px, py), (hx, hy), 2)
    pygame.draw.circle(surf, (255, 80, 80), (px, py), 4)

    surf.set_clip(None)
    pygame.draw.rect(surf, (40, 70, 50), (x, y, w, h), 1)
    surf.blit(f_sm.render(f"{step:g} m/div   N↑", True, (120, 165, 120)), (x + 5, y + h - 16))


def _render_position_window(surf, x, y, w, dr, st, f_md, f_sm):
    pygame.draw.line(surf, (55, 75, 135), (x, y + 10), (x + w, y + 10), 1)
    surf.blit(f_sm.render("POSICIÓN (m · ref. 0,0,0 en calib.)", True, (110, 160, 230)), (x, y))
    y += 26
    for lbl, val, clr in [("X Este ", dr.x, (255, 110, 110)),
                          ("Y Norte", dr.y, (110, 230, 110)),
                          ("Z Alt. ", dr.z, (110, 160, 255))]:
        surf.blit(f_md.render(f"{lbl}: {val:+8.2f} m", True, clr), (x, y))
        y += 25
    y += 4
    dist = math.hypot(dr.x, dr.y)
    spd  = math.hypot(dr.vx, dr.vy)
    surf.blit(f_sm.render(f"Dist. horiz.: {dist:7.2f} m",   True, (170, 190, 220)), (x, y)); y += 18
    surf.blit(f_sm.render(f"Vel. horiz.:  {spd:7.2f} m/s",  True, (170, 190, 220)), (x, y)); y += 18


def render_map_panel(surf, f_title, f_md, f_sm, dr, st):
    pygame.draw.rect(surf, (10, 12, 30), (0, 0, MAP_W, WIN_H))
    pygame.draw.line(surf, (55, 75, 145), (MAP_W, 0), (MAP_W, WIN_H), 2)

    y = 14
    t = f_title.render("MAPA / POSICIÓN", True, (85, 175, 255))
    surf.blit(t, ((MAP_W - t.get_width()) // 2, y)); y += 32

    m_pad = 12
    m_w   = MAP_W - 2 * m_pad
    _render_map(surf, m_pad, y, m_w, m_w, dr, st, f_sm)
    y += m_w + 18

    _render_position_window(surf, m_pad, y, m_w, dr, st, f_md, f_sm)

# ─────────────────────────────────────────────────────────────────────────────
# Fondo (gradiente de cielo, pre-renderizado)
# ─────────────────────────────────────────────────────────────────────────────
def make_sky(w, h):
    surf = pygame.Surface((w, h))
    for y in range(h):
        t = y / h
        pygame.draw.line(surf, (int(8+28*t), int(10+22*t), int(24+28*t)), (0,y), (w,y))
    return surf

# ─────────────────────────────────────────────────────────────────────────────
# Bucle principal
# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Visualizador 3D IMU ESP32-S3-Nano")
    ap.add_argument('--port', help="Puerto serie (ej. /dev/ttyACM0, COM3)")
    ap.add_argument('--udp',  action='store_true', help=f"Recibir datos por WiFi UDP (puerto {UDP_PORT})")
    ap.add_argument('--demo', action='store_true', help="Modo demo sin hardware")
    args = ap.parse_args()

    if args.demo:
        print("[Demo] Ejecutando en modo demostración (sin hardware).")
        threading.Thread(target=demo_runner, daemon=True).start()
    elif args.udp:
        threading.Thread(target=udp_receiver, daemon=True).start()
    else:
        if not SERIAL_OK:
            print("Instala pyserial:  pip install pyserial"); sys.exit(1)
        if args.port:
            port = args.port
        else:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            port = next((p for p in ports if 'ACM' in p), None) or (ports[0] if ports else '/dev/ttyACM0')
        if not port:
            print("No se encontró puerto serie.")
            print("Usa  --port /dev/ttyACM0  o  --udp  o  --demo")
            sys.exit(1)
        threading.Thread(target=serial_reader, args=(port,), daemon=True).start()

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("IMU Guía Inercial — Visualizador 3D (ESP32-S3 Nano)")

    sky    = make_sky(VP_W, WIN_H)
    panel  = Panel()
    clock  = pygame.time.Clock()
    focal  = (VP_W / 2) / math.tan(math.radians(22.5))

    # Fuentes del overlay de estado/consola (esquina superior izquierda)
    f_ov_status = pygame.font.SysFont('monospace', 15, bold=True)
    f_ov_msg    = pygame.font.SysFont('monospace', 12)

    # Fuentes de la franja de mapa / posición (izquierda)
    f_map_title = pygame.font.SysFont('monospace', 18, bold=True)
    f_map_md    = pygame.font.SysFont('monospace', 16)
    f_map_sm    = pygame.font.SysFont('monospace', 12)

    # Estima de posición por dead-reckoning
    dr      = DeadReckoning()
    last_dr = time.time()

    # Estado de cámara (esférico)
    cam_yaw, cam_pitch, cam_dist = 200.0, 22.0, 11.0
    DEF_CAM = (200.0, 22.0, 11.0)
    dragging  = False
    last_pos  = (0, 0)
    show_overlay = True   # ventanita de estado/consola (tecla T)

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit()
                if ev.key == pygame.K_r:
                    cam_yaw, cam_pitch, cam_dist = DEF_CAM
                if ev.key == pygame.K_c:
                    send_command('C')   # disparar calibración del magnetómetro
                if ev.key == pygame.K_t:
                    show_overlay = not show_overlay   # ocultar/mostrar ventanita
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                if VP_X0 <= ev.pos[0] < VP_X0 + VP_W:
                    dragging = True; last_pos = ev.pos
            if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                dragging = False
            if ev.type == pygame.MOUSEMOTION and dragging:
                dx = ev.pos[0] - last_pos[0]; dy = ev.pos[1] - last_pos[1]
                cam_yaw   += dx * 0.45
                cam_pitch  = max(-80, min(80, cam_pitch - dy * 0.30))
                last_pos   = ev.pos
            if ev.type == pygame.MOUSEWHEEL:
                cam_dist = max(4.0, min(28.0, cam_dist - ev.y * 0.6))

        st = _get()

        # ── Dead-reckoning: integrar posición con el dt real del frame ───────
        now = time.time()
        dt_dr, last_dr = now - last_dr, now
        if _flags['reset_origin']:
            dr.reset(z_ref=st['baro_alt'])
            _flags['reset_origin'] = False
            _log("Origen de posición fijado en (0,0,0)")
        dr.update(dt_dr, st['acc_x'], st['acc_y'], st['acc_z'], st['baro_alt'], st['yaw'])

        # Calcular posición de cámara
        cy_r = math.radians(cam_pitch)
        ca_r = math.radians(cam_yaw)
        ex   = cam_dist * math.cos(cy_r) * math.sin(ca_r)
        ey   = cam_dist * math.sin(cy_r)
        ez   = cam_dist * math.cos(cy_r) * math.cos(ca_r)
        eye  = (ex, ey, ez)
        cr, cu, cf = look_at(eye)

        # Matriz de rotación IMU
        R = imu_matrix(st['roll'], st['pitch'], st['yaw'])

        # ── Dibujado ──────────────────────────────────────────────────────
        screen.blit(sky, (VP_X0, 0))
        render_grid(screen, eye, cr, cu, cf, focal)
        render_airplane(screen, R, eye, cr, cu, cf, focal)
        render_mini_axes(screen, cr, cu)
        render_g_meter(screen, st['acc_x'], st['acc_y'])
        if show_overlay:
            render_status_overlay(screen, st, f_ov_status, f_ov_msg, _get_messages(16))
        render_map_panel(screen, f_map_title, f_map_md, f_map_sm, dr, st)
        panel.draw(screen, st, clock.get_fps())

        pygame.display.flip()
        clock.tick(FPS)


if __name__ == '__main__':
    main()
