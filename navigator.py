#!/usr/bin/env python3
"""
Navegacao autonoma reativa para robo diferencial com ESP32-CAM.

Fluxo:

    ESP32-CAM --MJPEG--> percepcao --> planejador --> UDP --> ESP32 --> motores

Uso:
    python3 navigator.py 10.0.0.200                 # com janela de depuracao
    python3 navigator.py 10.0.0.200 --dry-run       # so observa, nao aciona
    python3 navigator.py 10.0.0.200 --max-speed 0.3 # limita o duty

Teclas na janela:
    ESPACO   arma / desarma o robo (ele comeca DESARMADO)
    b        forca uma manobra de recuo
    q / ESC  sai (parando os motores)

Seguranca:
  * O robo comeca desarmado; nada se move sem ESPACO.
  * Uma thread envia o ultimo comando a 10 Hz, independente da taxa de quadros.
    Isso alimenta o watchdog do firmware sem deixar o controle preso ao video.
  * Se o frame mais recente ficar velho (stream travado), a thread passa a
    mandar parada em vez do ultimo comando.
  * Qualquer excecao, Ctrl+C ou fim do programa para os motores.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import cv2
import numpy as np
import requests

from link import MotorLink
from mjpeg import host_of, mjpeg_frames, normalize_url
from perception import CameraGeometry, FreeSpaceDetector, draw_overlay
from planner import PlannerConfig, ReactivePlanner


# --------------------------------------------------------------------- envio
class CommandPump:
    """Reenvia o ultimo comando em ritmo fixo, para alimentar o watchdog.

    O stream do ESP32-CAM com compressao em software roda a poucos quadros por
    segundo; amarrar o envio de velocidade a esse ritmo faria o watchdog do
    firmware disparar a cada engasgo da rede. Esta thread resolve isso.
    """

    def __init__(self, link: MotorLink, rate_hz: float = 10.0, stale_s: float = 0.5):
        self.link = link
        self.period = 1.0 / rate_hz
        self.stale_s = stale_s
        self._cmd = (0.0, 0.0)
        self._stamp = 0.0
        self._armed = False
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def set(self, left: float, right: float) -> None:
        with self._lock:
            self._cmd = (left, right)
            self._stamp = time.monotonic()

    def set_armed(self, armed: bool) -> None:
        with self._lock:
            self._armed = armed
            if not armed:
                self._cmd = (0.0, 0.0)

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            with self._lock:
                left, right = self._cmd
                armed = self._armed
                age = time.monotonic() - self._stamp
            if not armed or age > self.stale_s:
                left = right = 0.0     # desarmado ou imagem velha: parado
            try:
                self.link.set_speeds(left, right, want_reply=False)
            except OSError:
                pass
            self._stop_evt.wait(self.period)

    def shutdown(self) -> None:
        self.set_armed(False)
        self._stop_evt.set()
        self._thread.join(timeout=1.0)


# --------------------------------------------------------- detector de travamento
class StuckDetector:
    """Deteta robo preso comparando quadros consecutivos.

    Cobre o pior caso da percepcao monocular: encostar de frente numa parede
    lisa, onde a faixa de referencia do piso passa a ser a propria parede e
    tudo parece livre. Se mandamos andar e a imagem nao muda, estamos presos.
    """

    def __init__(self, diff_thresh: float = 2.0, frames: int = 8,
                 cooldown_s: float = 3.0):
        self.diff_thresh = diff_thresh
        self.frames = frames
        self.cooldown_s = cooldown_s   # evita reentrar em recuo sem parar
        self.last_diff = 0.0
        self._prev: np.ndarray | None = None
        self._count = 0
        self._last_trigger = -1e9

    def update(self, frame_bgr: np.ndarray, commanding_motion: bool,
               now: float) -> bool:
        small = cv2.cvtColor(
            cv2.resize(frame_bgr, (64, 48), interpolation=cv2.INTER_AREA),
            cv2.COLOR_BGR2GRAY,
        ).astype(np.float32)

        stuck = False
        in_cooldown = (now - self._last_trigger) < self.cooldown_s

        if self._prev is not None and commanding_motion and not in_cooldown:
            self.last_diff = float(np.mean(np.abs(small - self._prev)))
            if self.last_diff < self.diff_thresh:
                self._count += 1
                if self._count >= self.frames:
                    stuck = True
                    self._count = 0
                    self._last_trigger = now
            else:
                self._count = 0
        else:
            self._count = 0

        self._prev = small
        return stuck


# --------------------------------------------------------------------- desenho
def draw_radar(scan, size: int = 180, max_range: float = 2.5) -> np.ndarray:
    """Mini mapa polar do espaco livre, visto de cima."""
    img = np.zeros((size, size, 3), np.uint8)
    cx, cy = size // 2, size - 10
    for r_m in (0.5, 1.0, 1.5, 2.0):
        if r_m <= max_range:
            cv2.circle(img, (cx, cy), int(r_m / max_range * (size - 20)),
                       (40, 40, 40), 1)

    pts = []
    for b, d in zip(scan.bearings, scan.distances):
        rr = d / max_range * (size - 20)
        pts.append((int(cx + rr * math.sin(b)), int(cy - rr * math.cos(b))))
    if len(pts) > 1:
        cv2.polylines(img, [np.array(pts, np.int32)], False, (0, 220, 255), 1)
        poly = np.array([(cx, cy)] + pts, np.int32)
        overlay = img.copy()
        cv2.fillPoly(overlay, [poly], (0, 70, 0))
        img = cv2.addWeighted(img, 0.6, overlay, 0.4, 0)
    cv2.circle(img, (cx, cy), 3, (255, 255, 255), -1)
    return img


def compose_hud(frame, scan, cmd, armed: bool, fps: float, dry: bool,
                stuck_diff: float = 0.0) -> np.ndarray:
    view = draw_overlay(frame, scan)
    if view.shape[1] < 480:
        s = 480.0 / view.shape[1]
        view = cv2.resize(view, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)

    radar = draw_radar(scan, max_range=2.5)
    rh, rw = radar.shape[:2]
    if view.shape[0] >= rh and view.shape[1] >= rw:
        view[0:rh, view.shape[1] - rw:] = radar

    if dry:
        status, color = "DRY-RUN", (0, 200, 255)
    elif armed:
        status, color = "ARMADO", (0, 255, 0)
    else:
        status, color = "DESARMADO (espaco)", (0, 0, 255)

    lines = [
        f"{status}  |  {cmd.state}",
        f"L={cmd.left:+.2f}  R={cmd.right:+.2f}",
        f"{cmd.note}",
        f"conf={scan.confidence:.2f}  diff={stuck_diff:.1f}  {fps:.1f} fps",
    ]
    y = 22
    for i, text in enumerate(lines):
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    color if i == 0 else (255, 255, 255), 2 if i == 0 else 1,
                    cv2.LINE_AA)
        y += 20
    return view


# --------------------------------------------------------------------- principal
def run(args: argparse.Namespace) -> int:
    url = normalize_url(args.target)
    host = host_of(args.target)

    geometry = CameraGeometry(
        height_m=args.cam_height,
        tilt_deg=args.cam_tilt,
        hfov_deg=args.hfov,
        vfov_deg=args.vfov,
        max_range_m=args.max_range,
    )
    detector = FreeSpaceDetector(geometry=geometry)

    cfg = PlannerConfig()
    cfg.v_cruise = min(cfg.v_cruise, args.max_speed)
    cfg.turn_speed = min(cfg.turn_speed, args.max_speed)
    cfg.backup_speed = min(cfg.backup_speed, args.max_speed)
    planner = ReactivePlanner(cfg)
    stuck = StuckDetector(diff_thresh=args.stuck_diff)

    link = MotorLink(host, args.port)
    print(f"Procurando o ESP32 em {host}:{args.port} ...")
    if link.wait_for_board():
        print(f"Placa respondeu: {link.last_reply}")
    elif args.dry_run:
        print("Placa nao respondeu, mas --dry-run nao precisa dela.")
    else:
        print("ERRO: o ESP32 nao respondeu ao ping UDP.", file=sys.stderr)
        print("Verifique IP, porta 3333 e se o firmware novo foi gravado.",
              file=sys.stderr)
        link.close()
        return 1

    pump = CommandPump(link)
    if not args.dry_run:
        pump.start()

    window = "Navegacao ESP32-CAM"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    armed = False
    fps = 0.0
    t_prev = time.monotonic()
    t_fps = t_prev
    frames = 0
    rc = 0

    try:
        while True:
            try:
                print(f"Conectando ao stream {url} ...")
                for jpeg in mjpeg_frames(url):
                    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    now = time.monotonic()
                    dt = max(now - t_prev, 1e-3)
                    t_prev = now

                    scan = detector.process(frame)

                    lc = planner.last_command
                    moving = (
                        armed
                        and planner.state != "BACKUP"
                        and abs(lc[0]) + abs(lc[1]) > 0.1
                    )
                    if stuck.update(frame, moving, now):
                        print("Travamento detectado: recuando.")
                        planner.request_backup(now)

                    cmd = planner.step(scan, dt, now)

                    if armed and not args.dry_run:
                        pump.set(cmd.left, cmd.right)
                    else:
                        pump.set(0.0, 0.0)

                    frames += 1
                    if frames % 10 == 0:
                        fps = 10.0 / (now - t_fps)
                        t_fps = now

                    cv2.imshow(window, compose_hud(frame, scan, cmd, armed, fps,
                                                   args.dry_run, stuck.last_diff))

                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        return rc
                    if key == ord(" "):
                        armed = not armed
                        pump.set_armed(armed and not args.dry_run)
                        planner.reset()
                        print("ARMADO" if armed else "DESARMADO")
                    if key == ord("b"):
                        planner.request_backup(now)

            except KeyboardInterrupt:
                return rc
            except (requests.RequestException, OSError) as exc:
                # Stream caiu: desarma e tenta reconectar. Nunca siga cego.
                armed = False
                pump.set_armed(False)
                print(f"Stream interrompido ({exc}). Motores parados; "
                      "nova tentativa em 2 s.", file=sys.stderr)
                time.sleep(2)

    finally:
        pump.shutdown()
        link.close()
        cv2.destroyAllWindows()
        print("Motores parados, conexao encerrada.")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Navegacao autonoma reativa com ESP32-CAM.")
    p.add_argument("target", help="IP do ESP32-CAM (ou URL completa do stream)")
    p.add_argument("--port", type=int, default=3333, help="porta UDP de controle")
    p.add_argument("--dry-run", action="store_true",
                   help="processa e mostra, mas nao aciona os motores")
    p.add_argument("--max-speed", type=float, default=0.45,
                   help="limite de duty normalizado (0..1)")
    p.add_argument("--stuck-diff", type=float, default=2.0,
                   help="limiar de diferenca entre quadros p/ detectar travamento; "
                        "compare com o valor 'diff' mostrado na janela")

    g = p.add_argument_group("calibracao da camera")
    g.add_argument("--cam-height", type=float, default=0.12,
                   help="altura da lente ao chao, em metros")
    g.add_argument("--cam-tilt", type=float, default=12.0,
                   help="inclinacao da camera para baixo, em graus")
    g.add_argument("--hfov", type=float, default=65.0, help="campo de visao horizontal")
    g.add_argument("--vfov", type=float, default=50.0, help="campo de visao vertical")
    g.add_argument("--max-range", type=float, default=2.5,
                   help="alcance maximo considerado, em metros")

    return run(p.parse_args())


if __name__ == "__main__":
    sys.exit(main())
