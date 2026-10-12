#!/usr/bin/env python3
"""
Visualizador do stream MJPEG do ESP32-CAM.

Uso:
    python3 viewer.py 192.168.0.42
    python3 viewer.py http://192.168.0.42/stream
    python3 viewer.py 192.168.0.42 --save-dir ./capturas

Teclas na janela:
    q / ESC  sair
    s        salvar o frame atual em JPEG

Requisitos:
    pip install opencv-python requests numpy
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import requests


def normalize_url(target: str) -> str:
    """Aceita '192.168.0.42', 'esp32cam.local' ou uma URL completa."""
    if target.startswith(("http://", "https://")):
        return target
    return f"http://{target}/stream"


def mjpeg_frames(url: str, timeout: float = 10.0, chunk_size: int = 4096):
    """Gera frames JPEG (bytes) a partir de um stream multipart MJPEG."""
    with requests.get(url, stream=True, timeout=timeout) as resp:
        resp.raise_for_status()
        buf = bytearray()
        for chunk in resp.iter_content(chunk_size=chunk_size):
            if not chunk:
                continue
            buf.extend(chunk)

            # Procura o par de marcadores JPEG SOI (ffd8) / EOI (ffd9).
            while True:
                start = buf.find(b"\xff\xd8")
                if start < 0:
                    # Nenhum inicio de imagem ainda: descarta cabecalhos acumulados.
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                end = buf.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start > 0:
                        del buf[:start]
                    break
                frame = bytes(buf[start:end + 2])
                del buf[:end + 2]
                yield frame


def run(url: str, save_dir: str | None, scale: float, headless: bool) -> int:
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    window = "ESP32-CAM"
    if not headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    frames = 0
    fps = 0.0
    t0 = time.time()
    last_frame = None

    while True:  # laco externo: reconecta se o stream cair
        try:
            print(f"Conectando em {url} ...")
            for jpeg in mjpeg_frames(url):
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue  # frame corrompido, segue adiante
                last_frame = img

                frames += 1
                if frames % 15 == 0:
                    now = time.time()
                    fps = 15.0 / (now - t0)
                    t0 = now

                if headless:
                    if frames % 30 == 0:
                        print(f"{frames} frames | {fps:.1f} fps | {img.shape[1]}x{img.shape[0]}")
                    continue

                view = img
                if scale != 1.0:
                    view = cv2.resize(
                        img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR
                    )
                cv2.putText(
                    view, f"{fps:.1f} fps  {img.shape[1]}x{img.shape[0]}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
                cv2.imshow(window, view)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    cv2.destroyAllWindows()
                    return 0
                if key == ord("s") and last_frame is not None:
                    path = os.path.join(
                        save_dir or ".", time.strftime("cam_%Y%m%d_%H%M%S.jpg")
                    )
                    cv2.imwrite(path, last_frame)
                    print(f"Salvo: {path}")

        except KeyboardInterrupt:
            if not headless:
                cv2.destroyAllWindows()
            return 0
        except (requests.RequestException, OSError) as exc:
            print(f"Stream interrompido ({exc}). Nova tentativa em 2s...", file=sys.stderr)
            time.sleep(2)


def main() -> int:
    p = argparse.ArgumentParser(description="Visualiza o stream MJPEG do ESP32-CAM.")
    p.add_argument("target", help="IP, hostname ou URL completa do stream")
    p.add_argument("--save-dir", help="pasta onde salvar os snapshots (tecla s)")
    p.add_argument("--scale", type=float, default=1.0, help="fator de zoom da janela")
    p.add_argument("--headless", action="store_true",
                   help="sem janela, apenas imprime estatisticas")
    args = p.parse_args()
    return run(normalize_url(args.target), args.save_dir, args.scale, args.headless)


if __name__ == "__main__":
    sys.exit(main())
