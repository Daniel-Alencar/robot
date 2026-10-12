"""Leitura de um stream MJPEG (multipart/x-mixed-replace) do ESP32-CAM."""

import requests


def normalize_url(target: str) -> str:
    """Aceita '192.168.0.42', 'esp32cam.local' ou uma URL completa."""
    if target.startswith(("http://", "https://")):
        return target
    return f"http://{target}/stream"


def host_of(target: str) -> str:
    """Extrai apenas o host, para abrir o socket UDP de controle."""
    t = target
    for prefix in ("http://", "https://"):
        if t.startswith(prefix):
            t = t[len(prefix):]
    t = t.split("/", 1)[0]
    return t.split(":", 1)[0]


def mjpeg_frames(url: str, timeout: float = 10.0, chunk_size: int = 4096):
    """Gera frames JPEG (bytes) a partir de um stream multipart MJPEG.

    Localiza as imagens pelos marcadores SOI (ffd8) e EOI (ffd9) em vez de
    parsear os cabecalhos multipart -- mais tolerante a variacoes de boundary.
    """
    with requests.get(url, stream=True, timeout=timeout) as resp:
        resp.raise_for_status()
        buf = bytearray()
        for chunk in resp.iter_content(chunk_size=chunk_size):
            if not chunk:
                continue
            buf.extend(chunk)

            while True:
                start = buf.find(b"\xff\xd8")
                if start < 0:
                    # Nenhum inicio de imagem ainda: descarta cabecalhos.
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
