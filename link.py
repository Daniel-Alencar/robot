"""
Canal de controle PC -> ESP32-CAM, por UDP.

Por que UDP e nao HTTP: o handler do stream MJPEG no ESP32 e bloqueante e
monopoliza o servidor HTTP, entao um GET por comando competiria com o video.
Alem disso, num laco de velocidade o comando atrasado e inutil -- queremos que
ele seja descartado, nao enfileirado, e e exatamente o que UDP faz.

O firmware tem watchdog: sem comando valido por 400 ms, ele para os motores.
Portanto este modulo precisa enviar em ritmo constante, mesmo que o frame
ainda nao tenha mudado.
"""

from __future__ import annotations

import socket
import time


class MotorLink:
    def __init__(self, host: str, port: int = 3333, timeout: float = 0.05):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.last_reply: str = ""
        self.sent = 0
        self.replies = 0

    # ----------------------------------------------------------------- envio
    def _send(self, msg: str, want_reply: bool = True) -> str | None:
        self.sock.sendto(msg.encode("ascii"), self.addr)
        self.sent += 1
        if not want_reply:
            return None
        try:
            data, _ = self.sock.recvfrom(128)
        except (socket.timeout, OSError):
            return None
        self.replies += 1
        self.last_reply = data.decode("ascii", errors="replace").strip()
        return self.last_reply

    def set_speeds(self, left: float, right: float, want_reply: bool = True) -> str | None:
        left = max(-1.0, min(1.0, float(left)))
        right = max(-1.0, min(1.0, float(right)))
        return self._send(f"V {left:.3f} {right:.3f}", want_reply)

    def stop(self) -> str | None:
        return self._send("S")

    def ping(self) -> str | None:
        return self._send("P")

    # -------------------------------------------------------------- utilidade
    def wait_for_board(self, attempts: int = 10) -> bool:
        """Confirma que o ESP32 responde antes de liberar o robo."""
        for _ in range(attempts):
            if self.ping():
                return True
            time.sleep(0.2)
        return False

    @property
    def reply_rate(self) -> float:
        return self.replies / self.sent if self.sent else 0.0

    def close(self) -> None:
        # Sempre tente parar os motores ao encerrar, mesmo apos excecao.
        try:
            for _ in range(3):
                self.stop()
        except OSError:
            pass
        finally:
            self.sock.close()

    def __enter__(self) -> "MotorLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
