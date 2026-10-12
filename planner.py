"""
Planejador reativo para robo diferencial.

Entrada: a varredura de espaco livre produzida por perception.py.
Saida: velocidades normalizadas das duas rodas, em [-1, 1].

E um navegador *reativo* e local: ele desvia do que ve agora. Nao constroi mapa,
nao sabe onde esteve e nao persegue destino -- o robo vagueia pelo ambiente
evitando obstaculos. Mapeamento e navegacao com objetivo (SLAM) sao outro
projeto, e com camera monocular sem encoder nem dariam odometria confiavel.

Maquina de estados:

    CRUISE  --- frente bloqueada ---->  TURN
    TURN    --- frente liberada ----->  CRUISE
    qualquer --- travamento detectado ->  BACKUP --> TURN
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from perception import Scan


@dataclass
class Command:
    left: float          # [-1, 1]
    right: float         # [-1, 1]
    state: str
    v: float = 0.0       # velocidade linear normalizada
    omega: float = 0.0   # velocidade angular normalizada
    note: str = ""


@dataclass
class PlannerConfig:
    # --- velocidades (normalizadas; 1.0 = duty maximo do motor) ---
    v_cruise: float = 0.45         # ritmo em linha reta
    v_min_move: float = 0.18       # abaixo disso o motor nem vence o atrito
    turn_speed: float = 0.40       # magnitude do giro no proprio eixo
    backup_speed: float = 0.35

    # --- geometria do corredor de seguranca (metros) ---
    stop_dist: float = 0.28        # mais perto que isso: para e gira
    clear_dist: float = 0.45       # histerese: so volta a andar com esta folga
    cruise_dist: float = 1.20      # a partir daqui, velocidade cheia
    front_half_angle_deg: float = 22.0   # largura do corredor considerado "frente"

    # --- direcao ---
    turn_penalty: float = 0.55     # m de desconto por rad de desvio do centro
    k_omega: float = 1.30          # ganho de curvatura por rad de erro
    track_ratio: float = 0.85      # quanto do comando vai para o diferencial

    # --- suavizacao e seguranca ---
    slew_per_s: float = 2.5        # variacao maxima de duty por segundo
    min_confidence: float = 0.35   # percepcao pior que isso = trate como bloqueio
    backup_time_s: float = 0.8


class ReactivePlanner:
    def __init__(self, cfg: PlannerConfig | None = None):
        self.cfg = cfg or PlannerConfig()
        self.state = "CRUISE"
        self._turn_dir = 1.0          # +1 gira para a direita, -1 esquerda
        self._backup_until = 0.0
        self._last = (0.0, 0.0)
        self._t = 0.0

    # ------------------------------------------------------------------ util
    def request_backup(self, now: float) -> None:
        """Chamado de fora (detector de travamento) para recuar e girar."""
        self.state = "BACKUP"
        self._backup_until = now + self.cfg.backup_time_s

    def _differential(self, v: float, omega: float, dt: float) -> tuple[float, float]:
        half = self.cfg.track_ratio / 2.0
        left = v - omega * half
        right = v + omega * half

        # Normaliza preservando a razao entre as rodas (mantem a curvatura).
        peak = max(abs(left), abs(right), 1.0)
        left, right = left / peak, right / peak

        # Limita a variacao por segundo: evita pico de corrente e derrapagem.
        max_step = self.cfg.slew_per_s * max(dt, 1e-3)
        pl, pr = self._last
        left = float(np.clip(left, pl - max_step, pl + max_step))
        right = float(np.clip(right, pr - max_step, pr + max_step))
        self._last = (left, right)
        return left, right

    # ------------------------------------------------------------------ passo
    def step(self, scan: Scan, dt: float, now: float) -> Command:
        cfg = self.cfg
        self._t = now

        front = scan.sector(math.radians(cfg.front_half_angle_deg))
        front_dist = float(np.min(front)) if front.size else 0.0

        # Percepcao ruim e tratada como obstaculo: na duvida, nao avance.
        low_conf = scan.confidence < cfg.min_confidence
        blocked = front_dist < cfg.stop_dist or low_conf

        # ----------------------------------------------------------- BACKUP
        if self.state == "BACKUP":
            if now < self._backup_until:
                l, r = self._differential(-cfg.backup_speed, 0.0, dt)
                return Command(l, r, "BACKUP", -cfg.backup_speed, 0.0, "recuando")
            self.state = "TURN"

        # ------------------------------------------------------------- TURN
        if self.state == "TURN":
            if not blocked and front_dist > cfg.clear_dist:
                self.state = "CRUISE"
            else:
                omega = self._turn_dir * cfg.turn_speed * 2.0 / cfg.track_ratio
                l, r = self._differential(0.0, omega, dt)
                side = "direita" if self._turn_dir > 0 else "esquerda"
                return Command(l, r, "TURN", 0.0, omega,
                               f"girando p/ {side} (frente {front_dist:.2f} m)")

        # ----------------------------------------------------------- CRUISE
        if blocked:
            # Entra em giro, escolhendo o lado com mais espaco.
            left_side = scan.distances[scan.bearings < 0]
            right_side = scan.distances[scan.bearings > 0]
            left_free = float(np.mean(left_side)) if left_side.size else 0.0
            right_free = float(np.mean(right_side)) if right_side.size else 0.0
            self._turn_dir = 1.0 if right_free >= left_free else -1.0
            self.state = "TURN"
            omega = self._turn_dir * cfg.turn_speed * 2.0 / cfg.track_ratio
            l, r = self._differential(0.0, omega, dt)
            note = "bloqueado" if not low_conf else "percepcao incerta"
            return Command(l, r, "TURN", 0.0, omega, note)

        # Escolhe a direcao com mais espaco, penalizando desvios do centro.
        dist = scan.distances
        if dist.size >= 3:                      # suaviza para nao mirar em ruido
            dist = np.convolve(dist, np.ones(3) / 3.0, mode="same")
        score = dist - cfg.turn_penalty * np.abs(scan.bearings)
        target = float(scan.bearings[int(np.argmax(score))])

        omega = float(np.clip(cfg.k_omega * target, -1.5, 1.5))

        # Velocidade cai com a proximidade do obstaculo e com a curva fechada.
        span = max(cfg.cruise_dist - cfg.stop_dist, 1e-3)
        v = cfg.v_cruise * float(np.clip((front_dist - cfg.stop_dist) / span, 0.0, 1.0))
        v *= 1.0 - 0.5 * min(abs(target) / (math.pi / 2.0), 1.0)
        v = max(v, cfg.v_min_move)

        l, r = self._differential(v, omega, dt)
        return Command(l, r, "CRUISE", v, omega,
                       f"alvo {math.degrees(target):+.0f} deg, frente {front_dist:.2f} m")

    @property
    def last_command(self) -> tuple[float, float]:
        """Ultimo par (esquerda, direita) efetivamente emitido."""
        return self._last

    def reset(self) -> None:
        self.state = "CRUISE"
        self._last = (0.0, 0.0)
