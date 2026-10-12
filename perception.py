"""
Deteccao de espaco livre a partir de uma unica camera.

Ideia: com a camera apontada para frente e levemente para baixo, a parte de
baixo da imagem e quase sempre o piso imediatamente a frente do robo. Usamos
essa faixa como *modelo de aparencia do piso* e, para cada coluna da imagem,
subimos pixel a pixel ate a aparencia deixar de casar com o piso. A altura
desse limite indica a que distancia o obstaculo esta: quanto mais baixo na
imagem, mais perto.

Projetando esses limites no plano do chao (homografia de plano, com altura e
inclinacao da camera conhecidas) obtemos uma varredura de distancias por
angulo -- o equivalente a um laser 2D barato e aproximado.

LIMITES REAIS DESTA ABORDAGEM (leia antes de confiar no robo):
  * Piso estampado, tapete com padrao forte ou sombras duras geram falsos
    obstaculos.
  * Vidro, obstaculos muito escuros e objetos da cor do piso passam batido.
  * Obstaculos mais baixos que a borda inferior do campo de visao sao invisiveis.
  * Se o robo encostar de frente numa parede lisa, a propria faixa de
    referencia passa a ser a parede e tudo parece livre. Esse caso e coberto
    pelo detector de travamento em navigator.py, nao pela percepcao.
  * As distancias sao aproximadas e dependem da calibracao de altura/angulo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


# --------------------------------------------------------------------- geometria
@dataclass
class CameraGeometry:
    """Pose e optica da camera, para projetar pixels no plano do chao."""

    height_m: float = 0.12       # altura da lente em relacao ao chao
    # Inclinacao para baixo. Pouco inclinada e melhor: com a lente a 12 cm do
    # chao, 20 graus fazem a imagem inteira cobrir so 0,12 a 0,5 m a frente --
    # alcance curto demais para desviar em tempo. Com 12 graus a mesma imagem
    # cobre de ~0,16 m ate mais de 2 m.
    tilt_deg: float = 12.0       # inclinacao para baixo (0 = horizonte no centro)
    hfov_deg: float = 65.0       # campo de visao horizontal
    vfov_deg: float = 50.0       # campo de visao vertical
    max_range_m: float = 2.5     # alem disso, tratamos como "livre"

    def ground_projection(
        self, cols: np.ndarray, rows: np.ndarray, width: int, height: int
    ) -> tuple[np.ndarray, np.ndarray]:
        """Projeta pixels (col, row) no plano do chao.

        Retorna (distancia_m, azimute_rad). Azimute positivo = direita.
        Pixels acima da linha do horizonte recebem max_range_m.
        """
        fx = (width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        fy = (height / 2.0) / math.tan(math.radians(self.vfov_deg) / 2.0)
        cx, cy = width / 2.0, height / 2.0

        # Direcao do raio no referencial da camera (x direita, y baixo, z frente).
        dx = (cols.astype(np.float64) - cx) / fx
        dy = (rows.astype(np.float64) - cy) / fy
        dz = np.ones_like(dx)

        # Rotaciona pelo pitch: camera inclinada theta para baixo.
        t = math.radians(self.tilt_deg)
        cos_t, sin_t = math.cos(t), math.sin(t)
        wy = cos_t * dy + sin_t * dz
        wz = -sin_t * dy + cos_t * dz
        wx = dx

        # Intersecao com o plano y = altura. wy <= 0 aponta para cima: sem chao.
        dist = np.full_like(dx, self.max_range_m)
        bearing = np.arctan2(wx, np.maximum(wz, 1e-6))

        valid = wy > 1e-6
        if np.any(valid):
            s = self.height_m / wy[valid]
            gx = s * wx[valid]
            gz = s * wz[valid]
            d = np.hypot(gx, gz)
            dist[valid] = np.clip(d, 0.0, self.max_range_m)
            bearing[valid] = np.arctan2(gx, np.maximum(gz, 1e-6))

        return dist, bearing


# --------------------------------------------------------------------- resultado
@dataclass
class Scan:
    """Varredura de espaco livre, estilo laser 2D."""

    bearings: np.ndarray          # rad, crescente da esquerda para a direita
    distances: np.ndarray         # m, distancia livre em cada setor
    confidence: float             # 0..1, qualidade do modelo de piso
    boundary_rows: np.ndarray = field(default_factory=lambda: np.empty(0))
    obstacle_mask: np.ndarray | None = None   # para visualizacao
    work_size: tuple[int, int] = (0, 0)       # (largura, altura) processadas

    def sector(self, half_width_rad: float) -> np.ndarray:
        """Distancias dentro de +-half_width em torno da frente."""
        sel = np.abs(self.bearings) <= half_width_rad
        return self.distances[sel] if np.any(sel) else self.distances


# --------------------------------------------------------------------- detector
class FreeSpaceDetector:
    def __init__(
        self,
        geometry: CameraGeometry | None = None,
        work_width: int = 160,
        n_sectors: int = 31,
        floor_ref_rows: int = 12,
        chroma_k: float = 3.5,
        luma_k: float = 6.0,
        edge_thresh: float = 55.0,
        run_length: int = 3,
    ):
        self.geo = geometry or CameraGeometry()
        self.work_width = work_width
        self.n_sectors = n_sectors
        self.floor_ref_rows = floor_ref_rows
        self.chroma_k = chroma_k      # tolerancia de cor (a,b do LAB)
        self.luma_k = luma_k          # tolerancia de brilho, mais folgada (sombras)
        self.edge_thresh = edge_thresh
        self.run_length = run_length  # pixels seguidos para confirmar obstaculo

    # ----------------------------------------------------------------- interno
    def _obstacle_mask(self, bgr: np.ndarray) -> tuple[np.ndarray, float]:
        blur = cv2.GaussianBlur(bgr, (5, 5), 0)
        lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB).astype(np.float32)
        L, A, B = lab[:, :, 0], lab[:, :, 1], lab[:, :, 2]
        h, w = L.shape

        # Faixa de referencia: piso logo a frente do robo (base, centro).
        r0 = max(0, h - self.floor_ref_rows)
        c0, c1 = int(w * 0.30), int(w * 0.70)
        ref = lab[r0:h, c0:c1, :].reshape(-1, 3)

        med = np.median(ref, axis=0)
        # Escala robusta (MAD), com piso para nao explodir em piso uniforme.
        mad = np.median(np.abs(ref - med), axis=0) * 1.4826
        scale = np.maximum(mad, np.array([2.0, 1.0, 1.0], dtype=np.float32))

        dL = np.abs(L - med[0]) / scale[0]
        dA = np.abs(A - med[1]) / scale[1]
        dB = np.abs(B - med[2]) / scale[2]

        # Bordas fortes no brilho: tipicamente o contorno do obstaculo.
        gx = cv2.Sobel(L, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(L, cv2.CV_32F, 0, 1, ksize=3)
        edge = cv2.magnitude(gx, gy)

        mask = (
            (dA > self.chroma_k)
            | (dB > self.chroma_k)
            | (dL > self.luma_k)
            | (edge > self.edge_thresh)
        ).astype(np.uint8)

        # Remove pontos isolados (ruido do JPEG) sem fechar vaos reais.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # Confianca: faixa de referencia homogenea e pouco "obstaculo" nela.
        ref_noise = float(np.mean(mad[1:]))             # dispersao de cor
        ref_busy = float(np.mean(mask[r0:h, c0:c1]))    # quanto da ref virou obstaculo
        confidence = float(
            np.clip(1.0 - ref_busy * 2.0, 0.0, 1.0)
            * np.clip(1.0 - (ref_noise - 2.0) / 12.0, 0.2, 1.0)
        )
        return mask, confidence

    @staticmethod
    def _first_run_from_bottom(mask: np.ndarray, run: int) -> np.ndarray:
        """Para cada coluna, a linha onde comeca o primeiro obstaculo vindo de baixo.

        Exige `run` pixels consecutivos marcados, para nao disparar com ruido.
        Colunas sem obstaculo recebem 0 (topo da imagem = livre ate o alcance).
        """
        h, w = mask.shape
        f = mask.astype(np.float32)
        # Media movel vertical: 1.0 => janela inteira marcada.
        kernel = np.ones((run, 1), np.float32) / run
        runs = cv2.filter2D(f, -1, kernel, borderType=cv2.BORDER_REPLICATE)
        full = runs > 0.999

        rev = full[::-1, :]
        idx = np.argmax(rev, axis=0)          # primeira ocorrencia vinda de baixo
        found = rev[idx, np.arange(w)]
        center = (h - 1) - idx
        boundary = np.where(found, np.clip(center + run // 2, 0, h - 1), 0)
        return boundary.astype(np.int32)

    # ----------------------------------------------------------------- publico
    def process(self, frame_bgr: np.ndarray) -> Scan:
        h0, w0 = frame_bgr.shape[:2]
        scale = self.work_width / float(w0)
        work = cv2.resize(
            frame_bgr, (self.work_width, max(1, int(round(h0 * scale)))),
            interpolation=cv2.INTER_AREA,
        )
        h, w = work.shape[:2]

        mask, confidence = self._obstacle_mask(work)
        boundary = self._first_run_from_bottom(mask, self.run_length)

        cols = np.arange(w)
        dist, bearing = self.geo.ground_projection(cols, boundary, w, h)

        # Agrupa colunas em setores, pegando o MINIMO de cada um (conservador).
        edges = np.linspace(0, w, self.n_sectors + 1).astype(int)
        s_dist = np.empty(self.n_sectors, dtype=np.float64)
        s_bear = np.empty(self.n_sectors, dtype=np.float64)
        for i in range(self.n_sectors):
            a, b = edges[i], max(edges[i] + 1, edges[i + 1])
            s_dist[i] = float(np.min(dist[a:b]))
            s_bear[i] = float(np.mean(bearing[a:b]))

        order = np.argsort(s_bear)
        return Scan(
            bearings=s_bear[order],
            distances=s_dist[order],
            confidence=confidence,
            boundary_rows=boundary,
            obstacle_mask=mask,
            work_size=(w, h),
        )


# --------------------------------------------------------------------- debug
def draw_overlay(frame_bgr: np.ndarray, scan: Scan) -> np.ndarray:
    """Desenha a mascara de obstaculo e a linha de espaco livre sobre o frame."""
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    if scan.obstacle_mask is None or scan.boundary_rows.size == 0:
        return out

    mw, mh = scan.work_size
    sx, sy = w / float(mw), h / float(mh)

    # Mascara em vermelho translucido.
    big = cv2.resize(scan.obstacle_mask * 255, (w, h), interpolation=cv2.INTER_NEAREST)
    tint = np.zeros_like(out)
    tint[:, :, 2] = big
    out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)

    # Fronteira do espaco livre.
    pts = [
        (int(c * sx), int(scan.boundary_rows[c] * sy))
        for c in range(mw)
        if scan.boundary_rows[c] > 0
    ]
    for p in pts:
        cv2.circle(out, p, 1, (0, 255, 255), -1)

    return out
