"""Deteksi lintasan dari kamera: BEV -> marka (putih ATAU hitam) -> penelusur garis -> peran -> garis tengah.

Lintasan KSTI: dua garis tepi utuh + satu garis tengah putus-putus, lebar total ~50 cm, tikungan
sampai radius ~0.5 m (contoh panitia). Karena itu garis disimpan sebagai polyline (urutan titik),
bukan polinom y(x): polinom tak mampu mewakili belokan 90 derajat di dalam pandangan kamera.

Alur per frame:
 1. BEV (bird's-eye) via homografi kamera -> tanah.
 2. Marka = struktur tipis yang lebih terang (top-hat) atau lebih gelap (black-hat) dari lantai;
    polaritas dipilih otomatis dari kecerahan lantai.
 3. Penelusur: mulai dari piksel marka terdekat, melangkah mengikuti arah garis (orientasi apa pun),
    melompati celah garis putus-putus. Cakupan (coverage) ~1 = garis utuh, ~0.6 = putus-putus.
 4. Peran garis lewat konsensus: tiap garis dicoba sebagai tepi kiri/tengah/tepi kanan; hipotesis
    yang paling banyak disetujui garis lain (dan pola putus-putus) menang; frame lalu = penalti lunak.
 5. Garis tengah = garis berperan digeser sepanjang normalnya, digabung.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class LaneConfig:
    track_width: float = 0.50      # lebar antar garis tepi [m]
    x_min: float = 0.45            # jangkauan BEV dari poros belakang [m]
    hood_row: int = 300            # baris citra kamera mulai moncong mobil terlihat (0 = tidak ada); cek /autopilot/debug
    x_max: float = 1.50
    y_half: float = 0.75
    res: float = 0.005             # m/piksel BEV
    polarity: str = "auto"         # auto | white | black
    line_max_width: float = 0.06   # marka lebih sempit dari ini dianggap garis (top-hat)
    min_contrast: int = 25
    step_m: float = 0.04           # langkah penelusur garis [m]
    search_r: float = 0.035        # radius cari centroid tiap langkah [m]
    max_gap_m: float = 0.24        # celah terpanjang yang dilompati (garis putus-putus) [m]
    max_lines: int = 6
    min_pix: int = 6
    min_hits: int = 3
    dashed_below: float = 0.85     # coverage di bawah ini = garis putus-putus
    role_tol: float = 0.09         # toleransi pencocokan peran garis [m]
    init_center: float = 0.0       # posisi garis tengah lintasan saat start (+ = kiri mobil)


@dataclass
class Line:
    pts: np.ndarray            # (N,2) polyline berurutan menjauhi mobil, meter
    hits: int
    coverage: float            # ~1 garis utuh, ~0.6 putus-putus
    role: float | None = None  # offset dari garis tengah: +W/2 tepi kiri, 0 tengah, -W/2 tepi kanan

    @property
    def weight(self):
        return self.hits * self.coverage


@dataclass
class LaneResult:
    path: np.ndarray        # (N,2) garis tengah lintasan di frame mobil, berurutan menjauh
    conf: float             # 0..1
    lines: list
    polarity: str
    bev: np.ndarray = field(repr=False)
    valid: np.ndarray = field(repr=False)
    mask: np.ndarray = field(repr=False)


# ---------- utilitas polyline ----------

def normals(p):
    """Normal kiri satuan tiap titik polyline."""
    if len(p) < 2:
        return np.tile([0.0, 1.0], (len(p), 1))
    t = np.gradient(p, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    return np.stack([-t[:, 1], t[:, 0]], 1)


def offset_polyline(p, off):
    return p + off * normals(p)


def signed_offset(q, p, max_along=0.12):
    """Offset lateral bertanda titik q terhadap polyline p (+ = kiri). NaN bila di luar ujung p."""
    if len(p) < 2 or len(q) == 0:
        return np.full(len(q), np.nan)
    d2 = ((q[:, None, :] - p[None, :, :]) ** 2).sum(-1)
    k = d2.argmin(1)
    n = normals(p)[k]
    t = np.stack([n[:, 1], -n[:, 0]], 1)
    r = q - p[k]
    lat = (r * n).sum(1)
    along = (r * t).sum(1)
    inside = ((k > 0) & (k < len(p) - 1)) | (np.abs(along) < max_along)
    return np.where(inside, lat, np.nan)


def resample(p, step=0.03):
    if len(p) < 2:
        return p
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    ss = np.arange(0, s[-1] + 1e-9, step)
    return np.stack([np.interp(ss, s, p[:, 0]), np.interp(ss, s, p[:, 1])], 1)


def curvature(p, span=0.2):
    """Kelengkungan |dθ/ds| sepanjang polyline (dihaluskan sepanjang `span` m)."""
    if len(p) < 3:
        return np.zeros(len(p))
    seg = np.diff(p, axis=0)
    th = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    ds = np.maximum(np.linalg.norm(seg, axis=1), 1e-6)
    m = max(1, int(span / ds.mean()))
    k = np.abs(th[m:] - th[:-m]) / (ds.mean() * m) if len(th) > m else np.abs(np.diff(th)) / ds[1:]
    return k


class LaneDetector:
    def __init__(self, cam, cfg: LaneConfig | None = None):
        self.cfg = c = cfg or LaneConfig()
        self.cam = cam
        self.Hb, self.B, self.size = cam.bev_homography(c.x_min, c.x_max, c.y_half, c.res)
        ones = np.full((cam.h, cam.w), 255, np.uint8)
        if c.hood_row > 0:
            ones[c.hood_row:] = 0  # moncong mobil sendiri: tepinya terbaca sebagai garis melintang
        v = cv2.warpPerspective(ones, self.Hb, self.size, flags=cv2.WARP_INVERSE_MAP | cv2.INTER_NEAREST)
        self.valid = cv2.erode(v, np.ones((5, 5), np.uint8)) > 0
        k = int(round(c.line_max_width / c.res)) | 1
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        W, H = self.size
        self.row_x = c.x_max - (np.arange(H) + 0.5) * c.res
        self.col_y = c.y_half - (np.arange(W) + 0.5) * c.res
        self.reset()

    def reset(self, center=None):
        c = self.cfg
        xs = np.arange(0.0, c.x_max + 1e-9, 0.03)
        self.center = np.stack([xs, np.full_like(xs, c.init_center if center is None else center)], 1)
        self.trust = 1.0

    def predict(self, ds, dyaw):
        """Geser garis tengah frame lalu sesuai gerak mobil (maju ds, belok dyaw)."""
        x, y = self.center[:, 0] - ds, self.center[:, 1]
        c, s = np.cos(-dyaw), np.sin(-dyaw)
        self.center = np.stack([c * x - s * y, s * x + c * y], 1)

    def bev(self, bgr):
        return cv2.warpPerspective(bgr, self.Hb, self.size, flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR)

    def marking_mask(self, bev):
        c = self.cfg
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        # lantai = median di sekitar garis tengah prediksi; di tepi arena sebagian besar pandangan bisa
        # lantai lain (lebih terang/gelap) yang membalik polaritas
        near = self.corridor(self.center, c.track_width / 2) & self.valid
        src = gray[near] if near.sum() > 200 else gray[self.valid]
        floor = float(np.median(src)) if src.size else 0.0
        pol = c.polarity if c.polarity != "auto" else ("white" if floor < 128 else "black")
        g = gray.copy()
        g[~self.valid] = int(floor)
        op = cv2.MORPH_TOPHAT if pol == "white" else cv2.MORPH_BLACKHAT
        th = cv2.morphologyEx(g, op, self.kernel)
        vals = th[self.valid]
        t = max(c.min_contrast, 0.4 * float(np.percentile(vals, 99.5))) if vals.size else 255
        sat = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)[:, :, 1]
        return (th > t) & (sat < 90) & self.valid, pol, floor

    def corridor(self, path, half):
        """Mask BEV piksel berjarak < half dari polyline path."""
        m = np.zeros(self.valid.shape, np.uint8)
        if len(path) > 1:
            cv2.polylines(m, [self.to_px(path).round().astype(np.int32)], False, 1,
                          max(1, int(2 * half / self.cfg.res)))
        return m > 0

    def to_m(self, colrow):
        c = self.cfg
        return np.stack([c.x_max - (colrow[:, 1] + 0.5) * c.res, c.y_half - (colrow[:, 0] + 0.5) * c.res], 1)

    def to_px(self, p):
        c = self.cfg
        return np.stack([(c.y_half - p[:, 1]) / c.res - 0.5, (c.x_max - p[:, 0]) / c.res - 0.5], 1)

    def _trace(self, P, used, seed):
        """Telusuri satu garis dari piksel P[seed] mengikuti arahnya; tandai piksel terpakai."""
        c = self.cfg
        W, H = self.size
        step, r2 = c.step_m / c.res, (c.search_r / c.res) ** 2
        p = P[seed].astype(float)
        near = np.sum((P - p) ** 2, 1) < 4 * r2
        q = P[near] - P[near].mean(0)
        d = np.linalg.eigh(q.T @ q)[1][:, -1] if len(q) > 2 else np.array([0.0, -1.0])
        if abs(d[1]) < 0.3:   # garis melintang: arahkan ke tengah BEV
            d = d if (W / 2 - p[0]) * d[0] > 0 else -d
        elif d[1] > 0:        # selain itu menjauhi mobil (ke atas BEV)
            d = -d
        cents, sel_all, hits, misses, gaps = [], [], 0, 0, 0
        max_miss = int(c.max_gap_m / c.step_m)
        for k in range(int(3.0 / c.step_m)):
            target = p if k == 0 else p + d * step
            m = np.sum((P - target) ** 2, 1) < r2
            if m.sum() >= c.min_pix:
                cen = P[m].mean(0)
                if hits:
                    nd = cen - p
                    n = np.linalg.norm(nd)
                    if n > 1e-6:
                        nd /= n
                        if nd @ d < 0.5:  # belokan mendadak = bukan garis yang sama
                            break
                        d = 0.6 * d + 0.4 * nd
                        d /= np.linalg.norm(d)
                p = cen
                cents.append(cen)
                sel_all.append(np.nonzero(m)[0])
                hits += 1
                gaps += misses
                misses = 0
            else:
                p = target
                misses += 1
                if misses > max_miss:
                    break
            if not (0 <= p[0] < W and 0 <= p[1] < H) or not self.valid[int(p[1]), int(p[0])]:
                break
        if sel_all:
            used[np.unique(np.concatenate(sel_all))] = True
        used[seed] = True
        if hits < c.min_hits:
            return None
        return Line(resample(self.to_m(np.array(cents))), hits, hits / (hits + gaps))

    def _assign_roles(self, lines):
        c = self.cfg
        roles = (c.track_width / 2, 0.0, -c.track_width / 2)
        near = [ln.pts[:max(2, int(0.25 / 0.03))] for ln in lines]  # bagian dekat paling andal
        wsum = sum(ln.weight for ln in lines) or 1.0
        prior = self.center[self.center[:, 0] >= c.x_min - 0.1][:15]
        best = None
        for ln in lines:
            for r in roles:
                h = offset_polyline(ln.pts, -r)
                assign, score = {}, 0.0
                for j, lj in enumerate(lines):
                    d = signed_offset(near[j], h)
                    if np.isnan(d).all():
                        continue
                    d = float(np.nanmean(d))
                    rj = min(roles, key=lambda o: abs(d - o))
                    if abs(d - rj) < c.role_tol:
                        assign[j] = rj
                        # pola = satu-satunya pembeda tepi vs tengah (jarak keduanya sama 25 cm)
                        dashed_ok = (lj.coverage < c.dashed_below) == (rj == 0.0)
                        judgeable = lj.hits >= 8
                        score += lj.weight * (1.0 if dashed_ok else (0.15 if judgeable else 0.7))
                score *= 1 + 0.5 * (len(assign) - 1)
                dp = signed_offset(prior, h)
                dprior = float(np.nanmean(np.abs(dp))) if not np.isnan(dp).all() else c.track_width / 2
                score -= wsum * 0.35 * self.trust * dprior / (c.track_width / 2)
                if best is None or score > best[0]:
                    best = (score, assign)
        if best:
            for j, r in best[1].items():
                lines[j].role = r

    def detect(self, bgr) -> LaneResult:
        c = self.cfg
        bev = self.bev(bgr)
        mask, pol, _ = self.marking_mask(bev)
        rows, cols = np.nonzero(mask)
        P = np.stack([cols, rows], 1)
        used = np.zeros(len(P), bool)

        lines = []
        for i in np.argsort(-rows, kind="stable"):  # benih: piksel marka terdekat yang belum terpakai
            if len(lines) >= c.max_lines:
                break
            if not used[i]:
                ln = self._trace(P, used, i)
                if ln is not None:
                    lines.append(ln)
        self._assign_roles(lines)

        cands = sorted(((offset_polyline(ln.pts, -ln.role), ln.weight) for ln in lines if ln.role is not None),
                       key=lambda cw: -cw[1])
        conf = 0.0
        if cands:
            base, wb = cands[0]
            acc, wacc = base * wb, np.full(len(base), wb)
            for cp, w in cands[1:]:
                d2 = ((base[:, None] - cp[None]) ** 2).sum(-1)
                k = d2.argmin(1)
                ok = d2[np.arange(len(base)), k] < 0.08 ** 2
                acc[ok] += cp[k[ok]] * w
                wacc[ok] += w
            meas = acc / wacc[:, None]
            # sambung ke arah mobil: dari prediksi frame lalu bila ada, kalau tidak ekstrapolasi lurus
            if meas[0, 0] > c.x_min + 0.02:
                t = meas[1] - meas[0] if len(meas) > 1 else np.array([1.0, 0.0])
                t /= max(np.linalg.norm(t), 1e-9)
                n_ext = int((meas[0, 0] - 0.0) / 0.03)
                ext = meas[0] - t[None] * (np.arange(n_ext, 0, -1)[:, None] * 0.03)
                meas = np.vstack([ext, meas])
            conf = min(1.0, sum(ln.hits for ln in lines if ln.role is not None) / (1.5 * (c.x_max - c.x_min) / c.step_m))
            self.center = meas
        self.trust = 0.8 * self.trust + 0.2 * conf
        path = self.center[self.center[:, 0] >= c.x_min - 0.15]
        return LaneResult(path.copy(), conf, lines, pol, bev, self.valid, mask)

    def draw(self, res: LaneResult):
        """Citra debug BEV: marka, garis berperan, garis tengah."""
        img = res.bev.copy()
        img[res.mask] = (255, 0, 255)
        W = self.cfg.track_width / 2
        col = {W: (255, 128, 0), 0.0: (0, 255, 255), -W: (0, 128, 255)}
        for ln in res.lines:
            self._poly(img, ln.pts, col.get(ln.role, (128, 128, 128)), 2)
        self._poly(img, res.path, (0, 255, 0), 3)
        return img

    def _poly(self, img, p, color, th):
        if len(p) > 1:
            cv2.polylines(img, [self.to_px(p).round().astype(np.int32)], False, color, th)
