"""Rumus skor guidebook KSTI 2026 + wasit otomatis untuk simulasi.

Jalankan `python -m ksti_mav.core.score` untuk tabel strategi.
"""
T_STAGE = 300.0  # batas waktu percobaan [s]
PENALTY = {"checkpoint": 0.5, "obstacle": 0.5, "lepas": 0.2, "keluar": 0.8}
STP_ACKERMANN = 0.25


def penalty(kind, n_checkpoints, t_stage=T_STAGE):
    return PENALTY[kind] * t_stage / n_checkpoints


def s_ar(t_total, cp, t_stage=T_STAGE):
    return (1 + max((t_stage - t_total) / t_stage, 0.0)) * cp


def s_arc(t_total, cp, tcp, t_stage=T_STAGE):
    return s_ar(t_total, cp, t_stage) / (2 * tcp)


def total(sar, ackermann=True, sarc=0.0):
    return sar * (1 + (STP_ACKERMANN if ackermann else 0.0) + sarc)


class Referee:
    """Wasit simulasi: progres sepanjang garis tengah, checkpoint berurutan, keluar jalur.

    Keluar jalur = lebih dari `out_frac` lebar mobil berada di luar garis tepi
    (guidebook: 30% untuk Autonomous Race, 50% untuk Challenge).
    """

    def __init__(self, track, car_width=0.19, out_frac=0.30, t_stage=T_STAGE):
        self.tr, self.cw, self.out_frac, self.t_stage = track, car_width, out_frac, t_stage
        n = len(track.center)
        si = track.start_idx
        self.cp_s = sorted(((i - si) % n) * track.length / n for i in track.d["checkpoints_idx"])
        self.reset(0.0)

    def reset(self, t):
        self.t0, self.s_prev, self.progress = t, 0.0, 0.0
        self.cp, self.violations, self.out, self.finished, self.t_total = 0, [], False, False, None

    def update(self, x, y, t):
        """-> list kejadian baru ('checkpoint', 'keluar', 'finish')."""
        ev = []
        if self.finished:
            return ev
        s, lat, w = self.tr.project(x, y)
        ds = (s - self.s_prev + self.tr.length / 2) % self.tr.length - self.tr.length / 2
        self.s_prev = s
        self.progress += ds
        while self.cp < len(self.cp_s) and self.progress >= self.cp_s[self.cp]:
            self.cp += 1
            ev.append("checkpoint")
        outside = (abs(lat) + self.cw / 2 - w / 2) / self.cw
        if outside > self.out_frac and not self.out:
            self.violations.append("keluar")
            ev.append("keluar")
        self.out = outside > self.out_frac
        if self.progress >= self.tr.length - 0.05:
            self.finished, self.t_total = True, t - self.t0
            ev.append("finish")
        if t - self.t0 > self.t_stage:
            self.finished = True
        return ev

    def last_checkpoint_s(self):
        return self.cp_s[self.cp - 1] if self.cp else 0.0

    def result(self, t_now):
        n = len(self.cp_s)
        t = (self.t_total if self.t_total is not None else min(t_now - self.t0, self.t_stage))
        t += sum(penalty(k, n, self.t_stage) for k in self.violations)
        return {"t_total": round(t, 2), "cp": self.cp, "tcp": n, "violations": list(self.violations),
                "finished": self.t_total is not None, "S_AR": round(s_ar(t, self.cp, self.t_stage), 3)}


if __name__ == "__main__":
    n = 8
    print(f"Penalti per pelanggaran (Tcp={n}): " + ", ".join(f"{k} +{penalty(k, n):.1f}s" for k in PENALTY))
    print("\nWaktu  S_AR(semua cp)  Total(+Ackermann)  Total(+Ack+Challenge sama)")
    for t in (20, 30, 45, 60, 90, 120, 300):
        sar = s_ar(t, n)
        print(f"{t:4d}s  {sar:8.2f}        {total(sar):8.2f}           {total(sar, True, s_arc(t, n, n)):8.2f}")
