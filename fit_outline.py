"""Traces the top shell's rim outline from ref/top_shell_underside.webp and writes
outline.json (points in mm, base coordinates: +X screw end, -Y USB side).

The photo gives the shape; the absolute size comes from the flatbed scan
(ref/scan.jpg), where the shell is 37.0 mm long (its width is clipped there).
Run: python3 fit_outline.py
"""
import json
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

SHELL_LENGTH = 37.0     # mm, from the scan
HARMONICS = 10          # smoothing of the traced contour
POINTS = 96

im = np.asarray(Image.open("ref/top_shell_underside.webp").convert("RGB")).astype(float)
r, b = im[..., 0], im[..., 2]
mask = (r < 120) & (r - b < 60)                       # dark shell, not the orange mat
lab, n = ndi.label(mask)
mask = lab == 1 + np.argmax(ndi.sum(mask, lab, range(1, n + 1)))
mask = ndi.binary_opening(ndi.binary_fill_holes(mask), iterations=6)

ys, xs = np.where(mask)
cu, cv = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2     # bounding-box centre
scale = SHELL_LENGTH / (ys.max() - ys.min() + 1)                    # mm per pixel

edge = mask & ~ndi.binary_erosion(mask)
ev, eu = np.where(edge)
# photo is the shell seen from below: screw end is down, USB side is on the right
# after mirroring -> X = v, Y = -u
x, y = (ev - cv) * scale, -(eu - cu) * scale
th, rad = np.arctan2(y, x), np.hypot(x, y)
order = np.argsort(th); th, rad = th[order], rad[order]
k = np.arange(1, HARMONICS + 1)
A = np.hstack([np.ones((len(th), 1)), np.cos(np.outer(th, k)), np.sin(np.outer(th, k))])
coef = np.linalg.lstsq(A, rad, rcond=None)[0]
t = np.linspace(-np.pi, np.pi, POINTS, endpoint=False)
rr = np.hstack([np.ones((POINTS, 1)), np.cos(np.outer(t, k)), np.sin(np.outer(t, k))]) @ coef
pts = np.c_[rr * np.cos(t), rr * np.sin(t)]
print("fit residual (mm): rms %.3f max %.3f" % (np.sqrt(np.mean((A @ coef - rad) ** 2)),
                                                np.abs(A @ coef - rad).max()))
print("outline size: %.2f x %.2f mm" % (np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
json.dump([[round(float(a), 3), round(float(c), 3)] for a, c in pts], open("outline.json", "w"))
