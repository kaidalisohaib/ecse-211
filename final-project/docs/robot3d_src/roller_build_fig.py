import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
import numpy as np

fig = plt.figure(figsize=(15, 10.4))
ax = fig.add_axes([0.03, 0.34, 0.94, 0.60])
ax.set_aspect("equal"); ax.axis("off")
YC = 0
def rect(x0, x1, y0, y1, **k): ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **k))
# tube (section along the axis)
rect(-10.0, 9.2, 2.8, 2.95, color="#f06a1d"); rect(-10.0, 9.2, -2.95, -2.8, color="#f06a1d")
for y in (2.95, -2.95):
    s = 1 if y > 0 else -1
    for x0 in np.arange(-10.0, 9.2, 2.74):
        rect(x0 + 0.1, min(x0 + 2.64, 9.1), y if s > 0 else y - 1.1, y + 1.1 if s > 0 else y, color="#2f4fd8", alpha=0.75)
# motor + beams (stationary)
rect(-7.84, -0.6, -1.2, 1.2, color="#cfd3d8", ec="k", lw=0.8)
ax.text(-4.2, 0, "EV3 Medium\n(does not turn)", ha="center", va="center", fontsize=9)
for y in (1.6, -1.6):
    rect(-11.0, -0.6, y - 0.4, y + 0.4, color="#9aa0a6", ec="k", lw=0.6)
ax.text(-9.4, 2.25, "4 x 13-hole beams", fontsize=8.5)
rect(-11.0, -10.2, -3.6, 3.6, color="#6b7280", ec="k", lw=0.6); ax.text(-10.6, -4.6, "left frame\nx = -10.6", ha="center", fontsize=8.5)
# wheels (rim + tyre) as hubs
for c, nm in ((1.0, "spare wheel 1\n(rim No.16 + tyre No.17)"), (7.8, "spare wheel 2")):
    rect(c - 1.4, c + 1.4, -2.8, 2.8, color="#26292e", ec="k", lw=0.6)
    rect(c - 1.32, c + 1.32, -2.16, 2.16, color="#b9bec8", ec="k", lw=0.6)
    ax.text(c, -6.2 if c < 5 else -6.2, nm, ha="center", fontsize=9)
    ax.annotate("", (c, -2.9), (c, -5.4), arrowprops=dict(arrowstyle="->"))
# axles
rect(-1.4, 3.4, -0.24, 0.24, color="k"); ax.text(0.2, 0.55, "6L axle", ha="center", fontsize=8.5)
rect(3.4, 10.6, -0.24, 0.24, color="k"); ax.text(5.6, 0.55, "9L axle", ha="center", fontsize=8.5)
rect(2.6, 4.2, -0.42, 0.42, color="#444", ec="k", lw=0.6); ax.text(3.4, -1.0, "axle joiner\n(No.18)", ha="center", fontsize=8)
rect(9.4, 10.2, -3.6, 3.6, color="#6b7280", ec="k", lw=0.6); rect(10.2, 11.0, -3.6, 3.6, color="#4b5563", ec="k", lw=0.6)
ax.text(11.3, -5.2, "right frame\n(2 layers,\nbearing holes)", ha="center", fontsize=8.5)
# tape bands
for x in (1.0, 7.8):
    rect(x - 0.9, x + 0.9, 2.95, 3.25, color="#e5e7eb", ec="#6b7280", lw=0.6, hatch="//")
    rect(x - 0.9, x + 0.9, -3.25, -2.95, color="#e5e7eb", ec="#6b7280", lw=0.6, hatch="//")
ax.text(4.4, 4.6, "tape bands over each wheel (squeeze the tube onto the tyre)", ha="center", fontsize=9)
ax.annotate("", (1.0, 3.3), (3.0, 4.4), arrowprops=dict(arrowstyle="->")); ax.annotate("", (7.8, 3.3), (5.8, 4.4), arrowprops=dict(arrowstyle="->"))
# dims
def dim(x0, x1, y, t):
    ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", lw=0.8)); ax.text((x0 + x1) / 2, y + 0.25, t, ha="center", fontsize=9)
dim(-10.0, 9.2, 5.9, "tube 19.2 cm (flaps the full length)")
dim(-10.0, -0.4, -7.6, "9.6 cm: tube slides over the motor and beams")
dim(-0.4, 2.4, -7.6, "2.8"); dim(6.4, 9.2, -7.6, "2.8 (flush with the end)")
ax.text(-12.5, 0, "axis", va="center", fontsize=8.5, color="#555")
ax.plot([-12, 12], [0, 0], "--", color="#999", lw=0.6)
ax.set_xlim(-13, 12.5); ax.set_ylim(-8.4, 6.6)
ax.set_title("Roller v5.1 (V-angled flaps), section along the axis (cm). Orange = card tube, blue = flaps, black/grey = the two spare wheels", fontsize=11)

# end view
bx = fig.add_axes([0.03, 0.02, 0.28, 0.28]); bx.set_aspect("equal"); bx.axis("off")
bx.add_patch(Circle((0, 0), 2.9, color="#f06a1d")); bx.add_patch(Circle((0, 0), 2.8, color="#26292e")); bx.add_patch(Circle((0, 0), 2.16, color="#b9bec8"))
for k in range(8):
    a = k * np.pi / 4; bx.plot([2.9 * np.cos(a), 4.05 * np.cos(a)], [2.9 * np.sin(a), 4.05 * np.sin(a)], color="#2f4fd8", lw=3)
bx.add_patch(Circle((0, 0), 4.05, fill=False, ls=":", color="#2f4fd8"))
bx.text(0, -4.9, "end view: tube 5.6 inside / ~5.8 outside, 8 flaps 1.15 tall, tip circle 8.1", ha="center", fontsize=8.5)
bx.set_xlim(-4.6, 4.6); bx.set_ylim(-5.3, 4.4)
# steps
tx = fig.add_axes([0.33, 0.0, 0.66, 0.335]); tx.axis("off")
steps = ("How to build it (paper, cardboard and tape only):\n"
 "1. Roll thin card (cereal-box weight) twice around one of the spare wheels so it is a snug sliding fit (5.6 cm inside). Tape the seam along its length. Cut it to 19.2 cm.\n"
 "2. Flaps, V-angled (chevron): 16 strips of card, 10.5 x ~2.2 cm (two per row, one per half). Fold each lengthwise into an L (1 cm foot, 1.15 cm flap).\n"
 "    Draw 8 V lines on the tube, 45 deg apart: each half runs straight (at ~25 deg to the axis) from the tube's middle to its end, the end ~4.4 cm further round in the turning direction.\n"
 "    Tape each foot along its line, flap standing up and leading. Slit the flaps every ~2.7 cm. Ends lead and the centre trails, so pom-poms are pushed toward the middle.\n"
 "3. Shaft first: push a 6L axle through wheel 1's hub so 2.4 cm sticks out on one side, and push the axle joiner (No.18) onto the other end.\n"
 "    Push wheel 1 (with its axle, stub pointing left) into the tube from the right until its centre is 11 cm from the left end. Push wheel 2 in flush with the right end.\n"
 "    Wrap 2-3 turns of tape around the outside of the tube over each wheel: the tape squeezes the card onto the rubber, and that grip is what holds the wheels.\n"
 "4. Slide the tube onto the motor from the right (it passes over the motor and its 4 beams) and turn it until the 6L stub drops into the EV3 Medium's output.\n"
 "5. Push a 9L axle in from outside through both right-frame bearing holes and wheel 2's hub into the joiner. It ends halfway through the outer layer: tape over that hole.")
import textwrap
steps = "\n".join(textwrap.fill(l, 128, subsequent_indent="    ") for l in steps.split("\n"))
tx.text(0, 0.98, steps, va="top", fontsize=9.2, family="DejaVu Sans")
fig.savefig("/home/sohaib/projects/mcgill-classes/ecse-211/final-project/docs/renders/10_roller_build.png", dpi=110)
