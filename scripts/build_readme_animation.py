"""Draw a conceptual path-following GIF; this does not simulate the plant.

Requires Pillow. The arm pose is prescribed by planar inverse kinematics.
No hydraulic dynamics, feedback response, or tracking performance is modeled.
"""
from pathlib import Path
import math

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT, SCALE = 960, 520, 2
INK = '#253746'
ORANGE = '#CB7624'
BLUE = '#537FA0'
BASE = (200, 405)
L1, L2 = 310, 260


def point(a, b, t):
    return tuple(x + t * (y - x) for x, y in zip(a, b))


def make_frame(phase):
    im = Image.new('RGB', (WIDTH * SCALE, HEIGHT * SCALE), '#FAFBFC')
    draw = ImageDraw.Draw(im)

    def line(points, fill, width):
        draw.line([(int(x*SCALE), int(y*SCALE)) for x, y in points],
                  fill=fill, width=width*SCALE, joint='curve')

    def circle(p, r, fill, outline=None, width=1):
        x, y = p
        draw.ellipse(((x-r)*SCALE, (y-r)*SCALE,
                      (x+r)*SCALE, (y+r)*SCALE),
                     fill=fill, outline=outline, width=width*SCALE)

    def box(bounds, fill, radius=4):
        draw.rounded_rectangle(tuple(v*SCALE for v in bounds),
                               radius=radius*SCALE, fill=fill)

    def text(p, label, size=18, fill=INK):
        draw.text(tuple(v*SCALE for v in p), label,
                  font=ImageFont.load_default(size=size*SCALE), fill=fill)

    def path(t):
        return 665 + 65*math.cos(t), 315 + 30*math.sin(t)

    text((42, 30), 'Hydraulic manipulator', 26)
    text((42, 66), 'Smooth path following', 17, '#647583')
    text((735, 42), 'ILLUSTRATIVE MOTION', 13, '#647583')

    # A fixed reference and analytically prescribed end-effector pose.
    for j in range(72):
        if j % 2 == 0:
            line([path(2*math.pi*j/72), path(2*math.pi*(j+1)/72)], BLUE, 2)
    tip = path(phase)
    dx, dy = tip[0]-BASE[0], BASE[1]-tip[1]
    d = math.hypot(dx, dy)
    shoulder = math.atan2(dy, dx) + math.acos((L1*L1+d*d-L2*L2)/(2*L1*d))
    elbow = BASE[0]+L1*math.cos(shoulder), BASE[1]-L1*math.sin(shoulder)

    # Ground, pedestal, links, and a simplified mowing head.
    line([(85, 438), (842, 438)], '#DCE3E8', 2)
    box((138, 424, 262, 438), '#617582')
    box((166, 397, 234, 425), '#8C9DA8')
    line([BASE, elbow], INK, 23)
    line([elbow, tip], INK, 19)
    line([BASE, elbow], ORANGE, 17)
    line([elbow, tip], ORANGE, 13)

    def cylinder(a, b):
        line([a, b], '#465B68', 10)
        line([a, point(a, b, 0.63)], '#9EADB7', 16)
        line([point(a, b, 0.63), b], '#D9E1E6', 6)
        circle(a, 5, INK)
        circle(b, 5, INK)

    cylinder((181, 392), point(BASE, elbow, 0.70))
    cylinder(point(BASE, elbow, 0.77), point(elbow, tip, 0.40))
    for p, r in [(BASE, 13), (elbow, 11), (tip, 7)]:
        circle(p, r, INK)
        circle(p, r-4, '#E6EBEF')
    line([tip, (tip[0], tip[1]+18)], INK, 6)
    box((tip[0]-29, tip[1]+16, tip[0]+29, tip[1]+31), ORANGE)
    line([(tip[0]-32, tip[1]+32), (tip[0]+32, tip[1]+32)], INK, 5)
    circle(tip, 3, BLUE)

    line([(42, 484), (74, 484)], BLUE, 2)
    text((84, 474), 'Reference path', 15, '#647583')
    text((610, 474), 'Concept illustration · no simulation data', 14, '#647583')
    return im.resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS)


def main():
    frames = [make_frame(2*math.pi*i/96) for i in range(96)]
    palette = frames[0].quantize(colors=128)
    indexed = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in frames]
    output = ROOT/'docs/assets/path-following-illustration.gif'
    output.parent.mkdir(parents=True, exist_ok=True)
    indexed[0].save(output, save_all=True, append_images=indexed[1:],
                    duration=60, loop=0, optimize=True, disposal=2)
    print(output)


if __name__ == '__main__':
    main()
