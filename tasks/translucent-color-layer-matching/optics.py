"""Tint model for the filter set.

Stacked filters multiply componentwise, which is how the lightbox window
colour is computed at run time and how scoring decides whether a stack
matches the reference. The renderer does not composite stacked glass, so the
window colour is computed from this model rather than ray traced; see
README.md.
"""

import argparse
from itertools import combinations
import math

FILTER_TINTS = {"amber": (1.00, 0.60, 0.25), "cyan": (0.25, 0.90, 1.00),
                "violet": (0.80, 0.25, 0.85), "lime": (0.55, 1.00, 0.32)}
REQUIRED = frozenset(("amber", "cyan"))
MATCH_TOLERANCE = 0.05  # maximum per-channel difference counted as the same tint


def nominal_tint(names):
    return tuple(math.prod(FILTER_TINTS[name][channel] for name in names) for channel in range(3))


def difference(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def subsets():
    for n in range(len(FILTER_TINTS) + 1):
        yield from combinations(FILTER_TINTS, n)


REFERENCE_TINT = nominal_tint(REQUIRED)
# Every set of filters whose combined tint is the reference tint. Scoring uses
# this rather than the names in REQUIRED, so any set that genuinely reproduces
# the reference colour earns full credit.
MATCHING_SETS = frozenset(frozenset(names) for names in subsets()
                          if difference(nominal_tint(names), REFERENCE_TINT) <= MATCH_TOLERANCE)


def _roi(value):
    try:
        x, y, w, h = (int(s) for s in value.split(","))
        if x < 0 or y < 0 or w < 3 or h < 3:
            raise ValueError
    except ValueError as error:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height; size at least 3x3") from error
    return x, y, w, h


def _linear(c):
    c /= 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def image_comparison(path, stack_roi, reference_roi):
    """Compare equal-sized interior ROIs of one image, assuming sRGB output."""
    from PIL import Image

    if stack_roi[2:] != reference_roi[2:]:
        raise ValueError("stack and reference ROIs must have equal dimensions")
    with Image.open(path) as image:
        image = image.convert("RGB")
        means, clipped = [], []
        for x, y, w, h in (stack_roi, reference_roi):
            if x < 0 or y < 0 or w < 3 or h < 3 or x + w > image.width or y + h > image.height:
                raise ValueError("invalid ROI or ROI extends beyond the image")
            pixels = [image.getpixel((px, py)) for py in range(y, y + h) for px in range(x, x + w)]
            means.append(tuple(sum(_linear(p[i]) for p in pixels) / len(pixels) for i in range(3)))
            clipped.append(sum(any(v in (0, 255) for v in p) for p in pixels) / len(pixels))
    return means[0], means[1], difference(*means), tuple(clipped)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", help="one unedited simulator screenshot")
    parser.add_argument("--stack-roi", type=_roi, help="stack window: x,y,width,height")
    parser.add_argument("--reference-roi", type=_roi, help="reference window: x,y,width,height")
    args = parser.parse_args()
    if args.image:
        if not args.stack_roi or not args.reference_roi:
            parser.error("--image requires both ROIs")
        stack, ref, delta, clipped = image_comparison(args.image, args.stack_roi, args.reference_roi)
        print(f"stack linear RGB: {stack}; reference linear RGB: {ref}")
        print(f"maximum channel difference: {delta:.4f}; preliminary threshold: {MATCH_TOLERANCE:.2f}")
        print(f"fraction with clipped channel: stack {clipped[0]:.1%}, reference {clipped[1]:.1%}")
        print("Preliminary pixel criterion met" if delta <= MATCH_TOLERANCE and max(clipped) <= 0.05 else
              "Pixel criterion not met or inconclusive (clipping)")
    elif args.stack_roi or args.reference_roi:
        parser.error("ROIs require --image")
    else:
        print(f"Nominal reference RGB: {REFERENCE_TINT}; threshold: {MATCH_TOLERANCE:.2f}")
        for names in subsets():
            tint = nominal_tint(names)
            print(f"{','.join(names) or '(empty)':25} {tuple(round(v, 4) for v in tint)}  "
                  f"max difference {difference(tint, REFERENCE_TINT):.4f}")


if __name__ == "__main__":
    main()
