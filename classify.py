import time

# ─── CONFIGURABLE TARGET POINTS ─────────────────────────────────────────
TOP_Y     = -80    # Y position of top cubes (adjust on camera)
BOTTOM_Y  =  60    # Y position of bottom cubes (adjust on camera)
X_SPLIT   =   0    # X = 0 splits left/right
WINDOW_MS = 500    # How long to collect blobs for
# ────────────────────────────────────────────────────────────────────────

TOP_POINT = [X_SPLIT, TOP_Y]
BOTTOM_POINT = [X_SPLIT, BOTTOM_Y]

def is_similar(block1, block2, threshold=0.91):
    x1, y1, w1, h1, cid1 = block1
    x2, y2, w2, h2, cid2 = block2

    if cid1 != cid2:
        return False

    return (
        (min(x1, x2) / max(x1, x2) >= threshold if x1 != 0 and x2 != 0 else x1 == x2) and
        (min(y1, y2) / max(y1, y2) >= threshold if y1 != 0 and y2 != 0 else y1 == y2) and
        (min(w1, w2) / max(w1, w2) >= threshold if w1 != 0 and w2 != 0 else w1 == w2) and
        (min(h1, h2) / max(h1, h2) >= threshold if h1 != 0 and h2 != 0 else h1 == h2)
    )

def clean_blocks(blocks):
    seen = set()
    cleaned = []

    for block in blocks:
        if block == (0, 0, 0, 0, 0):
            continue

        x, y, w, h, cid = block

        # Quantize values to simulate fuzzy matching (9% granularity)
        qx = int(x // 9)
        qy = int(y // 9)
        qw = int(w // 9)
        qh = int(h // 9)

        key = (cid, qx, qy, qw, qh)

        if key not in seen:
            seen.add(key)
            cleaned.append(block)

    return cleaned

def _euclidean_dist_sq(x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2

def scan_color_corners_window(blocks, read_fn, window_ms=WINDOW_MS):
    TL = [0] * 5  # votes[1-4] valid
    TR = [0] * 5
    BL = [0] * 5
    BR = [0] * 5

    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < window_ms:
        blocks = clean_blocks(read_fn())

        tl_dist, tr_dist = float('inf'), float('inf')
        bl_dist, br_dist = float('inf'), float('inf')

        tl_cid, tr_cid = 0, 0
        bl_cid, br_cid = 0, 0

        for b in blocks:
            x, y, _, _, cid = b

            if cid == 0:
                continue

            if y < (TOP_Y + BOTTOM_Y) // 2:
                dist = _euclidean_dist_sq(x, y, TOP_POINT[0], TOP_POINT[1])

                if x <= X_SPLIT and dist < tl_dist: #stanga si distanta minima vazuta pana acum
                    tl_dist = dist
                    tl_cid = cid
                elif x > X_SPLIT and dist < tr_dist: #dreapta si distanta minima vazuta pana acum
                    tr_dist = dist
                    tr_cid = cid
            else:
                dist = _euclidean_dist_sq(x, y, BOTTOM_POINT[0], BOTTOM_POINT[1])

                if x <= X_SPLIT and dist < bl_dist: #stanga si distanta minima vazuta pana acum
                    bl_dist = dist
                    bl_cid = cid
                elif x > X_SPLIT and dist < br_dist: #dreapta si distanta minima vazuta pana acum
                    br_dist = dist
                    br_cid = cid

        if tl_cid: TL[tl_cid] += 1
        if tr_cid: TR[tr_cid] += 1
        if bl_cid: BL[bl_cid] += 1
        if br_cid: BR[br_cid] += 1

    # Compute max voted color IDs
    tl_max, tr_max, bl_max, br_max = 0, 0, 0, 0
    tl_count, tr_count, bl_count, br_count = -1, -1, -1, -1

    for i in range(1, 5):
        if TL[i] > tl_count:
            tl_count = TL[i]
            tl_max = i

        if TR[i] > tr_count:
            tr_count = TR[i]
            tr_max = i

        if BL[i] > bl_count:
            bl_count = BL[i]
            bl_max = i
            
        if BR[i] > br_count:
            br_count = BR[i]
            br_max = i

    print(f"Color IDs per corner (after {window_ms} ms):")
    print("TL:", tl_max, "TR:", tr_max)
    print("BL:", bl_max, "BR:", br_max)

    return {
        'TL': tl_max,
        'TR': tr_max,
        'BL': bl_max,
        'BR': br_max
    }

