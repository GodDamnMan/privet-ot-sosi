import sys
import numpy as np


def read_image():
    data = []
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        vals = line.split()
        row = []
        for s in vals:
            x = int(s, 16)
            row.append([(x >> 16) & 255, (x >> 8) & 255, x & 255])
        data.append(row)
    return np.array(data, dtype=np.uint8)


def to_gray(img):
    return img.mean(axis=2)


def build_dark_mask(img):
    mx = img.max(axis=2)
    mn = img.min(axis=2)
    return (mx < 60) & ((mx - mn) < 35)


def connected_components(mask):
    h, w = mask.shape
    vis = np.zeros((h, w), dtype=bool)
    comps = []

    for y in range(h):
        for x in range(w):
            if mask[y, x] and not vis[y, x]:
                stack = [(y, x)]
                vis[y, x] = True
                pts = []

                while stack:
                    cy, cx = stack.pop()
                    pts.append((cy, cx))

                    for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not vis[ny, nx]:
                            vis[ny, nx] = True
                            stack.append((ny, nx))

                comps.append(np.array(pts, dtype=np.int32))

    return comps


def component_bbox(pts):
    ys = pts[:, 0]
    xs = pts[:, 1]
    return xs.min(), ys.min(), xs.max(), ys.max()


def component_score(pts):
    x0, y0, x1, y1 = component_bbox(pts)
    w = x1 - x0 + 1
    h = y1 - y0 + 1
    area = len(pts)
    fill = area / float(w * h)
    aspect = min(w, h) / float(max(w, h))
    score = area * fill * aspect
    return score, area, fill, aspect, (x0, y0, x1, y1)


def find_candidates(img):
    mask = build_dark_mask(img)
    comps = connected_components(mask)

    cands = []
    for pts in comps:
        score, area, fill, aspect, bbox = component_score(pts)

        if area < 180:
            continue
        if aspect < 0.45:
            continue
        if fill < 0.30:
            continue

        cands.append((score, pts, bbox, area, fill, aspect))

    cands.sort(key=lambda x: x[0], reverse=True)
    return cands


def compute_homography(src, dst):
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y, -u])
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y, -v])

    A = np.array(A, dtype=np.float64)
    _, _, vh = np.linalg.svd(A)
    H = vh[-1].reshape(3, 3)
    return H / H[2, 2]


def warp_perspective_nn(img, corners, size=120):
    dst = np.array([
        [0, 0],
        [size - 1, 0],
        [size - 1, size - 1],
        [0, size - 1]
    ], dtype=np.float64)

    H = compute_homography(dst, corners)
    out = np.zeros((size, size, 3), dtype=np.uint8)

    for v in range(size):
        for u in range(size):
            x, y, z = H @ np.array([u, v, 1.0], dtype=np.float64)
            if z == 0:
                continue
            x /= z
            y /= z

            xi = int(round(x))
            yi = int(round(y))

            if 0 <= xi < img.shape[1] and 0 <= yi < img.shape[0]:
                out[v, u] = img[yi, xi]

    return out


def sample_cells(warp, n=5, inner_a=0.30, inner_b=0.70):
    g = to_gray(warp)
    h, w = g.shape
    vals = np.zeros((n, n), dtype=np.float64)

    for r in range(n):
        for c in range(n):
            y0 = int((r + inner_a) * h / n)
            y1 = int((r + inner_b) * h / n)
            x0 = int((c + inner_a) * w / n)
            x1 = int((c + inner_b) * w / n)
            block = g[y0:y1, x0:x1]
            vals[r, c] = block.mean()

    thr = 0.5 * (vals.min() + vals.max())
    bits = (vals > thr).astype(np.int32)
    return vals, bits, thr


def try_decode_bits(bits5):
    border_ones = (
        bits5[0, :].sum() +
        bits5[-1, :].sum() +
        bits5[1:-1, 0].sum() +
        bits5[1:-1, -1].sum()
    )
    if border_ones > 2:
        return None

    inner = bits5[1:4, 1:4]

    for k in range(4):
        rot = np.rot90(inner, k)

        if not (
            rot[0, 0] == 0 and
            rot[0, 2] == 0 and
            rot[2, 0] == 0 and
            rot[2, 2] == 1
        ):
            continue

        top_bit = int(rot[0, 1])
        left_bit = int(rot[1, 0])
        right_bit = int(rot[1, 2])
        bottom_bit = int(rot[2, 1])
        center = int(rot[1, 1])

        ones = top_bit + left_bit + right_bit + bottom_bit
        expected_center = 1 if (ones % 2 == 1) else 0

        if center != expected_center:
            continue

        value = (
            top_bit * 1 +
            left_bit * 2 +
            right_bit * 4 +
            bottom_bit * 8
        )
        return value

    return None


def decode_candidate(img, pts):
    x0, y0, x1, y1 = component_bbox(pts)

    for pad in (-1, 0, 1, 2, 3):
        corners = np.array([
            [x0 - pad, y0 - pad],
            [x1 + pad, y0 - pad],
            [x1 + pad, y1 + pad],
            [x0 - pad, y1 + pad]
        ], dtype=np.float64)

        for size in (100, 120, 150, 180):
            warp = warp_perspective_nn(img, corners, size=size)

            for a, b in ((0.28, 0.72), (0.30, 0.70), (0.33, 0.67), (0.35, 0.65)):
                _, bits5, _ = sample_cells(warp, n=5, inner_a=a, inner_b=b)
                value = try_decode_bits(bits5)
                if value is not None:
                    return value

    return None


def decide_turn(img):
    R = img[:, :, 0].astype(np.int32)
    G = img[:, :, 1].astype(np.int32)
    B = img[:, :, 2].astype(np.int32)

    blue_score = (B - R) + (B - G)
    yellow_score = (R + G - 2 * B)

    blue_strength = np.percentile(blue_score, 99.5)
    yellow_strength = np.percentile(yellow_score, 99.5)

    if blue_strength >= yellow_strength:
        return "L"
    return "R"


def solve(img):
    candidates = find_candidates(img)

    for _, pts, bbox, area, fill, aspect in candidates[:10]:
        value = decode_candidate(img, pts)
        if value is not None:
            return str(value)

    return decide_turn(img)


def main():
    img = read_image()
    print(solve(img))


if __name__ == "__main__":
    main()