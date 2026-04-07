import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


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
    return cands, mask


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


def draw_grid(ax, size, n=5):
    step = size / n
    for i in range(1, n):
        ax.axhline(i * step, linewidth=1)
        ax.axvline(i * step, linewidth=1)


def show_cells(bits5, title="bits5"):
    plt.figure(figsize=(4, 4))
    plt.imshow(bits5, cmap="gray", vmin=0, vmax=1)
    plt.title(title)
    for r in range(bits5.shape[0]):
        for c in range(bits5.shape[1]):
            plt.text(c, r, str(bits5[r, c]), ha="center", va="center")
    plt.xticks(range(bits5.shape[1]))
    plt.yticks(range(bits5.shape[0]))
    plt.show()


def solve_with_debug(img):
    candidates, mask = find_candidates(img)

    print(f"candidates: {len(candidates)}")

    # 1. Исходное изображение
    plt.figure(figsize=(10, 6))
    plt.imshow(img)
    plt.title("Original image")
    plt.axis("off")
    plt.show()

    # 2. Маска чёрного
    plt.figure(figsize=(10, 6))
    plt.imshow(mask, cmap="gray")
    plt.title("Dark mask")
    plt.axis("off")
    plt.show()

    # 3. Кандидаты на исходной картинке
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(img)
    ax.set_title("Candidates")
    ax.axis("off")

    for i, (_, pts, bbox, area, fill, aspect) in enumerate(candidates[:10]):
        x0, y0, x1, y1 = bbox
        rect = Rectangle((x0, y0), x1 - x0 + 1, y1 - y0 + 1, fill=False, linewidth=2)
        ax.add_patch(rect)
        ax.text(x0, y0 - 2, f"{i}: area={area}, fill={fill:.2f}, asp={aspect:.2f}")

    plt.show()

    # 4. Перебор кандидатов
    for idx, (_, pts, bbox, area, fill, aspect) in enumerate(candidates[:10]):
        print(f"\n--- candidate {idx} ---")
        print(f"bbox={bbox}, area={area}, fill={fill:.3f}, aspect={aspect:.3f}")

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
                    vals, bits5, thr = sample_cells(warp, n=5, inner_a=a, inner_b=b)
                    value = try_decode_bits(bits5)

                    if value is not None:
                        print(f"decoded: value={value}, pad={pad}, size={size}, a={a}, b={b}, thr={thr:.2f}")

                        # warp
                        fig, ax = plt.subplots(figsize=(5, 5))
                        ax.imshow(warp)
                        draw_grid(ax, size, n=5)
                        ax.set_title(f"Warp for candidate {idx}")
                        ax.axis("off")
                        plt.show()

                        # средние яркости по клеткам
                        plt.figure(figsize=(4, 4))
                        plt.imshow(vals, cmap="gray")
                        plt.title("Cell mean brightness")
                        for r in range(vals.shape[0]):
                            for c in range(vals.shape[1]):
                                plt.text(c, r, f"{vals[r, c]:.0f}", ha="center", va="center")
                        plt.xticks(range(5))
                        plt.yticks(range(5))
                        plt.show()

                        # бинарная сетка
                        show_cells(bits5, title=f"Decoded bits5 -> value {value}")

                        return str(value)

        print("candidate failed")

    ans = decide_turn(img)
    print(f"fallback turn: {ans}")
    return ans


def main():
    img = read_image()
    ans = solve_with_debug(img)
    print("\nFINAL ANSWER:", ans)


if __name__ == "__main__":
    main()