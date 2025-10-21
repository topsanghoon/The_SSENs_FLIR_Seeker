import glob, cv2, numpy as np, os

# 입력/출력 설정
pattern = "../frame_*.tiff"   # 저장된 경로에 맞춰 수정
out_path = "out_auto.mp4"
fps = 8

files = sorted(glob.glob(pattern))
if not files:
    raise SystemExit("No TIFFs found")

# 첫 프레임으로 크기 파악
img0 = cv2.imread(files[0], cv2.IMREAD_UNCHANGED)  # 16-bit 그대로
h, w = img0.shape

# 비디오 라이터
fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 라즈베리파이에서 호환 좋음
writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h), True)

def to8u_auto(m16):
    # 16-bit 단일채널 → 프레임별 min/max로 8-bit
    mn, mx = int(m16.min()), int(m16.max())
    if mx <= mn:  # 안전 장치
        return (m16 >> 8).astype(np.uint8)
    m = ((m16.astype(np.float32) - mn) * 255.0 / (mx - mn)).clip(0,255)
    return m.astype(np.uint8)

for i, f in enumerate(files):
    m16 = cv2.imread(f, cv2.IMREAD_UNCHANGED)  # uint16
    if m16 is None:
        print("skip:", f); continue
    g8 = to8u_auto(m16)
    # 보기 좋게 컬러맵(옵션): 주석 해제하면 컬러로 저장
    # g8_color = cv2.applyColorMap(g8, cv2.COLORMAP_INFERNO)
    g8_bgr = cv2.cvtColor(g8, cv2.COLOR_GRAY2BGR)
    writer.write(g8_bgr)
    if (i+1) % 100 == 0:
        print(f"{i+1}/{len(files)} frames")

writer.release()
print("saved:", out_path)

