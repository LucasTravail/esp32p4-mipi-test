"""Generate main/clip.bin, the MJPEG clip played by the "Touch me" button: [u32 le size][jpeg] per frame.

    python tools/make_clip.py                 # synthetic test clip
    python tools/make_clip.py my_video.mp4    # your own video (needs ffmpeg in PATH, or: pip install imageio-ffmpeg)
"""
import glob, io, math, struct, subprocess, sys, tempfile
from PIL import Image, ImageDraw

W = H = 466      # must match header.w/h in main.c (video_button_create)
FPS = 8          # must match the lv_timer period (100 ms) in main.c
MAX_S = 8        # clip.bin is embedded in the app partition (6 MB): keep it small


def frames_from_video(path):
    d = tempfile.mkdtemp()
    try:
        import imageio_ffmpeg
        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()   # pip install imageio-ffmpeg
    except ImportError:
        ffmpeg = "ffmpeg"
    subprocess.run([ffmpeg, "-y", "-i", path, "-t", str(MAX_S), "-vf",
                    f"fps={FPS},crop='min(iw,ih)':'min(iw,ih)',scale={W}:{H}", d + "/f%04d.png"], check=True)
    return [Image.open(f).convert("RGB") for f in sorted(glob.glob(d + "/f*.png"))]


def synthetic(n=48):
    for i in range(n):
        a = 2 * math.pi * i / n
        im = Image.new("RGB", (W, H), (10, 10, 30))
        d = ImageDraw.Draw(im)
        for k in range(6):
            t = a + k * math.pi / 3
            x, y = W / 2 + 70 * math.cos(t), H / 2 + 70 * math.sin(t)
            c = tuple(int(127 + 127 * math.sin(a * 2 + k + p)) for p in (0, 2, 4))
            d.ellipse([x - 22, y - 22, x + 22, y + 22], fill=c)
        d.ellipse([W / 2 - 18, H / 2 - 18, W / 2 + 18, H / 2 + 18], fill=(255, 255, 255))
        yield im


frames = frames_from_video(sys.argv[1]) if len(sys.argv) > 1 else list(synthetic())
out = bytearray()
for im in frames:
    b = io.BytesIO()
    im.save(b, "JPEG", quality=65)
    out += struct.pack("<I", len(b.getvalue())) + b.getvalue()
open("main/clip.bin", "wb").write(out)
print(len(frames), "frames", len(out), "bytes")
