"""Encode the Fusion viewport frames with a developed view of the pen trace."""
import argparse
import shutil
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent
FPS = 25


def encode(frames, ffmpeg):
    traj = loadmat(ROOT.parent / "step4_trajectory.mat", simplify_cells=True)["traj"]
    q1, q2, q3, q4 = traj["Q"]
    z = 225 + 300*np.sin(q3) + 50*np.sin(q3+q4)
    u = -390*q1
    # Project the pen tip onto the developed wall during the writing phase.
    points = list(zip((160+1.25*u).tolist(), (465-1.25*(z-89.7)).tolist()))
    times = traj["t"]
    first, last = np.flatnonzero(traj["phase"] == 2)[[0, -1]]
    regular = ImageFont.truetype("segoeui.ttf", 18)
    bold = ImageFont.truetype("segoeuib.ttf", 24)
    target = [(59,129), (261,129), (59,129), (59,465), (261,465)]
    temp = ROOT / "E_Writing.building.mp4"

    def canvases():
        for frame in range(round(FPS*times[-1])):
            t = frame*times[-1]/round(FPS*times[-1])
            panel = Image.new("RGB", (320,640), (26,33,42))
            draw = ImageDraw.Draw(panel)
            draw.text((24,28),"E on the inner wall",font=bold,fill="#f0f5fc")
            draw.text((24,64),"Unwrapped pen trace",font=regular,fill="#aab8c8")
            draw.line(target,fill="#566473",width=3)
            draw.line([(59,297),(221,297)],fill="#566473",width=3)
            sample = np.searchsorted(times,t,side="right")-1
            if sample >= first:
                end = min(last,sample)
                if end > first:
                    draw.line(points[first:end+1],fill="#59d4e0",width=5)
                if sample <= last:
                    x,y = points[end]
                    draw.ellipse((x-5,y-5,x+5,y+5),fill="white")
            phase = ["Approach","Writing","Return"][traj["phase"][sample]-1]
            draw.text((24,525),phase,font=bold,fill="#f0f5fc")
            draw.text((24,570),f"{t:4.2f} s  /  {times[-1]:.2f} s",font=regular,fill="#aab8c8")
            canvas = Image.new("RGB",(1280,640))
            with Image.open(frames / f"frame_{frame:04}.png") as shot:
                canvas.paste(shot.convert("RGB"),(0,0))
            canvas.paste(panel,(960,0))
            yield canvas

    if ffmpeg and shutil.which(ffmpeg):
        args = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
                "-pix_fmt", "rgb24", "-s", "1280x640", "-r", str(FPS), "-i", "-",
                "-an", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", str(temp)]
        with subprocess.Popen(args, stdin=subprocess.PIPE) as encoder:
            for canvas in canvases():
                encoder.stdin.write(canvas.tobytes())
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("Video encoding failed")
    else:
        # no ffmpeg on this machine (2026-11-26): OpenCV writes the file, H.264 if it can, else MPEG-4
        import cv2
        count = 0
        for tag in ("avc1", "mp4v"):
            writer = cv2.VideoWriter(str(temp), cv2.VideoWriter_fourcc(*tag), FPS, (1280, 640))
            if not writer.isOpened():
                continue
            count = 0
            for canvas in canvases():
                writer.write(cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR))
                count += 1
            writer.release()
            check = cv2.VideoCapture(str(temp))
            written = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
            check.release()
            if written == count:
                print("encoded with OpenCV,", tag, count, "frames")
                break
        else:
            raise RuntimeError("Video encoding failed")
    temp.replace(ROOT / "E_Writing.mp4")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    args = parser.parse_args()
    encode(args.frames, args.ffmpeg)
