# 说明

## 压缩指令
ffmpeg -i input2.mp4 -vf "scale=240:240:force_original_aspect_ratio=decrease,pad=240:240:(ow-iw)/2:(oh-ih)/2:black" -c:v mjpeg -q:v 20 -r 12 -c:a pcm_s16le -ar 16000 -ac 1 -fflags +bitexact -flags +bitexact -f avi happy.avi
