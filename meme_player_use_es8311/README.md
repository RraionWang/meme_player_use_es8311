# 说明

## 硬件说明
esp32s3为了省事，用了[格子派家的模组](https://item.taobao.com/item.htm?abbucket=18&id=915728207906&mi_id=00005270PBI2RisjDPBx_fgMEdCpRxkgpmDzyucg6XMpKJ4&ns=1&priceTId=213e093017688383575898720e11d1&skuId=5859121257149&spm=a21n57.1.hoverItem.2&utparam=%7B%22aplus_abtest%22%3A%2211a40b0f737dd52bcca328dc1e3291a4%22%7D&xxc=taobaoSearch)，不过现在这个模组已经停产了，使用他们家的AI2模组可能要修改引脚定义，如果使用esp32s3也可以，需要外接一个NS4168的模组，如果后面有时间我会在做一个（不过大概率没时间）

## 视频压缩指令
ffmpeg -i input2.mp4 -vf "scale=240:240:force_original_aspect_ratio=decrease,pad=240:240:(ow-iw)/2:(oh-ih)/2:black" -c:v mjpeg -q:v 20 -r 12 -c:a pcm_s16le -ar 16000 -ac 1 -fflags +bitexact -flags +bitexact -f avi happy.avi

视频需要使用此指令进行转换

## 存储
我的代码默认将视频存储到flash，也可以改硬件放到sd卡。不过要改代码，欢迎一起交流