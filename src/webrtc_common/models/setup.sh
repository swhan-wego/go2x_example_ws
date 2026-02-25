# 기존 깨진 파일 삭제
rm -f detect_2021nov.prototxt detect_2021nov.caffemodel sr_2021nov.prototxt sr_2021nov.caffemodel
# 다시 다운로드
wget https://huggingface.co/opencv/qrcode_wechatqrcode/resolve/main/detect_2021nov.caffemodel
wget https://huggingface.co/opencv/qrcode_wechatqrcode/resolve/main/detect_2021nov.prototxt
wget https://huggingface.co/opencv/qrcode_wechatqrcode/resolve/main/sr_2021nov.caffemodel
wget https://huggingface.co/opencv/qrcode_wechatqrcode/resolve/main/sr_2021nov.prototxt

sudo apt-get update
sudo apt-get install libzbar0 libgl1-mesa-glx
