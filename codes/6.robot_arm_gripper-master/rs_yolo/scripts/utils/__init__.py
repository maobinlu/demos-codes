'''
Author: xyz 2410391147@qq.com
Date: 2023-02-22 15:23:35
LastEditors: xyz 2410391147@qq.com
LastEditTime: 2023-02-22 16:34:15
FilePath: /arm807_hand_ws/src/rs_yolo/scripts/utils/__init__.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from pathlib import Path

# import sys
# sys.path.append(r"/home/yxz/anaconda3/envs/yolov5/lib/python3.7/site-packages/torch/__init__.py"); 

#import torch
from PIL import ImageFont

FILE = Path(__file__).absolute()
ROOT = FILE.parents[1]  # yolov5/ dir

# Check YOLOv5 Annotator font
font = 'Arial.ttf'
# try:
#     ImageFont.truetype(font)
# except Exception as e:  # download if missing
#     url = "https://ultralytics.com/assets/" + font
#     print(f'Downloading {url} to {ROOT / font}...')
#     torch.hub.download_url_to_file(url, str(ROOT / font))
