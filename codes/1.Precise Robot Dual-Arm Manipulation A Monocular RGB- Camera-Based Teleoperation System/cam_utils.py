import torch
import numpy as np


def pixel2cam(pixel_coord, f, c):
    x = (pixel_coord[:, 0] - c[0]) / f[0] * pixel_coord[:, 2]
    y = (pixel_coord[:, 1] - c[1]) / f[1] * pixel_coord[:, 2]
    z = pixel_coord[:, 2]
    cam_coord = torch.stack([x, y, z], 1)
    # cam_coord = np.concatenate((x[:, None], y[:, None], z[:, None]), 1)
    return cam_coord


def cam2pixel(cam_coord, f, c):
    x = cam_coord[:, 0] / cam_coord[:, 2] * f[0] + c[0]
    y = cam_coord[:, 1] / cam_coord[:, 2] * f[1] + c[1]
    z = cam_coord[:, 2]
    pixel_coord = torch.stack([x, y, z], 1)
    return pixel_coord

# 直接不使用投影变换
# def pixel2cam(pixel_coord, f, c):
#     x = (pixel_coord[:, 0] - c[0]) / f[0] * 1000
#     y = (pixel_coord[:, 1] - c[1]) / f[1] * 1000
#     z = pixel_coord[:, 2]
#     cam_coord = torch.stack([x, y, z], 1)
#     # cam_coord = np.concatenate((x[:, None], y[:, None], z[:, None]), 1)
#     return cam_coord


# def cam2pixel(cam_coord, f, c):
#     x = cam_coord[:, 0] / 1000 * f[0] + c[0]
#     y = cam_coord[:, 1] / 1000 * f[1] + c[1]
#     z = cam_coord[:, 2]
#     pixel_coord = torch.stack([x, y, z], 1)
#     return pixel_coord


def bbox_transform(bbox, ratio, pre_w, pre_h, img_w, img_h):
    x, y, w, h = bbox
    l_w = max(w, pre_w)
    l_h = max(h, pre_h)
    l = max(l_w, l_h / ratio)
    l = max(w, h / ratio)
    y1 = y - l * ratio / 2
    y2 = y + l * ratio / 2
    x1 = x - l / 2
    x2 = x + l / 2
    # pdb.set_trace()
    # x1 = torch.clamp(x1, min=0)
    # y1 = torch.clamp(y1, min=0)
    # x2 = torch.clamp(x2, max=img_w)
    # y2 = torch.clamp(y2, max=img_h)
    return torch.stack([x1.floor(), y1.floor(), x2.ceil(), y2.ceil()]).to(bbox.device), l, l * ratio

# 利用Vitruvian man的比例关系，由人体2d关键点坐标计算bbox(x,y,w,h)


def get_bbox_from_kps(kps, ratio):
    '''利用维特鲁威人比例，以root为中心，计算bbox'''
    head = kps[10]
    root = kps[0]
    l = (head-root).norm()
    r = l/(1-0.618)
    w = r
    h = r*ratio
    return torch.tensor([root[0], root[1], w, h])
