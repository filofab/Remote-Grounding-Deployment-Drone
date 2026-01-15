import cv2
from config import FLIP_HORIZONTAL, FLIP_VERTICAL

def apply_flip(img):
    if FLIP_HORIZONTAL and FLIP_VERTICAL:
        return cv2.flip(img, -1)
    elif FLIP_HORIZONTAL:
        return cv2.flip(img, 1)
    elif FLIP_VERTICAL:
        return cv2.flip(img, 0)
    return img
