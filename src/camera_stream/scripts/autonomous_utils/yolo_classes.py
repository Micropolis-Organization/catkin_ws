"""yolov3_classes.py

NOTE: Number of YOLOv3 COCO output classes differs from SSD COCO models.
"""
import os

coco_path = os.path.join(os.path.dirname(__file__), "coco80.names")

with open(coco_path, 'r') as f:
    names = f.readlines()
print(names)
COCO_CLASSES_LIST = [x.split("\n")[0] for x in names]

def get_cls_dict(model):
    """Get the class ID to name translation dictionary."""
    if model == 'coco':
        cls_list = COCO_CLASSES_LIST
    else:
        raise ValueError('Bad model name')
    return {i: n for i, n in enumerate(cls_list)}