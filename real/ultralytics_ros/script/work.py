import os
import torch
torch.cuda.init()

from ultralytics import YOLO

model=YOLO('yolo12n.pt')


success=model.export(format ='engine', half=True, device=0)


if success:
    print("running on TensorRT")
else:
    print("NOT!!!!")