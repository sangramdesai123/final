import numpy as np
from PIL import Image
from copy import deepcopy

img = np.array(Image.open("checker_blue.png"))
print(img.shape)

img_black = img[:128,:128,:]
img_white = img[256:384,:128,:]

print(img_black.shape,img_white.shape)

img=[]

for i in range(2):
	img.append(deep)
	
