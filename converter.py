import cv2
import numpy as np
a = set()
def rgb2brghex(rgb_color):
#    return '0x' + format(((rgb_color[0]/64*4) << 8) + (rgb_color[2]/64*4) + ((rgb_color[1]/64*4) << 16), '02x')    
    max_color = max(rgb_color)
    for i in range(3):
        if max_color != rgb_color[i]:
            rgb_color[i] = 0
    return '0x' + format(((rgb_color[0]/16) << 8) + (rgb_color[2]/16) + ((rgb_color[1]/16) << 16), '02x')    

#img = cv2.imread('globe.jpg')
img = cv2.imread('hello.png')
resized_img = cv2.resize(img,(200, 100), interpolation=cv2.INTER_AREA)
print(resized_img.shape)
led_data = []
for i in range(200):
    buff = [0]*200
    for j in range(100):
        buff[j] = rgb2brghex(resized_img[j][i])
        buff[200-j-1] = rgb2brghex(resized_img[j][(100+i)%200])
    led_data.append(buff)
led_data = [j for i in led_data for j in i]

cv2.imwrite('stripe2.png',resized_img)
print(a)
print(len(led_data))
f = open('main/globe_pattern.h', 'w')
f.write('#include "esp_system.h" \nconst uint32_t globepattern[] = {' + ','.join(led_data) + '};')
f.close()


