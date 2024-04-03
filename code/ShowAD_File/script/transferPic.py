import numpy as np
import os
import cv2

def bgr2nv12_opencv(image):
    height, width = image.shape[0], image.shape[1]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

    nv12 = np.zeros_like(yuv420p)
    nv12[:height * width] = y
    nv12[height * width:] = uv_packed
    return nv12

def bgr2nv12(bgr:np.ndarray) -> np.ndarray:
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    #  print(yuv.shape[0])
    uv_row_cnt = yuv.shape[0]// 3
    uv_plane = np.transpose(yuv[uv_row_cnt * 2:].reshape(2, -1), [1, 0])
    yuv[uv_row_cnt * 2:] = uv_plane.reshape(uv_row_cnt, -1)
    return yuv

def bgr2nv21(bgr:np.ndarray) -> np.ndarray:
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    uv_row_cnt = yuv.shape[0] // 3
    u_row_cnt = uv_row_cnt // 2
    u_plane = yuv[uv_row_cnt * 2,uv_row_cnt * 2 + u_row_cnt]
    v_plane = yuv[uv_row_cnt * 2 + u_row_cnt,uv_row_cnt * 3]
    vu_plane = np.zeros((uv_row_cnt,yuv.shape[1]))
    vu_plane[:u_row_cnt] = v_plane
    vu_plane[u_row_cnt,uv_row_cnt] = u_plane
    vu_plane_ = np.transpose(vu_plane.reshape(2, -1), [1, 0])
    yuv[uv_row_cnt * 2:] = vu_plane_.reshape(uv_row_cnt, -1)
    return yuv


def process(input_path):
    try:
        input_image = cv2.imread(input_path)
        #  h,w,_ = input_image.shape
        resize_image = cv2.resize(input_image,(1920,1040))
        #  resize_image = input_image 
        print(resize_image.shape)
        height, width, _ = resize_image.shape  
        row_padding = (1080 - height) // 2
        col_padding = (1920 - width) // 2
        #  print(row_padding,":",col_padding)

        padded_image = np.zeros((1080, 1920, 3), dtype=np.uint8)
        padded_image[row_padding:row_padding+height, col_padding:col_padding+width, :] = resize_image
        #  print(resize_image.shape)
        result = bgr2nv21(padded_image)
        output_name = os.path.basename(input_path).split('.')[0] + "_nv21.bin"
        output_path = os.path.join("./out",output_name)
        result.tofile(output_path)
    except Exception as except_err:
        print(except_err)
        return 1
    else:
        return 0
if __name__ == "__main__":
    count_ok = 0
    count_ng = 0
    images = os.listdir(r'./')
    for image_name in images:
        if not image_name.endswith("png"):
            continue
        print("start to process image {}....".format(image_name))
        ret = process(os.path.join("./",image_name))
        if ret == 0:
            print("process image {} successfully".format(image_name))
            count_ok = count_ok + 1
        elif ret == 1:
            print("failed to process image {}".format(image_name))
            count_ng = count_ng + 1
    print("{} images in total, {} images process successfully, {} images process failed"
          .format(count_ok + count_ng, count_ok, count_ng))
