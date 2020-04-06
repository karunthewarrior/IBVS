import numpy as np 


def cyclic_sort(pts):
    center = np.mean(pts,axis=1)
    v = pts - center.reshape(-1,1)
    ind = np.argsort([convert_angle(np.rad2deg(math.atan2(y,x))) for x,y in v.T])
    return (pts[:,ind])
    
def convert_angle(x):
    if x < 0:
        return 360 + x
    else:
        return x
        
if __name__ == "__main__":
    im_pts = cyclic_sort(np.array([[304, 267, 267, 304], 
                                   [261, 261, 223, 223]]))
    img_dist = np.linalg.norm(im_pts - np.roll(im_pts,1,axis=1),axis=0)
    cam_dist = np.array([0.3,0.3,0.3,0.3])
    print(img_dist)