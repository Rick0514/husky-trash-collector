import math
import numpy as np

deg_to_rad = 1. / 180 * math.pi

def get_circle(X, Y, R):

    theta = np.arange(20, 360, 20) * deg_to_rad
    x = np.zeros_like(theta)
    y = np.zeros_like(theta)

    for i in range(len(theta)):
        x[i] = X - R * math.cos(theta[i])
        y[i] = Y - R * math.sin(theta[i])
    
    return x, y



if __name__ == "__main__":

    x, y = get_circle(8, 0, 6)

    file = '/home/rick/github-demo/decompose-lvisam/src/husky/husky_description/config/vp-circle0.txt'

    with open(file, 'w') as f:
        for i in range(len(x)):
            f.write(str(x[i]) + ' ' + str(y[i]) + '\n')
