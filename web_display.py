# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import skimage.io
from skimage.transform import resize
if __name__ == "__main__":
	o3d.visualization.webrtc_server.enable_webrtc()
	n1 = 15000
	n2 = 12000
	X = np.linspace(-1, 1, num=n1) # Meshgrid in X
	Y = np.linspace(-1, 1, num=n2) #Meshgrid in Y``
	[XX, YY] = np.meshgrid(X, Y)# Create  meshgrid of X and Y
	Z = 1-(XX)**2 -  (YY)**2
	noise = np.random.normal(0, 1, n1*n2) # The multiplicative factor multiplies a reasonable 		amount of noise to the parabolic surface... 
	noise = np.reshape(noise, (n2, n1))*0.1
	Znoise = Z+noise
	#fig = plt.figure()
	#ax = fig.add_subplot(projection='3d')
	#ax.scatter(XX.reshape((-1, 1)), YY.reshape((-1, 1)), Znoise.reshape((-1, 1)))
	img = skimage.io.imread('image.jpeg')
	img_resize = resize(img, (n2, n1), anti_aliasing = True)
	rgbd = np.hstack((XX.reshape((-1,1)), YY.reshape((-1,1)), Z.reshape((-1,1)), img_resize[:,:, 0].reshape((-1, 1)), img_resize[:,:, 1].reshape((-1, 1)), img_resize[:,:, 2].reshape((-1, 1))))
	pcd = rgbd[:, :3]
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(rgbd[:, :3])
	o3d.visualization.draw([pcd])
