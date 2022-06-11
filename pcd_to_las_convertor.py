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
import open3d as o3d # Using open 3d software as visualizer
import numpy as np
import matplotlib.pyplot as plt
import skimage.io
from skimage.transform import resize
#if __name__ == "__main__":
    #o3d.visualization.webrtc_server.enable_webrtc() # To enable visualization on web browser rather than local window
n1 = 15000 #Pixel size in x
n2 = 12000 #Pixel size in y
X = np.linspace(-1, 1, num=n1) # Meshgrid in X
Y = np.linspace(-1, 1, num=n2) #Meshgrid in Y``
[XX, YY] = np.meshgrid(X, Y)# Create  meshgrid of X and Y
Z = -(XX)**2 -  (YY)**2 # Create parabola surface
Znorm = -2*XX - 2*YY
noise = np.random.normal(0, 1, n1*n2)  
noise = np.reshape(noise, (n2, n1))*0.1 # The multiplicative factor multiplies a reasonable amount of noise to be added to the parabolic surface...
Znoise = Z+noise # Adding noise to the parabola
        #fig = plt.figure()
        #ax = fig.add_subplot(projection='3d')
        #ax.scatter(XX.reshape((-1, 1)), YY.reshape((-1, 1)), Znoise.reshape((-1, 1)))
img = skimage.io.imread('image.jpeg')
img_resize = resize(img, (n1, n2), anti_aliasing = True)
rgbd = np.hstack((XX.reshape((-1,1)), YY.reshape((-1,1)), Z.reshape((-1,1)), img_resize[:,:, 0].reshape((-1, 1)), img_resize[:,:, 1].reshape((-1, 1)), img_resize[:,:, 2].reshape((-1, 1))))

pcd = rgbd[:, :3]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(rgbd[:, :3])
pcd.colors = o3d.utility.Vector3dVector(rgbd[:, 3:])

#alpha = 0.0001
#mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
#mesh.compute_vertex_normals()

#open3d.io.write_triangle_mesh("sample.gltf", mesh, write_ascii=False, compressed=False, write_vertex_normals=True, write_vertex_colors=True, write_triangle_uvs=True, print_progress=False)

#with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
 #   mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
#print(mesh)


#o3d.io.write_triangle_mesh(output_path+"bpa_mesh.gltf", dec_mesh)
#o3d.io.write_point_cloud("testname.glb", pcd, write_ascii=False, compressed=False, print_progress=False)



    ##Meshing
#distances = pcd.compute_nearest_neighbor_distance()
#avg_dist = np.mean(distances)
#radius = 3 * avg_dist
    
#bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))
    #o3d.visualization.draw([pcd]) # Draws the parabolic visualization
    
#bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))
    
#dec_mesh = mesh.simplify_quadric_decimation(100000)
#dec_mesh.remove_degenerate_triangles()
#dec_mesh.remove_duplicated_triangles()
#dec_mesh.remove_duplicated_vertices()
#dec_mesh.remove_non_manifold_edges()



#Compute surface normals

pcd.estimate_normals()

#o3d.io.write_triangle_mesh(output_path+"bpa_mesh.gltf", dec_mesh)
    #o3d.io.write_triangle_mesh(output_path+"p_mesh_c.gltf", p_mesh_crop)
o3d.io.write_point_cloud("sample_point_cloud.pcd", pcd)

