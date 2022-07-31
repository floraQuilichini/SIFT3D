# SIFT3D
This code uses PCL.1.8.1 functions to compute SIFT3D detector on a point cloud/mesh, that is a 3D version of the image SIFT detector (https://link.springer.com/content/pdf/10.1023/B:VISI.0000029664.99615.94.pdf). <br />
It takes as input the filename of the mesh, the resolution of the mesh (ie mean edge length if mesh or mean distance to neighbors if point cloud), and the output filename where to write the selected keypoints. <br />
