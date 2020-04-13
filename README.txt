CSC305 Assignment4 Ray Tracing
Tong Zhang
V00926513
04-12-2020
***************************************

Basic Features (All implemented)
---------------------------------
 -A virtual camera system:
 	Pinhole Camera setup at line#2241
 -point lights:
	Point lights setup at line#2151
	the hard shadows come from point light
	the soft shadows come from arealight
 -diﬀuse shading:
	Matte for floor&wall, Phong for some balls
	Specular also implemented
 -multisampling:
	Multijittered
 -out-of-gamut colour handling:
	colour_handling() at line#2466
 -a variety of geometries:
	Triangle, Rectangle, Plane, Sphere, Box,
	Disk, OpenCylinder, Cylinder, Cone...
	geometries setup at line#2026


Optional Features (15 marks implemented)
---------------------------------
 -Medium parallelization (2 marks)
	divide the image into 100 slabs, distributed
	across STL multi-threads
	Rendered with multi-threads:  875.32s
	      without multi-threads:  1548.43s
		            speedup:  56.53%
	comment line#2274 and comment out #2278 to enable
	(Multi-threads works perfectly on my 2017 Macbook Pro	
         But may has the possiblitiy doesn't fit lab machines
	 if corruption, please disable multi-threads)

 -Implemented regular grids (1 mark)
	Rendered with grids time:  1546.14s, 
	      without grids time:  2722.07s, 
		         speedup:  43.2%
	Default turn on. Comment out #2077-2092 and 
	comment #2096-2114 to disable grids

 -Implemented mirror reﬂections (1 mark)
	Setup at line#1770 see the relection of 
	an aluminum ball at the left of pic.
	an bronze ball in the middle

 -Implemented glossy reﬂections (1 mark)
	Under the Arealighting, Blue and green glossy balls setup #1846

 -Implemented simple path tracing (1 mark)
	Implemented in assignment.hpp line#3933-3972
	and most of the materials

 -Implemented simple transparency (1 mark)
	Regular Transparent material setup at line# 1887

 -Implemented realistic transparency (2 mark)
	FresnelReflector at line#673 in assignment.hpp
	FresnelTransmitter at line#870 in assignment.hpp
	Dielectirc material class at line#2997-3354 in assignment.hpp
	Dielectric material setup at line# 1870 Switch to Whitted tracer
	to see the transparent ball at the right front of pic

 -Implemented regular texture mapping (1 mark)
	RectangularMap at line#6398 in assignment.hpp
	CylindricalMap at line#6485 in assignment.hpp
	SphericalMap at line#6436 in assignment.hpp
	See the earth in the rear middle of the picture 

 -Implemented procedural texture (1 mark)
	2D&3D checker textures, and texture transformations, 
	TInstance class. See the checker box and checker sphere 
	in the pic

 -Implemented shadows (1 mark)
	Shadows for all objects see in the pic

 -Implemented ambient occlusion (1 mark)
	AmbientOccluder at line#2144, default is turn on
	if you want to switch to regular, comment it and comment
	out upper one.

 -Implemented area lights (1 mark)
	Setup at line#2191, 2 rectangular arealights is set
	See the upper ceilling of pic

 -Rendered a mesh (1 mark)
	A mesh generated form tessellate_flat_sphere rendered.
	It's a 10*6 flat mesh. Setup at line#2021
	See the "diamond" in the middle rear of pic



Files: please put all files under the same directory
---------------------------------
	main.cpp
	assignment.hpp
	CMakeLists.txt
	paths.hpp.in
	EarthWithClouds.ppm