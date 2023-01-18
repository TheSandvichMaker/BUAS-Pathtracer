# BUAS-Pathtracer

![image](https://user-images.githubusercontent.com/49493579/128134150-dd9cebaa-3685-4101-831f-3b3339c7b460.png)
![image](https://user-images.githubusercontent.com/49493579/213304939-6876cd94-92e5-4b23-94fe-d961f3999895.png)
![image](https://user-images.githubusercontent.com/49493579/213304967-1c23fa52-89e7-4da0-be57-d01517cd5b5b.png)
![image](https://user-images.githubusercontent.com/49493579/128134270-c8062196-6d46-4692-b9ef-9077d705dc7a.png)
![Raytracer_24yrwS7zhg](https://user-images.githubusercontent.com/49493579/213315016-f0acf76d-d966-4130-9b83-b74b15600252.png)
![Raytracer_LdgeBg9c2O](https://user-images.githubusercontent.com/49493579/213305512-b4f7cb0e-5b0d-49da-8849-7f123480283f.png)

# Description
A CPU pathtracer written as my second project in my first year at the Breda University of Applied Sciences' games programming course over the course of 8 weeks.

# Features
- Realtime preview with camera controls and configuration
- Bounding Volume Hierarchy acceleration structure based on Igno Wald's 2007 paper [On fast Construction of SAH-based Bounding Volume Hierarchies](http://sci.utah.edu/~wald/Publications/2007/ParallelBVHBuild/fastbuild.pdf), for triangle meshes and top level acceleration structure.
- Tile-based multithreading
- HDR environment maps
- Spherical area lights
- Next event estimation
- Russian roulette path termination
- Cosine hemisphere importance sampling
- Mitchell-Netravali Reconstruction Filter
- Stratified sampling
- Nested dielectrics using a material stack
- Tonemapped, sRGB corrected dithered output
- [microui](https://github.com/rxi/microui) based user interface
- Render to bitmap
