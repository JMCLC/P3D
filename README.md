# 3D Programming

# 1º Project
## CPU RayTracer
### Tasks

1. T. Whitted Ray-Tracer
    1. Ray intersections with spheres, triangles, and axis-aligned boxes
    2. Local Blinn-Phong reflection model 
    3. Multiple source lights and Hard Shadows
    4. Global color component by implementing the mirror reflection and refraction with
Schlick approximation of Fresnel Equations for dieletric materials

![balls_high]("1º Project/Previews/balls_high.png")

2. Stochastic sampling techniques
    1. Anti-aliasing with the jittered method
    2. Soft shadows using an area of light with a set of N light source points (without
antialiasing) and the random method (with antialiasing)
    3. Depth of field effect where the lens is simulated by a random distribution of N
samples on a unit disk 

![dof]("1º Project/Previews/dof.png")

3. Acceleration structure
    1. Uniform Grid integration
    2. Bounding Volume Hierarchy (BVH)

![dragon]("1º Project/Previews/dragon.png")

## GPU PathTracer

### Tasks

1. Ray intersections with spheres and triangles
2. Local color, multiple source lights, hard Shadows
3. Global color
    1. Diffuse reflections for color bleeding
    2. Mirror and fuzzy specular reflections for metallic objects
    3. Reflection and refraction with Schlick approximation of Fresnel Equations for dieletric
transparent materials
4. Motion blur with spheres
5. Depth-of-Field
6. Beer’s law for light absorption inside a transparent dieletric 

![pathtracer]("1º Project/Previews/pathtracer.png")
