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

![balls_high](1º%20Project/Previews/balls_high.png)

2. Stochastic sampling techniques
    1. Anti-aliasing with the jittered method
    2. Soft shadows using an area of light with a set of N light source points (without
antialiasing) and the random method (with antialiasing)
    3. Depth of field effect where the lens is simulated by a random distribution of N
samples on a unit disk 

![dof](1º%20Project/Previews/dof.png)

3. Acceleration structure
    1. Uniform Grid integration
    2. Bounding Volume Hierarchy (BVH)

![dragon](1º%20Project/Previews/dragon.png)

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

![pathtracer](1º%20Project/Previews/pathtracer.png)

# 2º Project
## Unreal Engine 5
![demo](2º%20Project/Previews/Demo_Group6.gif)

1) A full 3D scene including different spaces, configuration, textures and lights
2) Objects that illustrate different types of collisions
a. Player control: destroy/pick up objects, push objects
b. Change the environment of the scene based on trigger events. Examples include turn the lights on/off, change textures, and so forth
c. Have autonomous objects like a ceiling fan
3) Visual effects
a. Different types of materials and lights, , textures; and post-processing 
b. Particle systems to convey smoke, fog, fire, explosion, etc.
c. Global Illumination effects (eg. Ambient Occlusion, Reflections, Refractions, Shadows - etc).

![Axes](2º%20Project/Previews/Axes.png)

![EasterEgg](2º%20Project/Previews/Easter%20Egg.png)