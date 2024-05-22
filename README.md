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

![balls_high](https://cdn.discordapp.com/attachments/1077928687447572513/1239693515152429096/balls_high.png?ex=6643da0e&is=6642888e&hm=642ba47be4d6fc0cb21f1dafa76644d32fd99183a9f31890534d6a509e7e4618&)

2. Stochastic sampling techniques
    1. Anti-aliasing with the jittered method
    2. Soft shadows using an area of light with a set of N light source points (without
antialiasing) and the random method (with antialiasing)
    3. Depth of field effect where the lens is simulated by a random distribution of N
samples on a unit disk 

![dof](https://cdn.discordapp.com/attachments/1077928687447572513/1239693531988230146/dof.png?ex=6643da12&is=66428892&hm=8c523fc96d0bcbcc1257feab9f607ff43d6d38ea28772b31614f3eceb69f040c&)

3. Acceleration structure
    1. Uniform Grid integration
    2. Bounding Volume Hierarchy (BVH)

![dragon](https://cdn.discordapp.com/attachments/1077928687447572513/1239693679774531725/RT_Output.png?ex=6643da35&is=664288b5&hm=c07db8b63aef4741072ee4b78d52267b9edcffa2b6fe086d5fe539acc675cc98&)

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

![pathtracer](https://cdn.discordapp.com/attachments/1077928687447572513/1239698009525059594/image.png?ex=664a75bd&is=6649243d&hm=3288924bb3daac57be7e3c99116e3685ba2c4a3f4e010e7248b12e9543da5e72&)
