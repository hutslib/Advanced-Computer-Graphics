This is the repository for COMP5411 Advantage Computer Graphics in HKUST (year2021). 

In this class I have complete some assignments and a final project.

Assignments include:
- Laplacian Smoothing
  - explicite 
  - implicite
- Naive Laplacian Deformation
Final Project:
- Rendering Fur On Bunny

# Final Project : Rendering Fur On Bunny

<p>Our group using webgl and three.js to render fur on a bunny object. We also render a background scenario and illuminate some light effects.</p>
<img src="./img/summary.png" alt="Summary" width="60%" height="auto" >

Here is the proposal video for our project:

<iframe width="100%" style="height: calc(60vw); max-height: 512px" src="https://youtu.be/03RzWorSsOs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Please check more details in the [proposal video](https://youtu.be/03RzWorSsOs).

#### The major challenges we faced are: 
  
- How to generate a real-time realistic fur 
- How to import a complex obj module into the program
- How to set background scene and finish the light illumination

### 1. How to generate a real-time realistic fur:
<p>First, we sample some points on each mesh of the bunny randomly. How many points we sample decides the fur density. We call this plane with sampled points one slice. </p>
<img src="./img/realistic-fur_1.png" alt="realistic-fur_1" width="60%" height="auto" >
<p>Then we grow slices on the direction of mesh normal. It means we convert a fur fiber into several fur points along the mesh normal direction.</p>
<img src="./img/realistic-fur_2.png" alt="realistic-fur_2" width="60%" height="auto" >
<p>And when one piece of fur slice moves around, all the fibers in this group moves. This operation fastens the rendering computation efficiently to avoid calculating each fiber’s movement.</p>

### 2. How to import a complex obj module into the program:
<p>Our bunny.obj files contains faces and vertices. However, we also need to know the normals of each face and how does the texture map onto each slice. So we utilize blender to pre-compute these normals and texture maps.</p>
<img src="./img/module_1.png" alt="module_1" width="60%" height="auto" >
<p>Then we found that .obj can’t save these parameters. So we convert the models into .js file which contains the whole information.</p>
<img src="./img/module_2.png" alt="module_2" width="60%" height="auto" >
<p>What’s more, we also remesh the bunny to have a uniform density of fur. And we do the smooth operation to have a soft varying normals.</p>
<img src="./img/module_3.png" alt="module_3" width="60%" height="auto" >

### 3. How to set background scene and finish the light illumination:
<p>We add background texture to simulate bunny is on a moon and using the phong illumination model and phong shading algorithm.</p>
<img src="./img/phong-illumination-module.png" alt="phong-illumination-module" width="60%" height="auto" >

### video:

Here is the firnal report video for our project:

<iframe width="100%" style="height: calc(60vw); max-height: 512px" src="https://youtu.be/KoC0cmfwZ74" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Please check more details in the [final video](https://youtu.be/KoC0cmfwZ74).
