#pragma once

const char* g_about_help_controls =
R"(Controls:
    RMB         - enter freelook
    Ctrl + LMB  - focus on selected point

    WASD        - (in freelook) move camera
    Space       - (in freelook) raise camera
    Ctrl        - (in freelook) lower camera

    Tab         - toggle control panel
    Shift + LMB - enter slider value

Third party code:
    SDL2
    MicroUI
    Robust Win32 IO by Charles Bloom
    stb_image.h

See the "About" buttons under some headers for additional information on the options provided.

Unnamed pathtracer by Daniel Cornelisse)";

const char* g_about_sampling_strategies =
R"(-------------------------------------

Uniform:

The uniform sampler simply picks completely random white noise samples every time.

-------------------------------------

Optimized Blue Noise:

The optimized blue noise sampler uses sample code provided by the authors of this paper:
https://hal.archives-ouvertes.fr/hal-02150657/document
It uses Owen-scrambled Sobol sequences with state of the art convergence properties, and distributes their Monte Carlo errors as blue noise in screen space without compromising their convergance properties.
It is here primarily as a point of reference to a state of the art sampling strategy. However, it relies on precomputed tables and the authors only provide up to 256 samples for 8 dimensions, so when I exceed these bounds I fall back to the stratified sampler.

-------------------------------------

Stratified:

This is my own implementation of a stratified sampler, using 64 (or 8x8) strata to divide up the sample space, distributing random samples within each strata evenly.
It permutes the order in which strata are sampled in order to make the sampling unbiased even when not having sampled all strata - making it suitable for a progressive renderer.

-------------------------------------)";

const char* g_about_integrators =
R"(-------------------------------------

Advanced Pathtracer:

This is my pathtracer which implements the bulk of my functionality, representing the best I have to offer.
It supports next event estimation, importance sampling for direct and indirect light, multiple importance sampling allowing contribution from the same light sources via both random walk and next event estimation, and russian roulette for variance reduction.
In addition, it supports nested dielectrics so long as each nested primitive is fully contained within the other, allowing for things like hollow glass spheres.

-------------------------------------

Whitted:

This is my whitted (kind of, it also has distributed ray tracing techniques applied for area lights) raytracer.
Be careful with the maximum bounce count when using the whitted integrator with a complex translucent primitive. The way a whitted style raytracer splits into a reflection and refraction ray at every surface interaction can get out of hand very fast. However, the UI stays fully responsive even if the render does not, so don't panic!

-------------------------------------

Ground Truth Recursive:

This is a simple recursive pathtracer that only supports diffuse dielectrics, written to test the correctness of the iterative ground truth pathtracer.

-------------------------------------

Ground Truth Iterative:

This is a simple recursive pathtracer that only supports diffuse dielectrics, written to test the correctness of the advanced pathtracer.

-------------------------------------

Normals:

This integrator just displays surface normals.

-------------------------------------

Distances:

This integrator just displays distances of objects in monochrome.

-------------------------------------)";

const char* g_about_reconstruction_filters =
R"(The selected reconstruction filter is used to splat samples onto the accumulation buffer, to provide higher-quality anti-aliasing than just regularly rounding to whichever pixel the sample falls in, which would be a box filter.

-------------------------------------

Box:

The default for anyone who does not use a reconstruction filter - a box filter, implicit when simply outputting samples for the pixel they fall in.
To see the issue with this approach, look at the edge of any sufficiently bright part of the image, the high dynamic range of bright samples dominates the average, and causes the resulting edge to be aliased no matter how many samples you throw at it.

-------------------------------------

Gaussian 3:

A gaussian kernel with a radius of 3, this filter will provide a softer image with no ringing.

-------------------------------------

Gaussian 12:

A gaussian blur, at this radius of 12. Here for proof of concept / amusement.

-------------------------------------

Mitchell Netravali:

High quality 2x2 filter constructed by Mitchell and Netravali specifically for image reconstruction/resampling. Mild ringing, good balance between soft and sharp.

-------------------------------------

Lanczos 3:

Lanczos reconstruction filter with a radius and window of 3. Provides a sharp image at the cost of high ringing.

-------------------------------------

Lanczos 4-12:

Increasingly wide lanczos kernels, with increasingly whacky ringing. Here for exploration / amusement.

-------------------------------------)";
