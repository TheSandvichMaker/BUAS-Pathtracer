#pragma once

struct Mesh;

union RadianceColor {
    struct {
        u8 r, g, b, exp;
    };
    u8 e[4];
};

enum RadianceFormat {
    Radiance_RGB,
    Radiance_XYZ,
};

struct RadianceImage {
    u32 w, h;

    V2 r_primary;
    V2 g_primary;
    V2 b_primary;
    V2 w_primary;

    RadianceFormat format;
    RadianceColor* pixels;
};

struct Image_V3 {
    u32 w, h;
    V3* pixels;
};

struct EnvironmentMap : Image_V3 {
    u32 tile_w, tile_h;
    f32* cdf;
};

struct Color_R8 {
    u8 r;
};

struct Image_R8 {
    u32 w, h;
    Color_R8* pixels;
};

struct Color_R8G8 {
    u8 r, g;
};

struct Image_R8G8 {
    u32 w, h;
    Color_R8G8* pixels;
};

inline Color_R8G8
sample_image_wrapping(Image_R8G8* image, u32 x, u32 y) {
    x = x % image->w;
    y = y % image->h;
    Color_R8G8 result = image->pixels[y*image->w + x];
    return result;
}

struct Color_R8G8B8 {
    u8 r, g, b;
};

struct Image_R8G8B8 {
    u32 w, h;
    Color_R8G8B8* pixels;
};

enum MeshWinding {
    MeshWinding_Clockwise,
    MeshWinding_CounterClockwise,
};

//

extern Image_R8     g_blue_noise1[8];
extern Image_R8G8   g_blue_noise2[8];
extern Image_R8G8B8 g_blue_noise3[8];

//

b32 parse_obj(Arena* arena, Arena* temp_arena, char* input, Mesh* mesh, MeshWinding winding);
b32 parse_hdr(Arena* arena, Arena* temp_arena, char* input, Image_V3* image);
void write_bitmap(u32* pixels, u32 w, u32 h, const char* file_name, Arena* temp_arena);
b32 load_image_r8(const char* file_name, Image_R8* out_image);
b32 load_image_r8g8(const char* file_name, Image_R8G8* out_image);
b32 load_image_r8g8b8(const char* file_name, Image_R8G8B8* out_image);
void load_blue_noise_textures();
EnvironmentMap* load_environment_map(Arena* arena, Arena* temp_arena, const char* file_name);
