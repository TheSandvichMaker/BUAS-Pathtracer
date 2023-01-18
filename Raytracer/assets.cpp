#include "precomp.h"
#include "common.h"
#include "primitives.h"
#include "assets.h"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#pragma warning(push)
#pragma warning(disable: 26819 6262 28182 6011 26451 6308)
#include "stb_image.h"
#pragma warning(pop)

Image_R8     g_blue_noise1[8];
Image_R8G8   g_blue_noise2[8];
Image_R8G8B8 g_blue_noise3[8];

//
// NOTE: stb image loading
//

static b32
load_ldr_image(const char* file_name, int channels, u8** out_pixels, u32* out_w, u32* out_h) {
    b32 result = false;

    int w, h, n;
    u8* pixels_rgba = stbi_load(file_name, &w, &h, &n, 4);
    if (pixels_rgba) {
        result = true;
        *out_w = w;
        *out_h = h;
        *out_pixels = (u8*)platform_allocate((usize)channels*w*h);
        for (usize y = 0; y < h; ++y)
        for (usize x = 0; x < w; ++x) {
            usize i = y*(w) + x;
            for (usize c = 0; c < channels; ++c) {
                (*out_pixels)[channels*i + c] = pixels_rgba[4*i + c];
            }
        }
        stbi_image_free(pixels_rgba);
    } else {
        const char* failure_reason = stbi_failure_reason();
        fprintf(stderr, "stb_image failed to load image '%s': %s\n", file_name, failure_reason);
    }

    return result;
}

b32
load_image_r8(const char* file_name, Image_R8* out_image) {
    return load_ldr_image(file_name, 1, (u8**)&out_image->pixels, &out_image->w, &out_image->h);
}

b32
load_image_r8g8(const char* file_name, Image_R8G8* out_image) {
    return load_ldr_image(file_name, 2, (u8**)&out_image->pixels, &out_image->w, &out_image->h);
}

b32
load_image_r8g8b8(const char* file_name, Image_R8G8B8* out_image) {
    return load_ldr_image(file_name, 3, (u8**)&out_image->pixels, &out_image->w, &out_image->h);
}

void
load_blue_noise_textures() {
    const char* texture_locations1[] = {
        "data/noise/LDR_LLL1_0.png",
        "data/noise/LDR_LLL1_1.png",
        "data/noise/LDR_LLL1_2.png",
        "data/noise/LDR_LLL1_3.png",
        "data/noise/LDR_LLL1_4.png",
        "data/noise/LDR_LLL1_5.png",
        "data/noise/LDR_LLL1_6.png",
        "data/noise/LDR_LLL1_7.png",
    };

    const char* texture_locations2[] = {
        "data/noise/LDR_RG01_0.png",
        "data/noise/LDR_RG01_1.png",
        "data/noise/LDR_RG01_2.png",
        "data/noise/LDR_RG01_3.png",
        "data/noise/LDR_RG01_4.png",
        "data/noise/LDR_RG01_5.png",
        "data/noise/LDR_RG01_6.png",
        "data/noise/LDR_RG01_7.png",
    };

    const char* texture_locations3[] = {
        "data/noise/LDR_RGB1_0.png",
        "data/noise/LDR_RGB1_1.png",
        "data/noise/LDR_RGB1_2.png",
        "data/noise/LDR_RGB1_3.png",
        "data/noise/LDR_RGB1_4.png",
        "data/noise/LDR_RGB1_5.png",
        "data/noise/LDR_RGB1_6.png",
        "data/noise/LDR_RGB1_7.png",
    };

    assert(ArrayCount(texture_locations1) <= ArrayCount(g_blue_noise1));
    assert(ArrayCount(texture_locations2) <= ArrayCount(g_blue_noise2));
    assert(ArrayCount(texture_locations3) <= ArrayCount(g_blue_noise3));

    for (usize i = 0; i < ArrayCount(texture_locations1); ++i) {
        load_image_r8(texture_locations1[i], &g_blue_noise1[i]);
    }

    for (usize i = 0; i < ArrayCount(texture_locations2); ++i) {
        load_image_r8g8(texture_locations2[i], &g_blue_noise2[i]);
    }

    for (usize i = 0; i < ArrayCount(texture_locations3); ++i) {
        load_image_r8g8b8(texture_locations3[i], &g_blue_noise3[i]);
    }
}

#define match_word(stream, word) match_word_(stream, sizeof(word) - 1, word)
static b32
match_word_(char** stream, u32 len, const char* word) {
    b32 result = false;

    char* at = *stream;

    // NOTE: Skip leading whitespace
    while (*at == ' ') ++at;

    if (0 == strncmp(at, word, len)) {
        result = true;
        *stream = at + len;
    }
    return result;
}

static b32
parse_f32(char** stream, f32* out_value) {
    b32 result = false;

    char* end;
    f32 value = strtof(*stream, &end);

    if (end != *stream) {
        result = true;

        *stream = end;
        *out_value = value;
    }

    return result;
}

static b32
parse_u32(char** stream, u32* out_value) {
    b32 result = false;

    char* end;
    u32 value = (u32)strtoul(*stream, &end, 0);

    if (end != *stream) {
        result = true;

        *stream = end;
        *out_value = value;
    }

    return result;
}

static b32
parse_s32(char** stream, s32* out_value) {
    b32 result = false;

    char* end;
    s32 value = (s32)strtol(*stream, &end, 0);

    if (end != *stream) {
        result = true;

        *stream = end;
        *out_value = value;
    }

    return result;
}

//
// NOTE: .obj
//

b32
parse_obj(Arena* arena, Arena* temp_arena, char* input, Mesh* mesh, MeshWinding winding) {
    b32 result = false;
    
    zero_struct(mesh);

    ScopedMemory scoped_memory(temp_arena);

    StretchyBuffer<V3> vertices            = StretchyBuffer<V3>::on_arena(temp_arena);
    StretchyBuffer<V3> texture_coordinates = StretchyBuffer<V3>::on_arena(temp_arena);
    StretchyBuffer<V3> normals             = StretchyBuffer<V3>::on_arena(temp_arena);

    vertices           .add(v3(0, 0, 0)); // NOTE: NULL vertex
    texture_coordinates.add(v3(0, 0, 0)); // NOTE: NULL texcoord
    normals            .add(v3(0, 0, 0)); // NOTE: NULL normal

    StretchyBuffer<Triangle> triangles                    = StretchyBuffer<Triangle>::on_arena(temp_arena);
    StretchyBuffer<Triangle> triangle_texture_coordinates = StretchyBuffer<Triangle>::on_arena(temp_arena);
    StretchyBuffer<Triangle> triangle_normals             = StretchyBuffer<Triangle>::on_arena(temp_arena);

    char* at = input;
    while (*at) {
        // NOTE: Skip leading whitespaces
        while (*at && (*at == ' ' || *at == '\t' || *at == '\r' || *at == '\n')) ++at;

        char* line_start = at;

        // NOTE: For convenience, let's just parse to the end of the line ahead of time
        char* line_end = at;
        while (*line_end && *line_end != '\r' && *line_end != '\n') ++line_end;

        char* next_line_start = line_end;
        if (*next_line_start == '\r') ++next_line_start;
        if (*next_line_start == '\n') ++next_line_start;

        switch (*at++) {
            default: {
                // NOTE: Skip comment / unknown command
            } break;

            case 'v': {
                StretchyBuffer<V3>* target_vertex_array = &vertices;
                if (*at == 'n') {
                    ++at;
                    target_vertex_array = &normals;
                } else if (*at == 't') {
                    ++at;
                    target_vertex_array = &texture_coordinates;
                }
                // NOTE: Parse vertex
                V3 v = {};
                for (usize i = 0; i < 3; ++i) {
                    char* end;
                    f32 e = strtof(at, &end);
                    if (end != at) {
                        v.e[i] = e;
                    }
                    at = end;
                }
                target_vertex_array->add(v);
            } break;

            case 'f': {
                struct Face {
                    u32 count;
                    u32 indices[32];
                };

                Face vertex_face  = {};
                Face texture_face = {};
                Face normal_face  = {};

                u32 source_counts[3] = { vertices.count, texture_coordinates.count, normals.count };
                Face*       faces[3] = { &vertex_face,   &texture_face,             &normal_face  };

                for (;;) {
                    for (u32 face_index = 0; face_index < ArrayCount(faces); ++face_index) {
                        Face* face = faces[face_index];

                        if (face->count >= ArrayCount(face->indices)) {
                            // NOTE: Too many vertices for a face (uhh, a face with > 32 vertices? okay)
                            fprintf(stderr, "OBJ PARSE ERROR: Too many vertices for face\n");
                            goto bail;
                        }

                        char* end;

                        s64 index = strtol(at, &end, 0);
                        if (index < 0) {
                            index = source_counts[face_index] + index;
                            assert(index > 0);
                        }

                        if (end != at) {
                            face->indices[face->count++] = (u32)index;
                        }

                        at = end;

                        if (*at == '/') {
                            ++at;
                        } else {
                            while (*at == ' ') ++at;
                            break;
                        }
                    }

                    if (at == line_end) {
                        break;
                    }
                }

                u32 a = 0;
                u32 b = 1;
                u32 c = 2;
                if (winding == MeshWinding_Clockwise) {
                    a = 2;
                    b = 1;
                    c = 0;
                } else {
                    assert(winding == MeshWinding_CounterClockwise);
                }

                StretchyBuffer<V3>* vertex_sources[3] = {
                    &vertices,
                    &texture_coordinates,
                    &normals,
                };

                StretchyBuffer<Triangle>* triangle_dests[3] = {
                    &triangles,
                    &triangle_texture_coordinates,
                    &triangle_normals,
                };

                for (usize face_index = 0; face_index < ArrayCount(faces); ++face_index) {
                    Face* face = faces[face_index];
                    if (face->count) { 
                        if (face->count < 3) {
                            fprintf(stderr, "OBJ PARSE ERROR: Not enough vertices to make a face.\n");
                            goto bail;
                        }
                        auto& src = *vertex_sources[face_index];
                        auto& dst = *triangle_dests[face_index];
                        // NOTE: Convert face into triangle fan
                        for (usize i = 1; i < face->count - 1; ++i) {
                            Triangle* tri = dst.add();
                            tri->v[a] = src[face->indices[0]];
                            tri->v[b] = src[face->indices[i]];
                            tri->v[c] = src[face->indices[i + 1]];
                        }
                    }
                }
            } break;
        }

        at = next_line_start;
    }

    if (triangle_texture_coordinates.count &&
        (triangle_texture_coordinates.count != triangles.count))
    {
        fprintf(stderr, "OBJ PARSE ERROR: Texture coordinates don't match triangles\n");
        goto bail;
    }

    if (triangle_normals.count &&
        (triangle_normals.count != triangles.count))
    {
        fprintf(stderr, "OBJ PARSE ERROR: Normals don't match triangles\n");
        goto bail;
    }

    if (texture_coordinates.count > 1) {
        mesh->has_texture_coordinates = true;
    }

    if (normals.count > 1) {
        mesh->has_normals = true;
    }

    mesh->triangle_count = triangles.count;

    {
        // NOTE: We'll write out all the triangle data sequentially so we can cut down on size
        //       size of the Mesh struct, storing only the head pointer.
        usize total_triangle_count = mesh->triangle_count*(1ull +
                                                           mesh->has_texture_coordinates +
                                                           mesh->has_normals);

        Triangle* triangles_at = push_array(arena, total_triangle_count, Triangle, no_clear());
        mesh->triangles = triangles_at;

        copy_array(mesh->triangle_count, triangles.data, triangles_at);

        if (mesh->has_texture_coordinates) {
            triangles_at += mesh->triangle_count;
            copy_array(mesh->triangle_count, triangle_texture_coordinates.data, triangles_at);
        }
        if (mesh->has_normals) {
            triangles_at += mesh->triangle_count;
            copy_array(mesh->triangle_count, triangle_normals.data, triangles_at);
        }
    }

    result = true;

bail:
    if (!result) {
        zero_struct(mesh);
    }

    return result;
}

//
// NOTE: .hdr
//

static f32
float_from_bits(u32 bits) {
    return (f32&)bits;
}

static V3
decode_radiance_color(RadianceColor col) {
    V3 result = {};
    if (col.exp > 9) {
        f32 mul = float_from_bits((col.exp - 9) << 23);
        result = v3(mul*((f32)col.r + 0.5f),
                    mul*((f32)col.g + 0.5f),
                    mul*((f32)col.b + 0.5f));
    }
    return result;
}

b32
parse_hdr(Arena* arena, Arena* temp_arena, char* input, Image_V3* image) {
    b32 result = false;

    zero_struct(image);

    // NOTE: The pixels will be pushed into this arena, we will commit the temporary memory
    //       if we successfully parse, otherwise it will automatically be freed by the scoped memory.
    ScopedMemory result_memory(arena);

    s32 x_advance =  1;
    s32 y_advance = -1;

    image->w = 0;
    image->h = 0;

    RadianceFormat format = Radiance_RGB;
    V2 r_primary = {};
    V2 g_primary = {};
    V2 b_primary = {};
    V2 w_primary = {};

    char* at = input;
    // NOTE: Parse header
    while (*at) {
        if (*at == '\n') {
            // NOTE: End of header
            ++at;
            break;
        } else {
            if (match_word(&at, "FORMAT")) {
                if (!match_word(&at, "=")) {
                    fprintf(stderr, "HDR PARSE ERROR: Malformed header.\n");
                    goto bail;
                }
                if (match_word(&at, "32-bit_rle_rgbe")) {
                    format = Radiance_RGB;
                } else if (match_word(&at, "32-bit_rle_xyz")) {
                    format = Radiance_XYZ;
                } else {
                    fprintf(stderr, "HDR PARSE WARNING: Unknown FORMAT, defaulting to RGB\n");
                }
            } else if (match_word(&at, "PRIMARIES")) {
                if (!match_word(&at, "=")) {
                    fprintf(stderr, "HDR PARSE ERROR: Malformed header.\n");
                    goto bail;
                }
                if (!parse_f32(&at, &r_primary.x) ||
                    !parse_f32(&at, &r_primary.y) ||
                    !parse_f32(&at, &g_primary.x) ||
                    !parse_f32(&at, &g_primary.y) ||
                    !parse_f32(&at, &b_primary.x) ||
                    !parse_f32(&at, &b_primary.y) ||
                    !parse_f32(&at, &w_primary.x) ||
                    !parse_f32(&at, &w_primary.y))
                {
                    fprintf(stderr, "HDR PARSE WARNING: Failed to correctly parse PRIMARIES. Using defaults.\n");
                    r_primary = v2(0.640f, 0.330f);
                    g_primary = v2(0.290f, 0.600f);
                    b_primary = v2(0.150f, 0.060f);
                    w_primary = v2(0.333f, 0.333f);
                }
            }
        }

        // NOTE: Skip unhandled lines
        while (*at && *at != '\n') ++at;
        if (*at == '\n') ++at;
    }

    if (format == Radiance_XYZ) {
        fprintf(stderr, "HDR PARSE WARNING: XYZ encoding is not handled. Who knows what your image will look like.\n");
    }

    if (!*at) {
        fprintf(stderr, "HDR PARSE ERROR: Unexpected end of file while parsing header.\n");
        goto bail;
    }

    // NOTE: Parse resolution string
    if (match_word(&at, "+Y")) {
        y_advance =  1;
    } else if (match_word(&at, "-Y")) {
        y_advance = -1;
    } else {
        fprintf(stderr, "HDR PARSE ERROR: Failed to parse resolution string (+/-Y).\n");
        goto bail;
    }

    if (!parse_u32(&at, &image->h)) {
        fprintf(stderr, "HDR PARSE ERROR: Failed to parse vertical resolution.\n");
        goto bail;
    }

    if (match_word(&at, "+X")) {
        x_advance =  1;
    } else if (match_word(&at, "-X")) {
        x_advance = -1;
    } else {
        fprintf(stderr, "HDR PARSE ERROR: Failed to parse resolution string (+/-X).\n");
        goto bail;
    }

    if (!parse_u32(&at, &image->w)) {
        fprintf(stderr, "HDR PARSE ERROR: Failed to parse horizontal resolution.\n");
        goto bail;
    }

    if (*at++ != '\n') {
        fprintf(stderr, "HDR PARSE ERROR: Expected newline after resolution string.\n");
        goto bail;
    }

    if (image->w && image->h) {
        ScopedMemory scoped_memory(temp_arena);

        RadianceColor* radiance_pixels = push_array(temp_arena, (usize)image->w*image->h, RadianceColor);
        RadianceColor* dst_row = radiance_pixels;

        if (x_advance < 0) dst_row += (usize)image->w - 1;
        if (y_advance < 0) dst_row += (usize)image->w*(image->h - 1);
        for (usize y = 0; y < image->h; ++y) {
            u16 signature = (at[0] << 8)|at[1];
            at += 2;

            if (signature != 0x0202) {
                fprintf(stderr, "HDR PARSE ERROR: .hdr format unsupported.\n");
                goto bail;
            }

            u16 scanline_length = (at[0] << 8)|at[1];
            at += 2;

            if (scanline_length != image->w) {
                fprintf(stderr, "HDR PARSE ERROR: Scanline length did not match image width.\n");
                goto bail;
            }

            // NOTE: Yes, that work up above is completely useless, this format is a bit shit.

            for (usize channel_index = 0; channel_index < 4; ++channel_index) {
                RadianceColor* dst = dst_row;
                for (usize x = 0; x < image->w;) {
                    u8 code = *at++;
                    if (code > 128) {
                        // NOTE: Run
                        u8 run_count = code & 127;
                        u8 run_value = *at++;
                        while (run_count--) {
                            assert_slow(x < image->w);
                            dst->e[channel_index] = run_value;
                            dst += x_advance;
                            ++x;
                        }
                    } else {
                        // NOTE: Literals
                        u8 literal_count = code;
                        while (literal_count--) {
                            assert_slow(x < image->w);
                            dst->e[channel_index] = *at++;
                            dst += x_advance;
                            ++x;
                        }
                    }
                }
            }

            dst_row += y_advance*(ssize)image->w;
        }

        image->pixels = push_array(arena, image->w*image->h, V3, no_clear());

        RadianceColor* src = radiance_pixels;
        V3* dst = image->pixels;
        for (usize i = 0; i < (usize)image->w*image->h; ++i) {
            *dst++ = decode_radiance_color(*src++);
        }
    } else {
        fprintf(stderr, "HDR PARSE ERROR: Malformed resolution.\n");
        goto bail;
    }

    if (*at != '\0') {
        fprintf(stderr, "HDR PARSE WARNING: Expected end of file, but there's more!\n");
    }

    result = true;
    commit_temporary_memory(result_memory);

bail:
    if (!result) {
        zero_struct(image);
    }

    return result;
}

EnvironmentMap*
load_environment_map(Arena* arena, Arena* temp_arena, const char* file_name) {
    EnvironmentMap* result = nullptr;

    ScopedMemory scoped_memory(temp_arena);

    EnvironmentMap map;
    char* image_file = platform_read_entire_file(temp_arena, file_name);
    if (image_file && parse_hdr(arena, temp_arena, image_file, &map)) {
        result = push_copy_struct(arena, &map);
        result->tile_w = result->w / 32;
        result->tile_h = result->h / 32;

        usize cdf_size = (usize)result->tile_w*result->tile_h;
        result->cdf = push_array(arena, cdf_size, f32);

        f32 sum = 0.0f;
        usize i = 0;
        for (usize y = 0; y < result->h; y += result->tile_h)
        for (usize x = 0; x < result->w; x += result->tile_w, ++i) {
            f32 prev = (i > 0 ? result->cdf[i - 1] : 0.0f);

            usize tile_min_x = x;
            usize tile_min_y = y;
            usize tile_one_past_max_x = Min(x + result->tile_w, result->w);
            usize tile_one_past_max_y = Min(y + result->tile_h, result->h);

            f32 curr = 0.0f;
            for (usize tile_y = tile_min_y; tile_y < tile_one_past_max_y; ++tile_y)
            for (usize tile_x = tile_min_x; tile_x < tile_one_past_max_x; ++tile_x) {
                V3 pixel = result->pixels[tile_y*result->w + tile_x];
                curr += luma(pixel);
            }

            sum += curr;
            result->cdf[i] = prev + curr;
        }

        f32 rcp_sum = 1.0f / sum;
        for (i = 0; i < cdf_size; ++i) {
            result->cdf[i] *= rcp_sum;
        }
    }

    return result;
}

//
// NOTE: Image writing
//

#pragma pack(push, 1)
struct BitmapHeader {
    u16 file_type;
    u32 file_size;
    u16 reserved1;
    u16 reserved2;
    u32 bitmap_offset;
    u32 size;
    s32 width;
    s32 height;
    u16 planes;
    u16 bits_per_pixel;

    u32 compression;
    u32 size_of_bitmap;
    s32 horz_resolution;
    s32 vert_resolution;
    u32 colors_used;
    u32 colors_important;
};
#pragma pack(pop)

void
write_bitmap(u32* pixels, u32 w, u32 h, const char* file_name, Arena* temp_arena) {
    u32 pixel_size = sizeof(*pixels)*w*h;

    ScopedMemory scoped_memory(temp_arena);

    usize total_size = sizeof(BitmapHeader) + pixel_size;
    void* mem = push_size(temp_arena, total_size, no_clear());

    BitmapHeader* header = (BitmapHeader*)mem;
    header->file_type        = 0x4D42;
    header->file_size        = sizeof(*header) + pixel_size;
    header->bitmap_offset    = sizeof(*header);
    header->size             = sizeof(*header) - 14;
    header->width            = w;
    header->height           = -(s32)h;
    header->planes           = 1;
    header->bits_per_pixel   = 32;
    header->compression      = 0;
    header->size_of_bitmap   = pixel_size;
    header->horz_resolution  = 4096;
    header->vert_resolution  = 4096;
    header->colors_used      = 0;
    header->colors_important = 0;

    u32* dst_pixels = (u32*)(header + 1);
    memcpy(dst_pixels, pixels, pixel_size);

    if (!platform_write_entire_file(file_name, total_size, mem)) {
        fprintf(stderr, "BMP WRITE ERROR: Failed to write output file %s.\n", file_name);
    }
}
