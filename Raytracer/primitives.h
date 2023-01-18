#pragma once

enum PrimitiveType : u16 {
    Primitive_None,
    Primitive_Plane,
    Primitive_Sphere,
    Primitive_Box,
    Primitive_Mesh,
    Primitive_CSGNode,
};

inline const char*
to_string(PrimitiveType type) {
    switch (type) {
        case Primitive_None   : { return "Primitive_None";    } break;
        case Primitive_Plane  : { return "Primitive_Plane";   } break;
        case Primitive_Sphere : { return "Primitive_Sphere";  } break;
        case Primitive_Box    : { return "Primitive_Box";     } break;
        case Primitive_Mesh   : { return "Primitive_Mesh";    } break;
        case Primitive_CSGNode: { return "Primitive_CSGNode"; } break;
    }
    return "Unknown PrimitiveType";
}

struct Plane {
    V3  n;
    f32 d;
};

struct Sphere {
    f32 r;
};

struct Box {
    V3 r;
};

struct Triangle {
    union {
		struct {
			V3 a, b, c;
		};
		V3 v[3];
	};
};

struct Triangle_4x {
    union {
        struct {
            V3_4x a;
            V3_4x b;
            V3_4x c;
        };
        V3_4x v[3];
    };
};

struct Mesh {
    b16 has_texture_coordinates;
    b16 has_normals;

    u32 triangle_count;
    Triangle* triangles; // NOTE: Triangles are packed so that first are the regular triangles,
                         //       then texture coordinates if you've got any, and then normals.

    struct MeshBVH* bvh;
};

force_inline Triangle*
get_texture_coordinates(Mesh* mesh) {
    assert_slow(mesh->has_texture_coordinates);
    return mesh->triangles + mesh->triangle_count;
}

force_inline Triangle*
get_normals(Mesh* mesh) {
    assert_slow(mesh->has_normals);
    return mesh->triangles + (1ull + mesh->has_texture_coordinates)*mesh->triangle_count;
}

enum CSGOperator {
    CSG_Union,
    CSG_Difference,
};

struct PrimitiveCSGNode {
    CSGOperator op;
    PrimitiveID left;
    PrimitiveID right;
};

struct Primitive {
    // NOTE: MaterialID and PrimitiveType are 16 bit, so now I am introducing some padding with this pointer
    M4x4Inv* transform;

    MaterialID material_id;
    PrimitiveType type;

    union {
        Plane plane;
        Sphere sphere;
        Box box;
        Mesh mesh;
        PrimitiveCSGNode node;
    };
};
