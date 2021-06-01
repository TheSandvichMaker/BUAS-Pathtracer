#include "precomp.h"
#include "common.h"
#include "assets.h"
#include "primitives.h"
#include "bvh.h"
#include "samplers.h"
#include "scene.h"

MaterialID
add_material(Scene* scene, Material source_material) {
    MaterialID result = MaterialID::from(scene->materials.count);

    Material* material = scene->materials.add();
    *material = source_material;

    if (dot(material->emission_color, v3(1.0f)) > 0.0f) {
        material->flags |= Material_Emissive;
    }

    return result;
}

MaterialID
add_diffuse_material(Scene* scene, V3 diffuse_color, f32 ior, f32 roughness, b32 checkers, V3 checker_color) {
    MaterialID result = MaterialID::from(scene->materials.count);
    Material* material = scene->materials.add();

    if (checkers) {
        material->flags |= Material_Checkers;
    }
    material->checker_color = checker_color;
    material->albedo = diffuse_color;
    material->ior = ior;
    material->roughness = roughness;

    return result;
}

MaterialID
add_translucent_material(Scene* scene, V3 absorb, f32 ior, f32 roughness) {
    MaterialID result = MaterialID::from(scene->materials.count);
    Material* material = scene->materials.add();

    material->is_participating_medium = true;
    material->medium.absorb = absorb;
    material->ior = ior;
    material->roughness = roughness;

    return result;
}

MaterialID
add_emissive_material(Scene* scene, V3 emission_color) {
    MaterialID result = MaterialID::from(scene->materials.count);
    Material* material = scene->materials.add();

    material->flags |= Material_Emissive;
    material->emission_color = emission_color;

    return result;
}

M4x4Inv*
push_transform(Scene* scene, M4x4Inv transform) {
    M4x4Inv* result = push_struct(&scene->arena, M4x4Inv, no_clear());
    *result = transform;
    return result;
}

AddPrimitiveResult
add_primitive(Scene* scene,
              PrimitiveType type,
              MaterialID material_id,
              M4x4Inv* transform)
{
    static M4x4Inv identity_transform = transform_identity();
    StretchyBuffer<Primitive>& primitive_buffer = (type == Primitive_Plane ? scene->planes
                                                                           : scene->primitives);

    PrimitiveID primitive_id = PrimitiveID::from(primitive_buffer.count);

    Primitive* primitive = primitive_buffer.add();
    primitive->type = type;
    primitive->material_id = material_id;

    if (transform) {
        primitive->transform = transform;
    } else {
        primitive->transform = &identity_transform;
    }

    // NOTE: If the material is emissive, it's a light, so get it in the lights array.
    Material* material = &scene->materials[material_id];
    if (material->flags & Material_Emissive) {
        scene->lights.add(primitive_id);
    }

    AddPrimitiveResult result;
    result.id = primitive_id;
    result.primitive = primitive;

    return result;
}

PrimitiveID
add_plane(Scene* scene, MaterialID material_id, V3 n, f32 d) {
    AddPrimitiveResult add = add_primitive(scene, Primitive_Plane, material_id);

    Plane* result = &add.primitive->plane;
    result->n = noz(n);
    result->d = d;

    return add.id;
}

PrimitiveID
add_sphere(Scene* scene, MaterialID material_id, f32 r, M4x4Inv* transform) {
    AddPrimitiveResult add = add_primitive(scene, Primitive_Sphere, material_id, transform);

    Sphere* result = &add.primitive->sphere;
    result->r = r;

    return add.id;
}

PrimitiveID
add_sphere(Scene* scene, MaterialID material_id, f32 r, M4x4Inv transform) {
    return add_sphere(scene, material_id, r, push_transform(scene, transform));
}

PrimitiveID
add_box(Scene* scene, MaterialID material_id, V3 r, M4x4Inv* transform) {
    AddPrimitiveResult add = add_primitive(scene, Primitive_Box, material_id, transform);

    Box* result = &add.primitive->box;
    result->r = r;

    return add.id;
}

PrimitiveID
add_box(Scene* scene, MaterialID material_id, V3 r, M4x4Inv transform) {
    return add_box(scene, material_id, r, push_transform(scene, transform));
}

PrimitiveID
add_mesh(Scene* scene, MaterialID material_id, Mesh* mesh, M4x4Inv* transform) {
	AddPrimitiveResult add = add_primitive(scene, Primitive_Mesh, material_id, transform);

	Mesh* result = &add.primitive->mesh;
	*result = *mesh;

    return add.id;
}

PrimitiveID
add_mesh(Scene* scene, MaterialID material_id, Mesh* mesh, M4x4Inv transform) {
    return add_mesh(scene, material_id, mesh, push_transform(scene, transform));
}

PrimitiveID
add_test_difference(Scene* scene, MaterialID positive_material, MaterialID negative_material, M4x4Inv* transform) {
    AddPrimitiveResult add = add_primitive(scene, Primitive_CSGNode, MaterialID::from(0), transform);

    PrimitiveCSGNode* node = &add.primitive->node;
    node->op = CSG_Difference;
    node->left  = add_sphere(scene, positive_material, 5.0f);
    node->right = add_sphere(scene, negative_material, 3.0f, transform_translate(v3(-5.0f, 0.0f, 0.0f)));

    return add.id;
}

void
create_scene_bvh(Scene* scene, Arena* temp_arena) {
    ScopedMemory scoped_memory(temp_arena);

    u32 max_input_data_count = scene->primitives.count - 1;
    BVHSortEntry* input_data = push_array(temp_arena, max_input_data_count, BVHSortEntry);

    u32 input_data_count = 0;
    for (usize primitive_index = 1; primitive_index < scene->primitives.count; ++primitive_index) {
        Primitive* primitive = &scene->primitives[primitive_index];

        V3 bv_min = {};
        V3 bv_max = {};
        switch (primitive->type) {
            case Primitive_Plane: {
                fprintf(stderr, "WARNING: Plane ended up in the general primitives array somehow.\n");
                fprintf(stderr, "         Primitive index: %zu\n", primitive_index);
                continue;
            } break;

            case Primitive_Sphere: {
                Sphere* sphere = &primitive->sphere;
                bv_min = v3(-sphere->r);
                bv_max = v3( sphere->r);
            } break;

            case Primitive_Box: {
                Box* box = &primitive->box;
                bv_min = -box->r;
                bv_max =  box->r;
            } break;

            case Primitive_Mesh: {
                Mesh* mesh = &primitive->mesh;
                if (!mesh->bvh) {
                    mesh->bvh = create_bvh_for_mesh(mesh->triangle_count, mesh->triangles, &scene->arena, temp_arena, BVH_SAHBinned, BVHStorage_Scalar);
                }
                // NOTE: Intellisense shut-upping null check.
                if (mesh->bvh) {
                    BVHNode* root_node = &mesh->bvh->nodes[0];
                    bv_min = root_node->bv_p - root_node->bv_r;
                    bv_max = root_node->bv_p + root_node->bv_r;
                } else {
                    INVALID_CODE_PATH;
                }
            } break;
        }

        BVHSortEntry* entry = &input_data[input_data_count++];
        entry->index = (u32)primitive_index;

        M4x4 mat = primitive->transform->forward;
        AABB bounds = inverted_infinity_aabb();
        bounds = grow_to_contain(bounds, mat*v3(bv_min.x, bv_min.y, bv_min.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_max.x, bv_min.y, bv_min.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_min.x, bv_max.y, bv_min.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_min.x, bv_min.y, bv_max.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_max.x, bv_max.y, bv_min.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_max.x, bv_min.y, bv_max.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_min.x, bv_max.y, bv_max.z));
        bounds = grow_to_contain(bounds, mat*v3(bv_max.x, bv_max.y, bv_max.z));

        entry->bv_p = 0.5f*(bounds.min + bounds.max);
        entry->bv_r = 0.5f*(bounds.max - bounds.min);

        int fuck_visual_studio_all_my_homies_hate_visual_studio = 1;
    }

    scene->bvh = create_bvh(input_data_count, input_data, &scene->arena, BVH_SAHBinned);
}

void
clear_scene(Scene* scene) {
    scene->materials.free();
    scene->lights.free();
    scene->planes.free();
    scene->primitives.free();
    scene->arena.clear();
    Arena temp = scene->arena;
    zero_struct(scene);
    scene->arena = temp;
}
