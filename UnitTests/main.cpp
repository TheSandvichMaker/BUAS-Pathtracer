#pragma warning(disable: 26812)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "..\MathLib\my_math.h"
using namespace math;

#include "glm\vec2.hpp"
#include "glm\vec3.hpp"
#include "glm\vec4.hpp"
#include "glm\mat4x4.hpp"
#include "glm\gtx\transform.hpp"

#define EPSILON 0.001f

static float
float_equals(float a, float b, float epsilon) {
	return absolute_value(a - b) <= epsilon;
}

static bool
are_equal(V2 v, glm::vec2 glm_v) {
	return ((v.x == glm_v.x) &&
			(v.y == glm_v.y));
}

static bool
are_equal(V3 v, glm::vec3 glm_v) {
	return ((v.x == glm_v.x) &&
			(v.y == glm_v.y) &&
			(v.z == glm_v.z));
}

static bool
are_equal(V4 v, glm::vec4 glm_v) {
	return ((v.x == glm_v.x) &&
			(v.y == glm_v.y) &&
			(v.z == glm_v.z) &&
			(v.w == glm_v.w));
}

static bool
are_equal(V3 a, V3 b) {
	return ((a.x == b.x) &&
			(a.y == b.y) &&
			(a.z == b.z));
}

static bool
are_equal(V4 a, V4 b) {
	return ((a.x == b.x) &&
			(a.y == b.y) &&
			(a.z == b.z) &&
			(a.w == b.w));
}

static bool
are_approximately_equal(V4 a, V4 b) {
	return (float_equals(a.x, b.x, EPSILON) &&
			float_equals(a.y, b.y, EPSILON) &&
			float_equals(a.z, b.z, EPSILON) &&
			float_equals(a.w, b.w, EPSILON));
}

static bool
are_equal(M4x4 m, glm::mat4 glm_m) {
	return ((m.e[0][0] == glm_m[0][0]) &&
			(m.e[0][1] == glm_m[1][0]) &&
			(m.e[0][2] == glm_m[2][0]) &&
			(m.e[0][3] == glm_m[3][0]) &&
	        (m.e[1][0] == glm_m[0][1]) &&
			(m.e[1][1] == glm_m[1][1]) &&
			(m.e[1][2] == glm_m[2][1]) &&
			(m.e[1][3] == glm_m[3][1]) &&
	        (m.e[2][0] == glm_m[0][2]) &&
			(m.e[2][1] == glm_m[1][2]) &&
			(m.e[2][2] == glm_m[2][2]) &&
			(m.e[2][3] == glm_m[3][2]) &&
	        (m.e[3][0] == glm_m[0][3]) &&
			(m.e[3][1] == glm_m[1][3]) &&
			(m.e[3][2] == glm_m[2][3]) &&
			(m.e[3][3] == glm_m[3][3]));
}

static bool
are_approximately_equal(M4x4 m, glm::mat4 glm_m) {
	return (float_equals(m.e[0][0], glm_m[0][0], EPSILON) &&
			float_equals(m.e[0][1], glm_m[1][0], EPSILON) &&
			float_equals(m.e[0][2], glm_m[2][0], EPSILON) &&
			float_equals(m.e[0][3], glm_m[3][0], EPSILON) &&
	        float_equals(m.e[1][0], glm_m[0][1], EPSILON) &&
			float_equals(m.e[1][1], glm_m[1][1], EPSILON) &&
			float_equals(m.e[1][2], glm_m[2][1], EPSILON) &&
			float_equals(m.e[1][3], glm_m[3][1], EPSILON) &&
	        float_equals(m.e[2][0], glm_m[0][2], EPSILON) &&
			float_equals(m.e[2][1], glm_m[1][2], EPSILON) &&
			float_equals(m.e[2][2], glm_m[2][2], EPSILON) &&
			float_equals(m.e[2][3], glm_m[3][2], EPSILON) &&
	        float_equals(m.e[3][0], glm_m[0][3], EPSILON) &&
			float_equals(m.e[3][1], glm_m[1][3], EPSILON) &&
			float_equals(m.e[3][2], glm_m[2][3], EPSILON) &&
			float_equals(m.e[3][3], glm_m[3][3], EPSILON));
}

TEST_CASE("Testing Scalar Functions")
{
	float test_float = 93.2953f;
	CHECK(square_root(test_float) == sqrtf(test_float));
}

TEST_CASE("Testing Vector2 functionality")
{
	SUBCASE("Construction")
	{
		V2 a = {};
		CHECK(a.x == 0);
		CHECK(a.y == 0);

		V2 b = v2(10);
		CHECK(b.x == 10);
		CHECK(b.y == 10);

		V2 c = v2(4, 8);
		CHECK(c.x == 4);
		CHECK(c.y == 8);

		V2 d = v2(INFINITY, INFINITY);
		CHECK(d.x == INFINITY);
		CHECK(d.y == INFINITY);

		V2 e = v2(-INFINITY, -INFINITY);
		CHECK(e.x == -INFINITY);
		CHECK(e.y == -INFINITY);
	}

	SUBCASE("Mathematical operators")
	{
		V2 a = v2(4, -6);
		V2 b = v2(9,  4);
		V2 c;

		glm::vec2 glm_a(4, -6);
		glm::vec2 glm_b(9,  4);
		glm::vec2 glm_c;

		// NOTE: Addition
		c = a + b;
		glm_c = glm_a + glm_b;
		CHECK(are_equal(c, glm_c));

		// NOTE: Subtraction
		c = a - b;
		glm_c = glm_a - glm_b;
		CHECK(are_equal(c, glm_c));

		// NOTE: Negation
		c = -a;
		glm_c = -glm_a;
		CHECK(are_equal(c, glm_c));

		// NOTE: Scalar multiplication
		c = 10.0f*a;
		glm_c = 10.0f*glm_a;
		CHECK(are_equal(c, glm_c));

		// NOTE: Scalar multiplication with a negative
		c = -10.0f*a;
		glm_c = -10.0f*glm_a;
		CHECK(are_equal(c, glm_c));

		// NOTE: Scalar division
		c = a / 10.0f;
		glm_c = glm_a / 10.0f;
		CHECK(are_equal(c, glm_c));

		// NOTE: Hadamard product
		c = a*b;
		glm_c = glm_a*glm_b;
		CHECK(are_equal(c, glm_c));

		// NOTE: Mathmatically questionable vector divide
		c = a / b;
		glm_c = glm_a / glm_b;
		CHECK(are_equal(c, glm_c));

		SUBCASE("Vector products")
		{
			// NOTE: Dot product
			float result = dot(a, b);
			float glm_result = glm::dot(glm_a, glm_b);
			CHECK(result == glm_result);
		}

		SUBCASE("Length operations")
		{
			CHECK(length_sq(a) == glm::dot(glm_a, glm_a));
			CHECK(length_sq(b) == glm::dot(glm_b, glm_b));
			CHECK(length(a) == glm::length(glm_a));
			CHECK(length(b) == glm::length(glm_b));

			CHECK(are_equal(normalize(a), glm::normalize(glm_a)));
			CHECK(are_equal(normalize(b), glm::normalize(glm_b)));

			// NOTE: Because floating point math, and specifically maybe because we normalize by multiplying
			//       by the reciprocal of the length, we can't really test if length == 1.0f, that will be
			//       expected to fail. So for now I'm allowing deviation by FLT_EPSILON either way, but this
			//       is a randomly picked range. I don't know what an acceptable range of error looks like
			//       in this case.
			CHECK(float_equals(length(normalize(a)), 1.0f, 0.001f));
			CHECK(float_equals(length(normalize(b)), 1.0f, 0.001f));

			// NOTE: Edge cases
			c = normalize(v2(0));
			CHECK(c.x != c.x);
			CHECK(c.y != c.y);

			c = normalize(v2(INFINITY));
			CHECK(c.x != c.x);
			CHECK(c.y != c.y);

			c = noz(v2(0));
			CHECK(c.x == 0);
			CHECK(c.y == 0);

			c = noz(v2(INFINITY));
			CHECK(c.x == 0);
			CHECK(c.y == 0);

			c = noz(v2(FLT_MAX));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
		}

		SUBCASE("Other functions")
		{
			const int test_granularity = 16;
			for (int i = 0; i < test_granularity; ++i) {
				float t = (float)i / (float)test_granularity;
                c = lerp(a, b, t);
                glm_c = glm::mix(glm_a, glm_b, t);
                CHECK(are_equal(c, glm_c));
			}

			c = absolute_value(a);
			glm_c = glm::abs(glm_a);
			CHECK(are_equal(c, glm_c));
		}
	}
}

TEST_CASE("Testing Vector3 functionality")
{
	SUBCASE("Construction")
	{
		V3 a = {};
		CHECK(a.x == 0);
		CHECK(a.y == 0);
		CHECK(a.z == 0);

		V3 b = v3(4, 8, 12);
		CHECK(b.x == 4);
		CHECK(b.y == 8);
		CHECK(b.z == 12);

		V3 c = v3(v2(4, 8), 12);
		CHECK(c.x == 4);
		CHECK(c.y == 8);
		CHECK(c.z == 12);

		V3 d = v3(4, v2(8, 12));
		CHECK(d.x == 4);
		CHECK(d.y == 8);
		CHECK(d.z == 12);

		V3 e = v3(8);
		CHECK(e.x == 8);
		CHECK(e.y == 8);
		CHECK(e.z == 8);
	}

	SUBCASE("Mathematical operators")
	{
		V3 a = v3(4, -6, 8);
		V3 b = v3(9,  4, 3);
		V3 c;

		glm::vec3 glm_a(4, -6, 8);
		glm::vec3 glm_b(9,  4, 3);
		glm::vec3 glm_c;

		// NOTE: Addition
		CHECK(are_equal(a + b, glm_a + glm_b));

		// NOTE: Subtraction
		CHECK(are_equal(a - b, glm_a - glm_b));

		// NOTE: Negation
		CHECK(are_equal(-a, -glm_a));

		// NOTE: Scalar multiplication
		CHECK(are_equal(10.0f*a, 10.0f*glm_a));

		// NOTE: Scalar multiplication with a negative
		CHECK(are_equal(-10.0f*a, -10.0f*glm_a));

		// NOTE: Scalar division
		CHECK(are_equal(a / 10.0f, glm_a / 10.0f));

		// NOTE: Hadamard product
		CHECK(are_equal(a*b, glm_a*glm_b));

		// NOTE: Mathmatically questionable vector divide
		CHECK(are_equal(a / b, glm_a / glm_b));

		SUBCASE("Vector products")
		{
			// NOTE: Dot product
			CHECK(dot(a, b) == glm::dot(glm_a, glm_b));

			// NOTE: Cross product
			V3 cross_lhs = v3(1, 0, 0);
			V3 cross_rhs = v3(0, 1, 0);

			V3 cross_result = cross(cross_lhs, cross_rhs);
			CHECK(cross_result.x == 0);
			CHECK(cross_result.y == 0);
			CHECK(cross_result.z == 1);

			CHECK(are_equal(cross(a, b), glm::cross(glm_a, glm_b)));
		}

		SUBCASE("Length operations")
		{
			CHECK(length_sq(a) == glm::dot(glm_a, glm_a));
			CHECK(length_sq(b) == glm::dot(glm_b, glm_b));
			CHECK(length(a) == glm::length(glm_a));
			CHECK(length(b) == glm::length(glm_b));

			CHECK(are_equal(normalize(a), glm::normalize(glm_a)));
			CHECK(are_equal(normalize(b), glm::normalize(glm_b)));

			// NOTE: Because floating point math, and specifically maybe because we normalize by multiplying
			//       by the reciprocal of the length, we can't really test if length == 1.0f, that will be
			//       expected to fail. So for now I'm allowing deviation by FLT_EPSILON either way, but this
			//       is a randomly picked range. I don't know what an acceptable range of error looks like
			//       in this case.
			CHECK(float_equals(length(normalize(a)), 1.0f, 0.001f));
			CHECK(float_equals(length(normalize(b)), 1.0f, 0.001f));

			// NOTE: Edge cases
			c = normalize(v3(0));
			CHECK(c.x != c.x);
			CHECK(c.y != c.y);
			CHECK(c.z != c.z);

			c = normalize(v3(INFINITY));
			CHECK(c.x != c.x);
			CHECK(c.y != c.y);
			CHECK(c.z != c.z);

			c = noz(v3(0));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
			CHECK(c.z == 0);

			c = noz(v3(INFINITY));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
			CHECK(c.z == 0);

			c = noz(v3(FLT_MAX));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
			CHECK(c.z == 0);
		}

		SUBCASE("Other functions")
		{
			const int test_granularity = 16;
			for (int i = 0; i < test_granularity; ++i) {
				float t = (float)i / (float)test_granularity;
                c = lerp(a, b, t);
                glm_c = glm::mix(glm_a, glm_b, t);
                CHECK(are_equal(c, glm_c));
			}

			c = absolute_value(a);
			glm_c = glm::abs(glm_a);
			CHECK(are_equal(c, glm_c));
		}
	}
}

TEST_CASE("Testing Vector4 functionality")
{
	SUBCASE("Construction")
	{
		V4 a = {};
		CHECK(a.x == 0);
		CHECK(a.y == 0);
		CHECK(a.z == 0);
		CHECK(a.w == 0);

		V4 b = v4(4, 8, 12, 16);
		CHECK(b.x == 4);
		CHECK(b.y == 8);
		CHECK(b.z == 12);
		CHECK(b.w == 16);

		V4 c = v4(v3(4, 8, 12), 16);
		CHECK(c.x == 4);
		CHECK(c.y == 8);
		CHECK(c.z == 12);
		CHECK(c.w == 16);

		V4 d = v4(4, v3(8, 12, 16));
		CHECK(d.x == 4);
		CHECK(d.y == 8);
		CHECK(d.z == 12);
		CHECK(d.w == 16);

		V4 e = v4(v2(4, 8), v2(12, 16));
		CHECK(e.x == 4);
		CHECK(e.y == 8);
		CHECK(e.z == 12);
		CHECK(e.w == 16);

		V4 f = v4(4, 8, v2(12, 16));
		CHECK(f.x == 4);
		CHECK(f.y == 8);
		CHECK(f.z == 12);
		CHECK(f.w == 16);

		V4 g = v4(v2(4, 8), 12, 16);
		CHECK(g.x == 4);
		CHECK(g.y == 8);
		CHECK(g.z == 12);
		CHECK(g.w == 16);

		V4 h = v4(2);
		CHECK(h.x == 2);
		CHECK(h.y == 2);
		CHECK(h.z == 2);
		CHECK(h.w == 2);
	}

	SUBCASE("Mathematical operators")
	{
		V4 a = v4(4, -6, 8,  30);
		V4 b = v4(9,  4, 3, -20);
		V4 c;

		glm::vec4 glm_a(4, -6, 8,  30);
		glm::vec4 glm_b(9,  4, 3, -20);
		glm::vec4 glm_c;

		// NOTE: Addition
		CHECK(are_equal(a + b, glm_a + glm_b));

		// NOTE: Subtraction
		CHECK(are_equal(a - b, glm_a - glm_b));

		// NOTE: Negation
		CHECK(are_equal(-a, -glm_a));

		// NOTE: Scalar multiplication
		CHECK(are_equal(10.0f*a, 10.0f*glm_a));

		// NOTE: Scalar multiplication with a negative
		CHECK(are_equal(-10.0f*a, -10.0f*glm_a));

		// NOTE: Scalar division
		CHECK(are_equal(a / 10.0f, glm_a / 10.0f));

		// NOTE: Hadamard product
		CHECK(are_equal(a*b, glm_a*glm_b));

		// NOTE: Mathmatically questionable vector divide
		CHECK(are_equal(a / b, glm_a / glm_b));

		SUBCASE("Vector products")
		{
			// NOTE: Dot product
			CHECK(dot(a, b) == glm::dot(glm_a, glm_b));
		}

		SUBCASE("Length operations")
		{
			CHECK(length_sq(a) == glm::dot(glm_a, glm_a));
			CHECK(length_sq(b) == glm::dot(glm_b, glm_b));
			CHECK(length(a) == glm::length(glm_a));
			CHECK(length(b) == glm::length(glm_b));

			CHECK(are_equal(normalize(a), glm::normalize(glm_a)));
			CHECK(are_equal(normalize(b), glm::normalize(glm_b)));

			// NOTE: Because floating point math, and specifically maybe because we normalize by multiplying
			//       by the reciprocal of the length, we can't really test if length == 1.0f, that will be
			//       expected to fail. So for now I'm allowing deviation by FLT_EPSILON either way, but this
			//       is a randomly picked range. I don't know what an acceptable range of error looks like
			//       in this case.
			CHECK(float_equals(length(normalize(a)), 1.0f, 0.001f));
			CHECK(float_equals(length(normalize(b)), 1.0f, 0.001f));

			// NOTE: Edge cases
			c = normalize(v4(0));
			CHECK(c.x != c.x);
			CHECK(c.y != c.y);
			CHECK(c.z != c.z);
			CHECK(c.w != c.w);

			c = normalize(v4(INFINITY));
			CHECK(c.x != c.x);
			CHECK(c.y != c.y);
			CHECK(c.z != c.z);
			CHECK(c.w != c.w);

			c = noz(v4(0));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
			CHECK(c.z == 0);
			CHECK(c.w == 0);

			c = noz(v4(INFINITY));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
			CHECK(c.z == 0);
			CHECK(c.w == 0);

			c = noz(v4(FLT_MAX));
			CHECK(c.x == 0);
			CHECK(c.y == 0);
			CHECK(c.z == 0);
			CHECK(c.w == 0);
		}

		SUBCASE("Other functions")
		{
			const int test_granularity = 16;
			for (int i = 0; i < test_granularity; ++i) {
				float t = (float)i / (float)test_granularity;
                c = lerp(a, b, t);
                glm_c = glm::mix(glm_a, glm_b, t);
                CHECK(are_equal(c, glm_c));
			}

			c = absolute_value(a);
			glm_c = glm::abs(glm_a);
			CHECK(are_equal(c, glm_c));
		}
	}
}

TEST_CASE("Testing Matrix44 functionality")
{
	SUBCASE("Construction")
	{
		/// Construct a new matrix from explicit values

		M4x4 m;
		m = m4x4(1,  2,  3,  4,
				 5,  6,  7,  8,
				 9,  10, 11, 12,
				 13, 14, 15, 16);
		CHECK(m.e[0][0] == 1);
		CHECK(m.e[0][1] == 2);
		CHECK(m.e[0][2] == 3);
		CHECK(m.e[0][3] == 4);
		CHECK(m.e[1][0] == 5);
		CHECK(m.e[1][1] == 6);
		CHECK(m.e[1][2] == 7);
		CHECK(m.e[1][3] == 8);
		CHECK(m.e[2][0] == 9);
		CHECK(m.e[2][1] == 10);
		CHECK(m.e[2][2] == 11);
		CHECK(m.e[2][3] == 12);
		CHECK(m.e[3][0] == 13);
		CHECK(m.e[3][1] == 14);
		CHECK(m.e[3][2] == 15);
		CHECK(m.e[3][3] == 16);

		/// Construct a new identity matrix
		/// Creates an identity matrix

		m = m4x4_identity();
		glm::mat4 glm_m(1.0f);

		CHECK(are_equal(m, glm_m));

		/// Creates a translation matrix

		m = m4x4_translate(v3(1, 2, 3));
		glm_m = glm::translate(glm::vec3(1, 2, 3));

		CHECK(are_equal(m, glm_m));

		/// Creates a scale matrix

		m = m4x4_scale(v3(1, 2, 3));
		glm_m = glm::scale(glm::vec3(1, 2, 3));

		CHECK(are_equal(m, glm_m));

		/// Creates a rotation matrix around the x axis (angle in radians)

		m = m4x4_rotate_x_axis(0.5f*PI_32);
		glm_m = glm::rotate(0.5f*PI_32, glm::vec3(1, 0, 0));

		CHECK(are_approximately_equal(m, glm_m));

		/// Creates a rotation matrix around the y axis (angle in radians)

		m = m4x4_rotate_y_axis(0.5f*PI_32);
		glm_m = glm::rotate(0.5f*PI_32, glm::vec3(0, 1, 0));

		CHECK(are_approximately_equal(m, glm_m));

		/// Creates a rotation matrix around the z axis (angle in radians)

		m = m4x4_rotate_z_axis(0.5f*PI_32);
		glm_m = glm::rotate(0.5f*PI_32, glm::vec3(0, 0, 1));

		CHECK(are_approximately_equal(m, glm_m));
	}

	SUBCASE("Mathematical operations")
	{
		/// Matrix multiplication
		M4x4 a = m4x4(1,  2,  3,  4,
				      5,  6,  7,  8,
				      9,  10, 11, 12,
				      13, 14, 15, 16);

		M4x4 b = m4x4(4,  3,  2,  1,
				      8,  7,  6,  5,
				      12, 11, 10, 9,
				      16, 15, 14, 13);

		glm::mat4 glm_a(1,  2,  3,  4,
				        5,  6,  7,  8,
				        9,  10, 11, 12,
				        13, 14, 15, 16);

		glm::mat4 glm_b(4,  3,  2,  1,
				        8,  7,  6,  5,
				        12, 11, 10, 9,
				        16, 15, 14, 13);

		// NOTE: Because glm's matrix convention differs from mine, we will transpose them
		glm_a = glm::transpose(glm_a);
		glm_b = glm::transpose(glm_b);

		M4x4 c = a*b;
		glm::mat4 glm_c = glm_a*glm_b;

		CHECK(are_equal(c, glm_c));

		SUBCASE("Inversion")
		{
			/// Transposes this matrix
			M4x4 a_transpose = transposed(a);
			glm::mat4 glm_a_transpose = glm::transpose(glm_a);

			CHECK(are_equal(a_transpose, glm_a_transpose));
		}
	}

	SUBCASE("Transformation")
	{
		V4 vector = v4(1, 0, 0, 0);
		V4 point  = v4(1, 0, 0, 1);
		M4x4 rotation    = m4x4_rotate_y_axis(PI_32);
		M4x4 translation = m4x4_translate(v3(1, 2, 3));

		/// Transform the given vector by this matrix.
		V4 transformed;

		transformed = rotation*vector;
		CHECK(are_approximately_equal(transformed, v4(-1, 0, 0, 0)));

		transformed = rotation*point;
		CHECK(are_approximately_equal(transformed, v4(-1, 0, 0, 1)));

		transformed = translation*vector;
		CHECK(are_equal(transformed, v4(1, 0, 0, 0)));

		transformed = translation*point;
		CHECK(are_equal(transformed, v4(2, 2, 3, 1)));
	}

	/// Retrieve translation part of the matrix
	M4x4 m = m4x4_translate(v3(1, 2, 3));
	CHECK(are_equal(translation(m), v3(1, 2, 3)));
}

static bool
ray_intersect_plane(V3 O, V3 D, V3 plane_N, float plane_d, float* out_t) {
    bool result = false;

	float t = (plane_d - dot(plane_N, O)) / dot(plane_N, D);
	if (t > 0.0f && t < INFINITY) {
		*out_t = t;
        result = true;
    }

    return result;
}

static bool
ray_intersect_sphere(V3 O, V3 D, float r,
					 float* t_near, float* t_far)
{
    bool result = false;

    V3 sphere_rel_o = O;
    float sphere_r_sq = r*r;

    float b = dot(D, sphere_rel_o);
    float c = dot(sphere_rel_o, sphere_rel_o) - sphere_r_sq;

    float discr = (b*b - c);
    if (discr >= 0) {
        float discr_root = square_root(discr);
        *t_near = -b - discr_root;
        *t_far  = -b + discr_root;
        result = true;
    }

    return result;
}

TEST_CASE("Testing Intersection Functions")
{
	SUBCASE("Plane")
	{
		float t;
		bool hit;

		V3 plane_N = v3(0.0f, 0.707106781f, 0.707106781f);
		float plane_d = -5.0f;

		hit = ray_intersect_plane(v3(0, 0, 10), v3(0, 0, -1), plane_N, plane_d, &t);
		CHECK(hit == true);
		CHECK(float_equals(t, 17.0710678f, EPSILON));

		hit = ray_intersect_plane(v3(0, 2, 10), v3(0, 0, -1), plane_N, plane_d, &t);
		CHECK(hit == true);
		CHECK(float_equals(t, 19.0710678f, EPSILON));

		hit = ray_intersect_plane(v3(5, -10, 10), v3(0, 0, -1), plane_N, plane_d, &t);
		CHECK(hit == true);
		CHECK(float_equals(t, 7.07106781f, EPSILON));

		// NOTE: Edge case, ray exactly parallel to plane
		hit = ray_intersect_plane(v3(0, 0, 0), v3(0, 0, -1), v3(0, 1, 0), 0.0f, &t);
		CHECK(hit == false);

		// NOTE: Plane is behind ray
		hit = ray_intersect_plane(v3(0, 0, -1), v3(0, 0, -1), v3(0, 0, 1), 0.0f, &t);
		CHECK(hit == false);
	}

	SUBCASE("Sphere")
	{
		float t_near, t_far;
		bool hit;
		
		hit = ray_intersect_sphere(v3(0, 0, 10), v3(0, 0, -1), 4.0f, &t_near, &t_far);
		CHECK(hit == true);
		CHECK(float_equals(t_near, 6, EPSILON));
		CHECK(float_equals(t_far, 14, EPSILON));

		hit = ray_intersect_sphere(v3(2, 0, 10), v3(0, 0, -1), 4.0f, &t_near, &t_far);
		CHECK(hit == true);
		CHECK(float_equals(t_near, 6.53589838f, EPSILON));
		CHECK(float_equals(t_far, 13.4641016f, EPSILON));

		hit = ray_intersect_sphere(v3(4, 0, 10), v3(0, 0, -1), 4.0f, &t_near, &t_far);
		CHECK(hit == true);
		CHECK(float_equals(t_near, 10, EPSILON));
		CHECK(float_equals(t_far, 10, EPSILON));

		hit = ray_intersect_sphere(v3(6, 0, 10), v3(0, 0, -1), 4.0f, &t_near, &t_far);
		CHECK(hit == false);
	}
}
