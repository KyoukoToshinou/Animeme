#pragma once

/* credits to nanocat for this */

#define Vec3toVec(shitty_vec3) Vector(shitty_vec3.x, shitty_vec3.y, shitty_vec3.z)

static const float pi = 3.1415926535897932384626433832795f;
static const float pi0p5 = pi * 0.5f;
static const float epsilon = 1.401298e-45f;

static void SinCos(const float rad, float &sin, float &cos) // #include <emmintrin.h>, #include <xmmintrin.h>
{
	const __m128 _ps_fopi = _mm_set1_ps(4.0f / pi);

	const __m128 _ps_0p5 = _mm_set1_ps(0.5f);
	const __m128 _ps_1 = _mm_set1_ps(1.0f);

	const __m128 _ps_dp1 = _mm_set1_ps(-0.7851562f);
	const __m128 _ps_dp2 = _mm_set1_ps(-2.4187564849853515625e-4f);
	const __m128 _ps_dp3 = _mm_set1_ps(-3.77489497744594108e-8f);

	const __m128 _ps_sincof_p0 = _mm_set1_ps(2.443315711809948e-5f);
	const __m128 _ps_sincof_p1 = _mm_set1_ps(8.3321608736e-3f);
	const __m128 _ps_sincof_p2 = _mm_set1_ps(-1.6666654611e-1f);
	const __m128 _ps_coscof_p0 = _mm_set1_ps(2.443315711809948e-5f);
	const __m128 _ps_coscof_p1 = _mm_set1_ps(-1.388731625493765e-3f);
	const __m128 _ps_coscof_p2 = _mm_set1_ps(4.166664568298827e-2f);

	const __m128i _pi32_1 = _mm_set1_epi32(1);
	const __m128i _pi32_i1 = _mm_set1_epi32(~1);
	const __m128i _pi32_2 = _mm_set1_epi32(2);
	const __m128i _pi32_4 = _mm_set1_epi32(4);

	const __m128 _mask_sign_raw = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
	const __m128 _mask_sign_inv = _mm_castsi128_ps(_mm_set1_epi32(~0x80000000));

	__m128  mm1, mm2;
	__m128i mmi0, mmi2, mmi4;

	__m128 x, y, z;
	__m128 y1, y2;

	__m128 a = _mm_set1_ps(rad);

	x = _mm_and_ps(a, _mask_sign_inv);
	y = _mm_mul_ps(x, _ps_fopi);

	mmi2 = _mm_cvtps_epi32(y);
	mmi2 = _mm_add_epi32(mmi2, _pi32_1);
	mmi2 = _mm_and_si128(mmi2, _pi32_i1);
	y = _mm_cvtepi32_ps(mmi2);

	mmi4 = mmi2;

	mmi0 = _mm_and_si128(mmi2, _pi32_4);
	mmi0 = _mm_slli_epi32(mmi0, 29);
	__m128 swap_sign_bit_sin = _mm_castsi128_ps(mmi0);

	mmi2 = _mm_and_si128(mmi2, _pi32_2);
	mmi2 = _mm_cmpeq_epi32(mmi2, _mm_setzero_si128());
	__m128 poly_mask = _mm_castsi128_ps(mmi2);

	x = _mm_add_ps(x, _mm_mul_ps(y, _ps_dp1));
	x = _mm_add_ps(x, _mm_mul_ps(y, _ps_dp2));
	x = _mm_add_ps(x, _mm_mul_ps(y, _ps_dp3));

	mmi4 = _mm_sub_epi32(mmi4, _pi32_2);
	mmi4 = _mm_andnot_si128(mmi4, _pi32_4);
	mmi4 = _mm_slli_epi32(mmi4, 29);

	__m128 sign_bit_cos = _mm_castsi128_ps(mmi4);
	__m128 sign_bit_sin = _mm_xor_ps(_mm_and_ps(a, _mask_sign_raw), swap_sign_bit_sin);


	z = _mm_mul_ps(x, x);

	y1 = _mm_mul_ps(_ps_coscof_p0, z);
	y1 = _mm_add_ps(y1, _ps_coscof_p1);
	y1 = _mm_mul_ps(y1, z);
	y1 = _mm_add_ps(y1, _ps_coscof_p2);
	y1 = _mm_mul_ps(y1, z);
	y1 = _mm_mul_ps(y1, z);
	y1 = _mm_sub_ps(y1, _mm_mul_ps(z, _ps_0p5));
	y1 = _mm_add_ps(y1, _ps_1);

	y2 = _mm_mul_ps(_ps_sincof_p0, z);
	y2 = _mm_add_ps(y2, _ps_sincof_p1);
	y2 = _mm_mul_ps(y2, z);
	y2 = _mm_add_ps(y2, _ps_sincof_p2);
	y2 = _mm_mul_ps(y2, z);
	y2 = _mm_mul_ps(y2, x);
	y2 = _mm_add_ps(y2, x);


	__m128 sin1y = _mm_andnot_ps(poly_mask, y1);
	__m128 sin2y = _mm_and_ps(poly_mask, y2);


	mm1 = _mm_add_ps(sin1y, sin2y);
	mm2 = _mm_add_ps(_mm_sub_ps(y1, sin1y), _mm_sub_ps(y2, sin2y));

	sin = _mm_cvtss_f32(_mm_xor_ps(mm1, sign_bit_sin));
	cos = _mm_cvtss_f32(_mm_xor_ps(mm2, sign_bit_cos));
}

static float Atan(float y, float x) // #include <emmintrin.h>, #include <xmmintrin.h>
{
	const __m128 _ps_atan_p0 = _mm_set1_ps(-0.0464964749f);
	const __m128 _ps_atan_p1 = _mm_set1_ps(0.15931422f);
	const __m128 _ps_atan_p2 = _mm_set1_ps(0.327622764f);

	const __m128 _ps_pi = _mm_set1_ps(pi);
	const __m128 _ps_pi0p5 = _mm_set1_ps(pi0p5);

	const __m128 _mask_sign_raw = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
	const __m128 _mask_sign_inv = _mm_castsi128_ps(_mm_set1_epi32(~0x80000000));

	__m128 mm1, mm2, mm3;
	__m128 axm, aym;

	__m128 xm = _mm_set1_ps(x);
	__m128 ym = _mm_set1_ps(y);

	axm = _mm_and_ps(xm, _mask_sign_inv);
	aym = _mm_and_ps(ym, _mask_sign_inv);

	mm1 = _mm_min_ps(axm, aym);
	mm2 = _mm_max_ps(axm, aym);
	mm1 = _mm_div_ps(mm1, mm2);
	mm2 = _mm_mul_ps(mm1, mm1);

	mm3 = _mm_mul_ps(mm2, _ps_atan_p0);
	mm3 = _mm_add_ps(mm3, _ps_atan_p1);
	mm3 = _mm_mul_ps(mm3, mm2);
	mm3 = _mm_sub_ps(mm3, _ps_atan_p2);
	mm3 = _mm_mul_ps(mm3, mm2);
	mm3 = _mm_mul_ps(mm3, mm1);
	mm3 = _mm_add_ps(mm3, mm1);

	__m128 mask;

	/* |y| > |x| */
	mask = _mm_cmpgt_ss(aym, axm);
	mm2 = _mm_and_ps(_ps_pi0p5, mask);
	mm1 = _mm_and_ps(_mask_sign_raw, mask);
	mm3 = _mm_xor_ps(mm3, mm1);
	mm3 = _mm_add_ps(mm3, mm2);

	/* x < 0 */
	mask = _mm_and_ps(xm, _mask_sign_raw);
	mm3 = _mm_xor_ps(mm3, mask);
	mm1 = _mm_castsi128_ps(_mm_srai_epi32(_mm_castps_si128(mm3), 30));
	mm1 = _mm_and_ps(_ps_pi, mm1);
	mm3 = _mm_add_ps(mm3, mm1);

	/* y < 0 */
	mm1 = _mm_and_ps(ym, _mask_sign_raw);
	mm3 = _mm_xor_ps(mm3, mm1);

	return _mm_cvtss_f32(mm3);
}

inline float Rad2Deg(const float &rad)
{
	return rad * (180.f / pi);
}

inline float Deg2Rad(const float &deg)
{
	return deg * (pi / 180.f);
}

inline float Sqrt(const float &sqr) // #include <xmmintrin.h>
{
	__m128 mm1;

	mm1 = _mm_set_ss(sqr);
	mm1 = _mm_sqrt_ss(mm1);

	return _mm_cvtss_f32(mm1);
}

template<typename T>
inline T Min(T x, T y)
{
	return x < y ? x : y;
}

template<typename T>
inline T Max(T x, T y)
{
	return x > y ? x : y;
}

class Vector
{
public:
	Vector();
	Vector(float, float, float);

	float  &operator[](int);
	Vector &operator=(const Vector &);

	Vector &operator+=(const Vector &);
	Vector &operator-=(const Vector &);
	Vector &operator*=(const Vector &);
	Vector &operator*=(const float);
	Vector &operator/=(const Vector &);
	Vector &operator/=(const float);

	Vector operator+(const Vector &) const;
	Vector operator-(const Vector &) const;
	Vector operator/(const Vector &) const;
	Vector operator/(const float) const;
	Vector operator*(const Vector &) const;
	Vector operator*(const float) const;

	bool	operator==(const Vector &) const;
	bool	operator!=(const Vector &) const;

	void	Zero();
	bool	IsZero(const float = epsilon) const;

	Vector	Cross(const Vector &) const;
	Vector	Rotate(const Vector &) const;
	void	RotateInPlace(const Vector &);

	float	Length() const;
	float	Length2D() const;
	float	Dot(const Vector &) const;
	float	DistTo(const Vector &) const;
	float	Normalize();
	void	NormalizeAngle();

	float	x, y, z;
};

class Vector2D
{
public:
	Vector2D();
	Vector2D(float, float);

	float  &operator[](int);
	Vector2D &operator=(const Vector2D &);

	Vector2D &operator+=(const Vector2D &);
	Vector2D &operator-=(const Vector2D &);
	Vector2D &operator*=(const Vector2D &);
	Vector2D &operator*=(const float);
	Vector2D &operator/=(const Vector2D &);
	Vector2D &operator/=(const float);

	Vector2D operator+(const Vector2D &) const;
	Vector2D operator-(const Vector2D &) const;
	Vector2D operator/(const Vector2D &) const;
	Vector2D operator/(const float) const;
	Vector2D operator*(const Vector2D &) const;
	Vector2D operator*(const float) const;

	bool	operator==(const Vector2D &) const;
	bool	operator!=(const Vector2D &) const;

	float	x, y;
};

inline Vector2D::Vector2D()
{
	x = y = 0.0f;
}

inline Vector2D::Vector2D(float a, float b)
{
	x = a;
	y = b;
}

inline Vector2D& Vector2D::operator=(const Vector2D &v)
{
	x = v.x;
	y = v.y;

	return *this;
}

inline Vector2D& Vector2D::operator+=(const Vector2D &v)
{
	x += v.x;
	y += v.y;

	return *this;
}

inline Vector2D& Vector2D::operator-=(const Vector2D &v)
{
	x -= v.x;
	y -= v.y;

	return *this;
}

inline Vector2D& Vector2D::operator*=(const Vector2D &v)
{
	x *= v.x;
	y *= v.y;

	return *this;
}

inline Vector2D& Vector2D::operator*=(const float f)
{
	x *= f;
	y *= f;

	return *this;
}

inline Vector2D& Vector2D::operator/=(const Vector2D &v)
{
	x /= v.x + epsilon;
	y /= v.y + epsilon;

	return *this;
}

inline Vector2D& Vector2D::operator/=(const float f)
{
	x /= f + epsilon;
	y /= f + epsilon;

	return *this;
}

inline Vector2D Vector2D::operator+(const Vector2D &v) const
{
	return Vector2D(x + v.x, y + v.y);
}

inline Vector2D Vector2D::operator-(const Vector2D &v) const
{
	return Vector2D(x - v.x, y - v.y);
}

inline Vector2D Vector2D::operator/(const Vector2D &v) const
{
	return Vector2D(x / (v.x + epsilon), y / (v.y + epsilon));
}

inline Vector2D Vector2D::operator/(const float f) const
{
	return Vector2D(x / (f + epsilon), y / (f + epsilon));
}

inline Vector2D Vector2D::operator*(const Vector2D &v) const
{
	return Vector2D(x * v.x, y * v.y);
}

inline Vector2D Vector2D::operator*(const float f) const
{
	return Vector2D(x * f, y * f);
}

inline bool Vector2D::operator==(const Vector2D &v) const
{
	return
		v.x == x &&
		v.y == y;
}

inline bool Vector2D::operator!=(const Vector2D &v) const
{
	return
		v.x != x ||
		v.y != y;
}

inline float& Vector2D::operator[](int index)
{
	return ((float *)this)[index];
}

class VectorAligned : public Vector
{
public:
	float w;
};

class matrix4x4
{
public:
	inline float *operator[](int i)
	{
		return m[i];
	}

	inline const float *operator[](int i) const
	{
		return m[i];
	}

	float m[4][4];
};

class matrix3x4
{
public:
	inline float *operator[](int i)
	{
		return m[i];
	}

	inline const float *operator[](int i) const
	{
		return m[i];
	}

	float m[3][4];
};

inline void Cross(const Vector &a, const Vector &b, Vector &x)
{
	x.x = ((a.y * b.z) - (a.z * b.y));
	x.y = ((a.z * b.x) - (a.x * b.z));
	x.z = ((a.x * b.y) - (a.y * b.x));
}

inline void AngleMatrix(const Vector&angles, matrix3x4 &matrix)
{
	float sp, sy, sr, cp, cy, cr;

	SinCos(Deg2Rad(angles.x), sp, cp);
	SinCos(Deg2Rad(angles.y), sy, cy);
	SinCos(Deg2Rad(angles.z), sr, cr);

	matrix[0][0] = cp * cy;
	matrix[1][0] = cp * sy;
	matrix[2][0] = -sp;

	float crcy = cr * cy;
	float crsy = cr * sy;
	float srcy = sr * cy;
	float srsy = sr * sy;

	matrix[0][1] = sp * srcy - crsy;
	matrix[1][1] = sp * srsy + crcy;
	matrix[2][1] = sr * cp;

	matrix[0][2] = sp * crcy + srsy;
	matrix[1][2] = sp * crsy - srcy;
	matrix[2][2] = cr * cp;

	matrix[0][3] = 0.0f;
	matrix[1][3] = 0.0f;
	matrix[2][3] = 0.0f;
}

inline void AngleVectors(const Vector&angles, Vector &forward)
{
	float sp, sy, cp, cy;

	SinCos(Deg2Rad(angles.x), sp, cp);
	SinCos(Deg2Rad(angles.y), sy, cy);

	forward.x = cp * cy;
	forward.y = cp * sy;
	forward.z = -sp;
	forward.Normalize();
}

inline void AngleVectors(const Vector&angles, Vector &forward, Vector &right, Vector &up)
{
	float sp, sy, sr, cp, cy, cr;

	SinCos(Deg2Rad(angles.x), sp, cp);
	SinCos(Deg2Rad(angles.y), sy, cy);
	SinCos(Deg2Rad(angles.z), sr, cr);


	forward.x = cp * cy;
	forward.y = cp * sy;
	forward.z = -sp;
	forward.Normalize();

	right.x = -(sr * sp * cy) + (cr * sy);
	right.y = -(sr * sp * sy) + -(cr * cy);
	right.z = -(sr * cp);
	right.Normalize();

	up.x = cr * sp * cy + -sr * -sy;
	up.y = cr * sp * sy + -sr * cy;
	up.z = cr * cp;
	up.Normalize();
}

inline void VectorRotate(const Vector &i, const matrix3x4 &matrix, Vector &o)
{
	o.x = i.Dot(Vector(matrix[0][0], matrix[0][1], matrix[0][2]));
	o.y = i.Dot(Vector(matrix[1][0], matrix[1][1], matrix[1][2]));
	o.z = i.Dot(Vector(matrix[2][0], matrix[2][1], matrix[2][2]));
}

inline void VectorRotate(const Vector &i, const Vector&angles, Vector &o)
{
	matrix3x4 matrix;

	AngleMatrix(angles, matrix);
	VectorRotate(i, matrix, o);
}

inline void VectorAngles(const Vector &vec, Vector&angles)
{
	if (vec.x == 0.0f && vec.y == 0.0f)
	{
		if (vec.z > 0.0f)
		{
			angles.x = -90.0f;
		}
		else
		{
			angles.x = 90.0f;
		}
	}
	else
	{
		angles.x = Rad2Deg(Atan(-vec.z, vec.Length2D()));
		angles.y = Rad2Deg(Atan(vec.y, vec.x));
	}
}

inline void VectorAngles(const Vector &vec, const Vector &up, Vector&angles)
{
	Vector left;
	Cross(up, vec, left);

	left.Normalize();

	float len = vec.Length2D();
	angles.x = Rad2Deg(Atan(-vec.z, len));

	if (len > 0.001f)
	{
		angles.y = Rad2Deg(Atan(vec.y, vec.x));
		angles.z = Rad2Deg(Atan(left.z, ((left.y * vec.x) - (left.x * vec.y))));
	}
	else
	{
		angles.y = Rad2Deg(Atan(-left.x, left.y));
		angles.z = 0.0f;
	}

	//NormalizeAngles(angles);
}

inline void VectorTransform(const Vector &v, const matrix3x4 &matrix, Vector &o)
{
	o.x = v.Dot(Vector(matrix[0][0], matrix[0][1], matrix[0][2])) + matrix[0][3];
	o.y = v.Dot(Vector(matrix[1][0], matrix[1][1], matrix[1][2])) + matrix[1][3];
	o.z = v.Dot(Vector(matrix[2][0], matrix[2][1], matrix[2][2])) + matrix[2][3];
}

inline void MatrixGetColumn(const matrix3x4 &matrix, int i, Vector &o)
{
	o.x = matrix[0][i];
	o.y = matrix[1][i];
	o.z = matrix[2][i];
}

inline void MatrixPosition(const matrix3x4 &matrix, Vector &vec)
{
	MatrixGetColumn(matrix, 3, vec);
}

inline void MatrixAngles(const matrix3x4 &matrix, Vector&angles)
{
	Vector forward, left, up;

	forward[0] = matrix[0][0];
	forward[1] = matrix[1][0];
	forward[2] = matrix[2][0];

	left[0] = matrix[0][1];
	left[1] = matrix[1][1];
	left[2] = matrix[2][1];

	up[2] = matrix[2][2];

	float len2d = forward.Length2D();
	if (len2d > 0.001f)
	{
		angles.x = Rad2Deg(Atan(-forward.z, len2d));
		angles.y = Rad2Deg(Atan(forward.y, forward.x));
		angles.z = Rad2Deg(Atan(left.z, up.z));
	}
	else
	{
		angles.x = Rad2Deg(Atan(-forward.z, len2d));
		angles.y = Rad2Deg(Atan(-left.x, left.y));
		angles.z = 0.f;
	}
}

inline void MatrixAngles(const matrix3x4 &matrix, Vector&angles, Vector &vec)
{
	MatrixAngles(matrix, angles);
	MatrixPosition(matrix, vec);
}

inline Vector::Vector()
{
	x = y = z = 0.0f;
}

inline Vector::Vector(float a, float b, float c)
{
	x = a;
	y = b;
	z = c;
}

inline void Vector::RotateInPlace(const Vector&angles)
{
	VectorRotate(*this, angles, *this);
}

inline Vector Vector::Rotate(const Vector&angles) const
{
	Vector o;
	VectorRotate(*this, angles, o);

	return o;
}

inline Vector& Vector::operator=(const Vector &v)
{
	x = v.x;
	y = v.y;
	z = v.z;

	return *this;
}

inline Vector& Vector::operator+=(const Vector &v)
{
	x += v.x;
	y += v.y;
	z += v.z;

	return *this;
}

inline Vector& Vector::operator-=(const Vector &v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;

	return *this;
}

inline Vector& Vector::operator*=(const Vector &v)
{
	x *= v.x;
	y *= v.y;
	z *= v.z;

	return *this;
}

inline Vector& Vector::operator*=(const float f)
{
	x *= f;
	y *= f;
	z *= f;

	return *this;
}

inline Vector& Vector::operator/=(const Vector &v)
{
	x /= v.x + epsilon;
	y /= v.y + epsilon;
	z /= v.z + epsilon;

	return *this;
}

inline Vector& Vector::operator/=(const float f)
{
	x /= f + epsilon;
	y /= f + epsilon;
	z /= f + epsilon;

	return *this;
}

inline Vector Vector::operator+(const Vector &v) const
{
	return Vector(x + v.x, y + v.y, z + v.z);
}

inline Vector Vector::operator-(const Vector &v) const
{
	return Vector(x - v.x, y - v.y, z - v.z);
}

inline Vector Vector::operator/(const Vector &v) const
{
	return Vector(x / (v.x + epsilon), y / (v.y + epsilon), z / (v.z + epsilon));
}

inline Vector Vector::operator/(const float f) const
{
	return Vector(x / (f + epsilon), y / (f + epsilon), z / (f + epsilon));
}

inline Vector Vector::operator*(const Vector &v) const
{
	return Vector(x * v.x, y * v.y, z * v.z);
}

inline Vector Vector::operator*(const float f) const
{
	return Vector(x * f, y * f, z * f);
}

inline bool Vector::operator==(const Vector &v) const
{
	return
		v.x == x &&
		v.y == y &&
		v.z == z;
}

inline bool Vector::operator!=(const Vector &v) const
{
	return
		v.x != x ||
		v.y != y ||
		v.z != z;
}

inline float& Vector::operator[](int index)
{
	return ((float *)this)[index];
}

inline void Vector::Zero()
{
	x = y = z = 0.0f;
}

inline bool Vector::IsZero(const float f) const
{
	return (x > -f && x < f && y > -f && y < f && z > -f && z < f);
}

inline float Vector::Length() const
{
	return Sqrt((x * x) + (y * y) + (z * z));
}

inline float Vector::Length2D() const
{
	return Sqrt((x * x) + (y * y));
}

inline float Vector::DistTo(const Vector &v) const
{
	return (*this - v).Length();
}

inline float Vector::Normalize()
{
	float l = Length();
	float m = 1.0f / (l + epsilon);

	x *= m;
	y *= m;
	z *= m;

	return l;
}

inline float Vector::Dot(const Vector &v) const
{
	return x * v.x + y * v.y + z * v.z;
}

inline Vector Vector::Cross(const Vector &v) const
{
	return Vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

inline void Vector::NormalizeAngle()
{
	while (x > 180.f)  x -= 360.f;
	while (x < -180.f) x += 360.f;

	while (y > 180.f)  y -= 360.f;
	while (y < -180.f) y += 360.f;

	while (z > 180.f)  z -= 360.f;
	while (z < -180.f) z += 360.f;
}


inline float DotProduct(const Vector &v1, const Vector &v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline Vector CrossProduct(const Vector &v1, const Vector &v2)
{
	return Vector(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}