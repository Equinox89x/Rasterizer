#pragma once
#include "Math.h"
#include "DataTypes.h"
#include "BRDF.h"

namespace dae
{
#pragma region Material BASE
	class Material
	{
	public:
		Material() = default;
		virtual ~Material() = default;

		Material(const Material&) = delete;
		Material(Material&&) noexcept = delete;
		Material& operator=(const Material&) = delete;
		Material& operator=(Material&&) noexcept = delete;

		/**
		 * \brief Function used to calculate the correct color for the specific material and its parameters
		 * \param hitrecord current
		 * \param l light direction
		 * \param v view direction
		 * \return color
		 */
		virtual ColorRGB Shade(const Vertex_Out& vert = {}, const Vector3& l = {}, const Vector3& v = {}) = 0;
	};
#pragma endregion

#pragma region Material SOLID COLOR
	//SOLID COLOR
	//===========
	class Material_SolidColor final : public Material
	{
	public:
		Material_SolidColor(const ColorRGB& color) : m_Color(color)
		{
		}

		ColorRGB Shade(const Vertex_Out& vert, const Vector3& l, const Vector3& v) override
		{
			return m_Color;
		}

	private:
		ColorRGB m_Color{ colors::White };
	};
#pragma endregion

#pragma region Material LAMBERT
	//LAMBERT
	//=======
	class Material_Lambert final : public Material
	{
	public:
		Material_Lambert(const ColorRGB& diffuseColor, float diffuseReflectance) :
			m_DiffuseColor(diffuseColor), m_DiffuseReflectance(diffuseReflectance) {}

		ColorRGB Shade(const Vertex_Out& vert = {}, const Vector3& l = {}, const Vector3& v = {}) override
		{
			return BRDF::Lambert(m_DiffuseReflectance, m_DiffuseColor);

		}

	private:
		ColorRGB m_DiffuseColor{ colors::White };
		float m_DiffuseReflectance{ 1.f }; //kd
	};
#pragma endregion

#pragma region Material LAMBERT PHONG
	//LAMBERT-PHONG
	//=============
	class Material_LambertPhong final : public Material
	{
	public:
		Material_LambertPhong(const ColorRGB& diffuseColor, float diffuseReflectance, ColorRGB specularReflectance, float phongExponent) :
			m_DiffuseColor(diffuseColor), m_DiffuseReflectance(diffuseReflectance), m_SpecularReflectance(specularReflectance),
			m_PhongExponent(phongExponent)
		{
		}

		ColorRGB Shade(const Vertex_Out& vert = {}, const Vector3& l = {}, const Vector3& v = {}) override
		{
			return BRDF::Lambert(m_DiffuseReflectance, m_DiffuseColor) + BRDF::Phong(m_SpecularReflectance, m_PhongExponent, l, -v, vert.normal);
		}

	private:
		ColorRGB m_DiffuseColor{ colors::White };
		float m_DiffuseReflectance{ 0.5f }; //kd
		ColorRGB m_SpecularReflectance{  }; //ks
		float m_PhongExponent{ 1.f }; //Phong Exponent
	};
#pragma endregion

#pragma region Material COOK TORRENCE
	//COOK TORRENCE
	class Material_CookTorrence final : public Material
	{
	public:
		Material_CookTorrence(const ColorRGB& albedo, float metalness, float roughness) :
			m_Albedo(albedo), m_Metalness(metalness), m_Roughness(roughness)
		{
		}

		ColorRGB Shade(const Vertex_Out& vert = {}, const Vector3& l = {}, const Vector3& v = {}) override
		{
			float a{ Square(m_Roughness) };
			ColorRGB f0; //= (m_Metalness == 0) ? (0.04f, 0.04f, 0.04f) : m_Albedo;
			if (m_Metalness == 0)
			{
				f0 = { 0.04f,0.04f,0.04f };
			}
			else
			{
				f0 = m_Albedo;
			}
			Vector3 halfVector = (v + -l) / (v + -l).Magnitude();

			ColorRGB F = BRDF::FresnelFunction_Schlick(halfVector.Normalized(), v.Normalized(), f0);
			float D = BRDF::NormalDistribution_GGX(-vert.normal, halfVector.Normalized(), a);
			float GS = BRDF::GeometryFunction_Smith(-vert.normal, v, -l, a);

			ColorRGB spec = ColorRGB(D * F * GS) / (4.f * Vector3::Dot(v, vert.normal) * Vector3::Dot(-l, vert.normal));

			ColorRGB kd;
			if (m_Metalness)
			{
				kd = ColorRGB(0, 0, 0);
			}
			else
			{
				kd = ColorRGB(1, 1, 1) - F;
			}

			ColorRGB diffuse = BRDF::Lambert(kd, m_Albedo);


			return spec + diffuse;
		}


	private:
		ColorRGB m_Albedo{ 0.955f, 0.637f, 0.538f }; //Copper
		float m_Metalness{ 1.0f };
		float m_Roughness{ 0.1f }; // [1.0 > 0.0] >> [ROUGH > SMOOTH]
	};
#pragma endregion
}
