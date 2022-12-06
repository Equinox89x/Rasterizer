#pragma once
#include "Math.h"
#include "vector"

namespace dae
{
	struct Vertex
	{
		Vector3 position{};
		ColorRGB color{colors::White};
		Vector2 uv{};
		Vector3 normal{};
		Vector3 tangent{};
		Vector3 viewDirection{};
	};

	struct Vertex_Out
	{
		Vector4 position{};
		ColorRGB color{ colors::White };
		Vector2 uv{};
		Vector3 normal{};
		Vector3 tangent{};
		Vector3 viewDirection{};
	};

	enum class PrimitiveTopology
	{
		TriangeList,
		TriangleStrip
	};

	struct Mesh
	{
		std::vector<Vertex> vertices{};
		std::vector<uint32_t> indices{};
		PrimitiveTopology primitiveTopology{ PrimitiveTopology::TriangleStrip };

		std::vector<Vertex_Out> vertices_out{};
		Matrix worldMatrix{};

		Vector3 totalTranslation{};
		Matrix rotationTransform{};
		Matrix translationTransform{ {1,1,1},{1,1,1},{1,1,1},{1,1,1} };
		Matrix scaleTransform{ {1,1,1},{1,1,1},{1,1,1},{1,1,1} };
	};

	enum class LightingMode {
		ObservedArea,
		Diffuse,
		Specular,
		Combined
	};
}