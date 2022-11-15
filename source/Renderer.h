#pragma once

#include <cstdint>
#include <vector>

#include "Camera.h"
#include "DataTypes.h"

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Texture;
	struct Mesh;
	struct Vertex;
	class Timer;
	class Scene;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer();

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Update(Timer* pTimer);
		void Render();

		void RenderTriangleList();
		void HandleRender(const Vector2& triangleV0, const Vector2& triangleV1, const Vector2& triangleV2, const Vector2& a, const Vector2& b, const Vector2& c, std::vector<Vertex>& verts, ColorRGB& finalColor);
		void RenderMesh();

		void RenderTriangleStrip(const Mesh& mesh);
		void RenderMeshTriangleList(const Mesh& mesh);

		bool SaveBufferToImage() const;



	private:
		SDL_Window* m_pWindow{};

		SDL_Surface* m_pFrontBuffer{ nullptr };
		SDL_Surface* m_pBackBuffer{ nullptr };
		uint32_t* m_pBackBufferPixels{};

		float* m_pDepthBufferPixels{};
		ColorRGB* m_pColorBuffer{};

		Camera m_Camera{};

		int m_Width{};
		int m_Height{};

		#pragma region W1
		std::vector<Vertex> m_TriangleNDC{
			{ {0.f, 4.f, 2.f}, colors::Red },
			{ {3.f, -2.f, 2.f}, colors::Green },
			{ {-3.f, -2.f, 2.f}, colors::Blue}
		};
		
		std::vector<Vertex> m_TriangleDepth{
			{ {0.f, 2.f, 0.f}, colors::White },
			{ {1.f, 0.f, 0.f}, colors::White },
			{ {-1.f, 0.f, 0.f}, colors::White}
		};
		std::vector<std::vector<Vertex>> m_Triangles{ m_TriangleNDC, m_TriangleDepth };
		#pragma endregion
		
		#pragma region W2
		#pragma region using list of list of vertex
		////top left - right
		//std::vector<Vertex> m_Triangle1{
		//	{ {-3, 3, -2}, colors::Red }, //v0
		//	{ {0, 3, -2}, colors::Green }, //v1
		//	{ {0, 0, -2}, colors::Blue} //v4
		//};
		////top left - left
		//std::vector<Vertex> m_Triangle2{
		//	{ {-3, 3, -2}, colors::Red }, //v0
		//	{ {0, 0, -2}, colors::Green }, //v4
		//	{ {-3, 0, -2}, colors::Blue} //v3
		//};
		//
		////bottom left - right
		//std::vector<Vertex> m_Triangle3{
		//	{ {-3, 0, -2}, colors::Red }, //v3
		//	{ {0, 0, -2}, colors::Green }, //v4
		//	{ {0, -3, -2}, colors::Blue} //v7
		//};
		////botom left - left
		//std::vector<Vertex> m_Triangle4{
		//	{ {-3, 0, -2}, colors::Red }, //v3
		//	{ {-3, -3, -2}, colors::Green }, //v6
		//	{ {0, -3, -2}, colors::Blue} //v7
		//};
		//
		////bottom right - right
		//std::vector<Vertex> m_Triangle5{
		//	{ {0, 0, -2}, colors::Red }, //v4
		//	{ {3, 0, -2}, colors::Green }, //v5
		//	{ {3, -3, -2}, colors::Blue} //v8
		//};
		////bottom right - left
		//std::vector<Vertex> m_Triangle6{
		//	{ {0, 0, -2}, colors::Red }, //v4
		//	{ {0, -3, -2}, colors::Green }, //v7
		//	{ {3, -3, -2}, colors::Blue} //v8
		//};
		//
		////top right - right
		//std::vector<Vertex> m_Triangle7{
		//	{ {0, 3, -2}, colors::Red }, //v1
		//	{ {3, 3, -2}, colors::Green }, //v2
		//	{ {3, 0, -2}, colors::Blue} //v5
		//};
		////top right - left
		//std::vector<Vertex> m_Triangle8{
		//	{ {0, 3, -2}, colors::Red }, //v1
		//	{ {0, 0, -2}, colors::Green }, //v4
		//	{ {3, 0, -2}, colors::Blue} //v5
		//};
		//std::vector<std::vector<Vertex>> m_Triangles{ m_Triangle1, m_Triangle2, m_Triangle3, m_Triangle4, m_Triangle5, m_Triangle6, m_Triangle7, m_Triangle8 };
		#pragma endregion

		#pragma region Using meshes
		std::vector<Mesh> m_Meshes{
			Mesh{
				{
					Vertex{{-3,3,-2}},
					Vertex{{0,3,-2}},
					Vertex{{3,3,-2}},
					Vertex{{ -3,0,-2}},
					Vertex{{0,0,-2}},
					Vertex{{3,0,-2}},
					Vertex{{-3,-3,-2}},
					Vertex{{0,-3,-2}},
					Vertex{{3,-3,-2}}
				},
				{
					3,0,4,
					1,5,2,
					2,6,6,
					3,7,4,
					8,5
				},
				PrimitiveTopology::TriangleStrip
			}
		};
			
		//std::vector<Mesh> m_Meshes{
		//Mesh{
		//	{
		//		Vertex{{-3,3,-2}},
		//		Vertex{{0,3,-2}},
		//		Vertex{{3,3,-2}},
		//		Vertex{{ -3,0,-2}},
		//		Vertex{{0,0,-2}},
		//		Vertex{{3,0,-2}},
		//		Vertex{{-3,-3,-2}},
		//		Vertex{{0,-3,-2}},
		//		Vertex{{3,-3,-2}}
		//	},
		//	{
		//		3,0,1,   1,4,3,   4,1,2,
		//		2,5,4,   6,3,4,   4,7,6,
		//		7,4,5,   5,8,7
		//	},
		//	PrimitiveTopology::TriangeList
		//}
		//};
		#pragma endregion
		#pragma endregion

		//Function that transforms the vertices from the mesh from World space to Screen space
		void VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const; //W1 Version
	};
}
