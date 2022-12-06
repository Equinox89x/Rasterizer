#pragma once

#include <cstdint>
#include <vector>

#include "Camera.h"
#include "DataTypes.h"
#include "Texture.h"

struct SDL_Window;
struct SDL_Surface;

#pragma region Defines
//Weeks
#define W3_AND_UP

//Features
#define BOUNDINGBOX
#define TEXTURE

//Triangle topologies & meshes
//#define Strip
//#define List
#define MESH
#pragma endregion

namespace dae
{
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

		void CycleLightingMode();
		void ToggleRotation() { m_HasRotation = !m_HasRotation; }
		void ToggleNormalMap() { m_HasNormalMap = !m_HasNormalMap; }

		bool SaveBufferToImage() const;

	private:
		SDL_Window* m_pWindow{};

		SDL_Surface* m_pFrontBuffer{ nullptr };
		SDL_Surface* m_pBackBuffer{ nullptr };
		uint32_t* m_pBackBufferPixels{};

		float* m_pDepthBuffer{};
		ColorRGB* m_pColorBuffer{};
		#ifdef TEXTURE
		Texture* m_pTexture{ nullptr };
		Texture* m_pNormals{ nullptr };
		Texture* m_pGloss{ nullptr };
		Texture* m_pSpecular{ nullptr };
		#endif

		Camera m_Camera{};

		int m_Width{};
		int m_Height{};

		LightingMode m_LightingMode{ LightingMode::ObservedArea };

		Vector3 lightDirection{ .577f, -.577f, .577f };

		bool m_HasRotation{ true };
		bool m_HasNormalMap{ true };

		//Function that transforms the vertices from the mesh from World space to Screen space
		void VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex_Out>& vertices_out, const Matrix& worldMatrix) const;

		void HandleRenderBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor);
		void HandleRenderNoBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor);
		Vertex_Out CalculateVertexWithAttributes(const std::vector<Vertex_Out>& verts, float w0, float w1, float w2, float& outShininess, float& outSpecularKS) const;
		ColorRGB PixelShading(const Vertex_Out& v, const float shininess, const float specularKS) const;

		void RenderMeshTriangleStrip(const Mesh& mesh);
		void RenderMeshTriangleList(const Mesh& mesh);
		//ColorRGB PixelShading(const Vertex_Out& v) const;


		#ifdef Strip
		std::vector<Mesh> m_Meshes{
			Mesh{
				{
					Vertex{{-3,3,-2},{}, {0.f,0.f}},
					Vertex{{0,3,-2},{}, {.5f,0.f}},
					Vertex{{3,3,-2},{}, {1.f,0.f}},
					Vertex{{ -3,0,-2},{}, {0.f,0.5f}},
					Vertex{{0,0,-2},{}, {.5f,.5f}},
					Vertex{{3,0,-2},{}, {1.f,.5f}},
					Vertex{{-3,-3,-2},{}, {0.f,1.f}},
					Vertex{{0,-3,-2},{}, {.5,1.f}},
					Vertex{{3,-3,-2},{}, {1.f,1.f}}
				},
				{
					3,0,4,1,5,2,
					2,6,
					6,3,7,4,8,5
				},
				PrimitiveTopology::TriangleStrip
			}
		};
		#endif // Strip
		
		#ifdef List
		const std::vector<Mesh> m_Meshes{
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
				3,0,1,   1,4,3,   4,1,2,
				2,5,4,   6,3,4,   4,7,6,
				7,4,5,   5,8,7
			},
			PrimitiveTopology::TriangeList
		}
		};
		#endif // List
		#pragma endregion

		#ifdef MESH
		std::vector<Vertex> m_Verts{ };
		std::vector<uint32_t> m_Indices{ };
		std::vector<Mesh> m_Meshes{Mesh{m_Verts,m_Indices, PrimitiveTopology::TriangeList}};
		#endif
		
	};
}
