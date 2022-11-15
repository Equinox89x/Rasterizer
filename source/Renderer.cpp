//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Texture.h"
#include "Utils.h"

using namespace dae;

//#define W1
#define W2

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow)
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);

	//Create Buffers
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	m_pColorBuffer = new ColorRGB[m_Width * m_Height];
	for (size_t i = 0; i < m_Width * m_Height; i++)
	{
		m_pColorBuffer[i] = colors::Gray;
	}
	m_pDepthBufferPixels = new float[m_Width * m_Height];
	for (size_t i = 0; i < m_Width * m_Height; i++)
	{
		m_pDepthBufferPixels[i] = FLT_MAX;
	}

	//Initialize Camera
	m_Camera.Initialize(60.f, { .0f,.0f,-10.f });
}

Renderer::~Renderer()
{
	delete[] m_pDepthBufferPixels;
	delete[] m_pColorBuffer;
}

void Renderer::Update(Timer* pTimer)
{
	m_Camera.Update(pTimer);
}

void Renderer::Render()
{
	//@START
	//Lock BackBuffer
	SDL_LockSurface(m_pBackBuffer);
	
	for (size_t i = 0; i < m_Width * m_Height; i++)
	{
		m_pColorBuffer[i] = colors::Gray;
	}

	for (size_t i = 0; i < m_Width * m_Height; i++)
	{
		m_pDepthBufferPixels[i] = FLT_MAX;
	}

	//RENDER LOGIC
	#ifdef W1
	RenderTriangleList();
	#endif // W1
	#ifdef W2
	RenderMesh();
	#endif // W2

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderTriangleList()
{
	ColorRGB finalColor{};
	//1 Loop through triangles
	for (int triangleIter{}; triangleIter < m_Triangles.size(); triangleIter++)
	{
		#pragma region Vertex screenCoordinate
		//with triangle list
		std::vector<Vertex> verts{};
		VertexTransformationFunction(m_Triangles[triangleIter], verts);

		//Triangle edge
		const Vector2 a{ verts[1].position.x - verts[0].position.x, verts[1].position.y - verts[0].position.y };
		const Vector2 b{ verts[2].position.x - verts[1].position.x, verts[2].position.y - verts[1].position.y };
		const Vector2 c{ verts[0].position.x - verts[2].position.x, verts[0].position.y - verts[2].position.y };

		//Triangle verts
		const Vector2 triangleV0{ verts[0].position.x, verts[0].position.y };
		const Vector2 triangleV1{ verts[1].position.x, verts[1].position.y };
		const Vector2 triangleV2{ verts[2].position.x, verts[2].position.y };
		#pragma endregion

		HandleRender(triangleV0, triangleV1, triangleV2, a, b, c, verts, finalColor);
	}
}

void dae::Renderer::HandleRender(const Vector2& triangleV0, const Vector2& triangleV1, const Vector2& triangleV2, const Vector2& a, const Vector2& b, const Vector2& c, std::vector<Vertex>& verts, ColorRGB& finalColor)
{
	//Loop through vertices
	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			const int currentPixel{ px + (py * m_Width) };
			Vector2 pixel{ static_cast<float>(px), static_cast<float>(py) };

			//Pixel position to vertices (also the weight)
			Vector2 pointToSide{ pixel - triangleV1 };
			float edgeA{ Vector2::Cross(b, pointToSide) };

			pointToSide = pixel - triangleV2;
			float edgeB{ Vector2::Cross(c, pointToSide) };

			pointToSide = pixel - triangleV0;
			float edgeC{ Vector2::Cross(a, pointToSide) };

			float triangleArea{ edgeA + edgeB + edgeC };
			float w0{ edgeA / triangleArea };
			float w1{ edgeB / triangleArea };
			float w2{ edgeC / triangleArea };

			//check if pixel is inside triangle
			if (w0 > 0.f && w1 > 0.f && w2 > 0.f) {
				//check if totalweight divided by area is 1
				//float totalWeight{ w0 + w1 + w2 };
				//if (totalWeight == 1) {}

				//depth test
				float interpolatedDepth{ w2 - w0 };
				float depth{ w0 + ((w1 / 100) * interpolatedDepth) }; // 100 -> for percentage
				if (depth > m_pDepthBufferPixels[currentPixel]) {
					continue;
				}
				m_pDepthBufferPixels[currentPixel] = depth;
				ColorRGB interpolatedColor{ (verts[0].color * w0) + (verts[1].color * w1) + (verts[2].color * w2) };
				m_pColorBuffer[currentPixel] = interpolatedColor;
			}

			//change color accordingly to triangle
			finalColor = m_pColorBuffer[currentPixel];

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBackBufferPixels[currentPixel] = SDL_MapRGB(m_pBackBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}
}

void dae::Renderer::RenderMesh()
{
	//1 Loop through meshes
	for (int triangleIter = 0; triangleIter < m_Meshes.size(); triangleIter++)
	{
		Mesh& mesh{ m_Meshes[triangleIter] };
		switch (mesh.primitiveTopology)
		{
		case PrimitiveTopology::TriangeList:
			RenderMeshTriangleList(mesh);
			break;
		case PrimitiveTopology::TriangleStrip:
			RenderTriangleStrip(mesh);
			break;
		default:
			break;
		}
	}
}

void dae::Renderer::RenderMeshTriangleList(const Mesh& mesh)
{
	ColorRGB finalColor{};
	//loop through indices (triangle ID's)
	for (int indiceIter = 0; indiceIter < mesh.indices.size(); indiceIter++)
	{
		//for every 3rd indice, calculate the triangle and it's color
		if (indiceIter % 3 == 0) {
			int indice1{ int(mesh.indices[indiceIter]) };
			int indice2{ int(mesh.indices[indiceIter + 1]) };
			int indice3{ int(mesh.indices[indiceIter + 2]) };
			std::vector<Vertex> triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };

			#pragma region Vertex screenCoordinate
			std::vector<Vertex> verts{ };
			VertexTransformationFunction(triangleVerts, verts);

			//Triangle edge
			const Vector2 a{ verts[1].position.x - verts[0].position.x, verts[1].position.y - verts[0].position.y };
			const Vector2 b{ verts[2].position.x - verts[1].position.x, verts[2].position.y - verts[1].position.y };
			const Vector2 c{ verts[0].position.x - verts[2].position.x, verts[0].position.y - verts[2].position.y };

			//Triangle verts
			const Vector2 triangleV0{ verts[0].position.x, verts[0].position.y };
			const Vector2 triangleV1{ verts[1].position.x, verts[1].position.y };
			const Vector2 triangleV2{ verts[2].position.x, verts[2].position.y };
			#pragma endregion

			HandleRender(triangleV0, triangleV1, triangleV2, a, b, c, verts, finalColor);
		}
	}
}

void dae::Renderer::RenderTriangleStrip(const Mesh& mesh)
{
	ColorRGB finalColor{};
	//loop through indices (triangle ID's)
	for (int indiceIter = 0; indiceIter < mesh.indices.size(); indiceIter++)
	{
		//for every 3rd indice, calculate the triangle and it's color
		if (indiceIter % 3 == 0) {
			int indice1{ int(mesh.indices[indiceIter]) };
			int indice2{ int(mesh.indices[indiceIter + 1]) };
			int indice3{};
			if (indiceIter == mesh.indices.size() - 2) {
				indice3 = int(mesh.indices[0]);
			}
			else {
				indice3 = int(mesh.indices[indiceIter + 2]);
			}
			std::vector<Vertex> triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };

			#pragma region Vertex screenCoordinate
			std::vector<Vertex> verts{ };
			VertexTransformationFunction(triangleVerts, verts);

			//Triangle edge
			const Vector2 a{ verts[1].position.x - verts[0].position.x, verts[1].position.y - verts[0].position.y };
			const Vector2 b{ verts[2].position.x - verts[1].position.x, verts[2].position.y - verts[1].position.y };
			const Vector2 c{ verts[0].position.x - verts[2].position.x, verts[0].position.y - verts[2].position.y };

			//Triangle verts
			const Vector2 triangleV0{ verts[0].position.x, verts[0].position.y };
			const Vector2 triangleV1{ verts[1].position.x, verts[1].position.y };
			const Vector2 triangleV2{ verts[2].position.x, verts[2].position.y };
			#pragma endregion

			HandleRender(triangleV0, triangleV1, triangleV2, a, b, c, verts, finalColor);
		}
	}
}

void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const
{
	const float screenWidth{ static_cast<float>(m_Width) };
	const float screenHeight{ static_cast<float>(m_Height) };
	const float aspectRatio{ screenWidth / screenHeight };

	vertices_out.resize(vertices_in.size());

	Matrix viewMatrix{ m_Camera.invViewMatrix};
	for (int i{}; i < vertices_in.size(); i++)
	{
		//Transform points to camera space
		Vector3 transformedVert{ viewMatrix.TransformPoint(vertices_in[i].position) };

		//Project point to 2d view plane (perspective divide)
		float projectedVertexX{ transformedVert.x / transformedVert.z };
		float projectedVertexY{ transformedVert.y / transformedVert.z };
		float projectedVertexZ{ transformedVert.z };

		//Calculate with camera settings
		projectedVertexX = projectedVertexX / (aspectRatio * m_Camera.fov);
		projectedVertexY = projectedVertexY / m_Camera.fov;

		//Convert Points To ScreenSpace (raster space)
		projectedVertexX = ((projectedVertexX + 1) / 2) * screenWidth;
		projectedVertexY = ((1 - projectedVertexY) / 2) * screenHeight;

		Vertex vert{ Vector3{ projectedVertexX, projectedVertexY , projectedVertexZ }, vertices_in[i].color };
		vertices_out[i] = vert;
	}
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}
