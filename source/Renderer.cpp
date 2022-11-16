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

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow)
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);

	//Create Buffers
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	int size{ m_Width * m_Height };
	m_pColorBuffer = new ColorRGB[size];
	for (int i{0}; i < size; i++)
	{
		m_pColorBuffer[i] = colors::Gray;
	}
	m_pDepthBufferPixels = new float[size];
	for (int i{0}; i < size; i++)
	{
		m_pDepthBufferPixels[i] = FLT_MAX;
	}
	//Initialize Camera
	m_Camera.Initialize(60.f, { .0f,.0f,-10.f });
	
	#ifdef W2Img
	m_pTexture = Texture::LoadFromFile("Resources/uv_grid_2.png");
	#endif
}

Renderer::~Renderer()
{
	#ifdef W2Img
	delete m_pTexture;
	#endif
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

	int size{ m_Width * m_Height };
	for (int i{0}; i < size; i++)
	{
		m_pColorBuffer[i] = colors::Gray;
	}

	for (int i{0}; i < size; i++)
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
	#ifdef W2Img
	RenderMesh();
	#endif // W2Img

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

/// <summary>
/// Rendering the triangles
/// </summary>
/// <param name="verts">The vertexes to loop through</param>
/// <param name="finalColor">The color to output</param>
void dae::Renderer::HandleRender(std::vector<Vertex>& verts, ColorRGB& finalColor)
{
	//Triangle edge
	const Vector2 a{ verts[1].position.x - verts[0].position.x, verts[1].position.y - verts[0].position.y };
	const Vector2 b{ verts[2].position.x - verts[1].position.x, verts[2].position.y - verts[1].position.y };
	const Vector2 c{ verts[0].position.x - verts[2].position.x, verts[0].position.y - verts[2].position.y };

	//Triangle verts
	const Vector2 triangleV0{ verts[0].position.x, verts[0].position.y };
	const Vector2 triangleV1{ verts[1].position.x, verts[1].position.y };
	const Vector2 triangleV2{ verts[2].position.x, verts[2].position.y };
	//Texture* tex = Texture::LoadFromFile("Resources/uv_grid_2.png");

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
				float interpolatedDepth{ 1 / verts[0].position.z * w0 + 1 / verts[1].position.z * w1 + 1 / verts[2].position.z * w2 };
				float actualDepth = 1 / interpolatedDepth;
				if (actualDepth > m_pDepthBufferPixels[currentPixel]) {
					continue;
				}

				m_pDepthBufferPixels[currentPixel] = actualDepth;

				#ifdef W1
				ColorRGB interpolatedColor{ (verts[0].color * w0) + (verts[1].color * w1) + (verts[2].color * w2) };
				#endif

				#ifdef W2
				ColorRGB interpolatedColor{ (verts[0].color * w0) + (verts[1].color * w1) + (verts[2].color * w2) };
				#endif

				#ifdef W2Img
				//ColorRGB interpolatedColor{ (tex->Sample(pixel) * w0) + (tex->Sample(pixel) * w1) + (tex->Sample(pixel) * w2)};
				ColorRGB interpolatedColor{ (m_pTexture->Sample(pixel) * w0) + (m_pTexture->Sample(pixel) * w1) + (m_pTexture->Sample(pixel) * w2)};
				#endif

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

/// <summary>
/// Render the list of triangles (list of a list of Vertexes)
/// </summary>
void dae::Renderer::RenderTriangleList()
{
	ColorRGB finalColor{};
	//1 Loop through triangles
	for (int triangleIter{}; triangleIter < m_Triangles.size(); triangleIter++)
	{
		std::vector<Vertex> verts{};
		VertexTransformationFunction(m_Triangles[triangleIter], verts);

		HandleRender(verts, finalColor);
	}
}

/// <summary>
/// Render the Mesh with List topology
/// </summary>
/// <param name="mesh">The mesh as const ref</param>
void dae::Renderer::RenderMeshTriangleList(const Mesh& mesh)
{
	ColorRGB finalColor{};
	//loop through indices (triangle ID's)
	for (int indiceIter = 0; indiceIter < mesh.indices.size(); indiceIter += 3)
	{
		//for every 3rd indice, calculate the triangle and it's color
		#pragma region Calculate the triangles from a mesh
		int indice1{ int(mesh.indices[indiceIter]) };
		int indice2{ int(mesh.indices[indiceIter + 1]) };
		int indice3{ int(mesh.indices[indiceIter + 2]) };
		std::vector<Vertex> triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };
		#pragma endregion

		std::vector<Vertex> verts{ };
		VertexTransformationFunction(triangleVerts, verts);

		HandleRender(verts, finalColor);
	}
}

/// <summary>
/// Render the Mesh with Strip topology
/// </summary>
/// <param name="mesh">The mesh as const ref</param>
void dae::Renderer::RenderMeshTriangleStrip(const Mesh& mesh)
{
	ColorRGB finalColor{};
	//loop through indices (triangle ID's)
	const int size{ static_cast<int>(mesh.indices.size()) };
	for (int indiceIter = 0; indiceIter < size; indiceIter++)
	{
		#pragma region Calculate the triangles from a mesh
		int indice1{};
		int indice2{};
		int indice3{};
		if (indiceIter == size / 2) {
			indice1 = int(mesh.indices[indiceIter + 1]);
			indice2 = int(mesh.indices[indiceIter + 2]);
			indice3 = int(mesh.indices[indiceIter + 3]);
		}
		else if (indiceIter < size - 2 ){
			indice1 = int(mesh.indices[indiceIter]);
			indice2 = int(mesh.indices[indiceIter + 1]);
			indice3 = int(mesh.indices[indiceIter + 2]);
		}
		std::vector<Vertex> triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };
		#pragma endregion

		std::vector<Vertex> verts{ };
		VertexTransformationFunction(triangleVerts, verts);

		HandleRender(verts, finalColor);
		
	}
}

/// <summary>
/// Transforms the points of the verts to the correct space and location.
/// </summary>
/// <param name="vertices_in">Original Vertexes</param>
/// <param name="vertices_out">Vertexes to output</param>
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

/// <summary>
/// Function that decides which render function to use based on primitive topology of a mesh
/// </summary>
void dae::Renderer::RenderMesh()
{
	//1 Loop through meshes
	for (const Mesh& mesh : m_Meshes)
	{
		switch (mesh.primitiveTopology)
		{
		case PrimitiveTopology::TriangeList:
			RenderMeshTriangleList(mesh);
			break;
		case PrimitiveTopology::TriangleStrip:
			RenderMeshTriangleStrip(mesh);
			break;
		default:
			break;
		}
	}
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}
