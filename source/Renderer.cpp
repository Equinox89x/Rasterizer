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
	m_pDepthBuffer = new float[size];

	//Initialize Camera
	m_Camera.Initialize(60.f, { .0f,.0f,-10.f });

	#ifdef TEXTURE
	m_pTexture = Texture::LoadFromFile("Resources/uv_grid_2.png");
	#endif
}

Renderer::~Renderer()
{
	#ifdef TEXTURE
	delete m_pTexture;
	m_pTexture = nullptr;
	#endif

	delete[] m_pDepthBuffer;
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

	const int size{ m_Width * m_Height };
	for (int i{ 0 }; i < size; i++)
	{
		m_pColorBuffer[i] = colors::Gray;
	}

	for (int i{ 0 }; i < size; i++)
	{
		m_pDepthBuffer[i] = FLT_MAX;
	}

	const float screenWidth{ static_cast<float>(m_Width) };
	const float screenHeight{ static_cast<float>(m_Height) };
	m_Camera.aspectRatio = screenWidth / screenHeight;

	//RENDER LOGIC
	#ifndef TEXTURE
	RenderTriangleList();
	#endif // W1
	#ifdef TEXTURE
	RenderMesh();
	#endif // W2Img

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

/// <summary>
/// Rendering the triangles without bounding box
/// </summary>
/// <param name="verts">The vertexes to loop through</param>
/// <param name="finalColor">The color to output</param>
void dae::Renderer::HandleRenderNoBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor) {
	//Triangle edge
	const Vector2 a{ verts[1].position.x - verts[0].position.x, verts[1].position.y - verts[0].position.y };
	const Vector2 b{ verts[2].position.x - verts[1].position.x, verts[2].position.y - verts[1].position.y };
	const Vector2 c{ verts[0].position.x - verts[2].position.x, verts[0].position.y - verts[2].position.y };

	//Triangle verts
	const Vector2 triangleV0{ verts[0].position.x, verts[0].position.y };
	const Vector2 triangleV1{ verts[1].position.x, verts[1].position.y };
	const Vector2 triangleV2{ verts[2].position.x, verts[2].position.y };

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			const int currentPixel{ px + (py * m_Width) };
			const Vector2 pixel{ static_cast<float>(px) + 0.5f, static_cast<float>(py) + 0.5f };

			//Pixel position to vertices (also the weight)
			Vector2 pointToSide{ pixel - triangleV1 };
			const float edgeA{ Vector2::Cross(b, pointToSide) };

			pointToSide = pixel - triangleV2;
			const float edgeB{ Vector2::Cross(c, pointToSide) };

			pointToSide = pixel - triangleV0;
			const float edgeC{ Vector2::Cross(a, pointToSide) };

			const float triangleArea{ edgeA + edgeB + edgeC };
			const float w0{ edgeA / triangleArea };
			const float w1{ edgeB / triangleArea };
			const float w2{ edgeC / triangleArea };

			//check if pixel is inside triangle
			if (w0 > 0.f && w1 > 0.f && w2 > 0.f) {

				//depth test
				float interpolatedDepth{ 1 / ((1 / verts[0].position.z) * w0 + (1 / verts[1].position.z) * w1 + (1 / verts[2].position.z) * w2) };
				if (interpolatedDepth > m_pDepthBuffer[currentPixel]) {
					continue;
				}
				m_pDepthBuffer[currentPixel] = interpolatedDepth;

				//Deciding color
				#ifndef TEXTURE
				ColorRGB interpolatedColor{ (verts[0].color * w0) + (verts[1].color * w1) + (verts[2].color * w2) };
				#endif

				#ifdef TEXTURE
				Vector2 interpolatedUV{
					(((verts[0].uv / verts[0].position.z) * w0) +
					((verts[1].uv / verts[1].position.z) * w1) +
					((verts[2].uv / verts[2].position.z) * w2)) * interpolatedDepth };

				ColorRGB interpolatedColor{ m_pTexture->Sample(interpolatedUV) };
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
/// Rendering the triangles with bounding box
/// </summary>
/// <param name="verts">The vertexes to loop through</param>
/// <param name="finalColor">The color to output</param>
void dae::Renderer::HandleRenderBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor)
{
	//Triangle edge
	const Vector2 a{ verts[1].position.x - verts[0].position.x, verts[1].position.y - verts[0].position.y };
	const Vector2 b{ verts[2].position.x - verts[1].position.x, verts[2].position.y - verts[1].position.y };
	const Vector2 c{ verts[0].position.x - verts[2].position.x, verts[0].position.y - verts[2].position.y };

	//Triangle verts
	const Vector2 triangleV0{ verts[0].position.x, verts[0].position.y };
	const Vector2 triangleV1{ verts[1].position.x, verts[1].position.y };
	const Vector2 triangleV2{ verts[2].position.x, verts[2].position.y };

	//find the top left and bottom right point of the bounding box
	float maxX = std::max(std::max(triangleV2.x, triangleV0.x), std::max(triangleV0.x, triangleV1.x));
	float minX = std::min(std::min(triangleV0.x, triangleV1.x), std::min(triangleV2.x, triangleV0.x));
	float maxY = std::max(std::max(triangleV0.y, triangleV1.y), std::max(triangleV2.y, triangleV0.y));
	float minY = std::min(std::min(triangleV0.y, triangleV1.y), std::min(triangleV2.y, triangleV0.y));

	//Loop through bounding box pixels
	for (int px{ int(minX) }; px < int(maxX); ++px)
	{
		for (int py{ int(minY) }; py < int(maxY); ++py)
		{
			const int currentPixel{ px + (py * m_Width) };
			if ((0 >= maxX <= (m_Width - 1)) &&
				(0 >= maxY <= (m_Width - 1)) &&
				(0 >= minX <= (m_Height - 1)) &&
				(0 >= minY <= (m_Height - 1))) {
				const Vector2 pixel{ static_cast<float>(px) + 0.5f, static_cast<float>(py) + 0.5f };

				//Pixel position to vertices (also the weight)
				Vector2 pointToSide{ pixel - triangleV1 };
				const float edgeA{ Vector2::Cross(b, pointToSide) };

				pointToSide = pixel - triangleV2;
				const float edgeB{ Vector2::Cross(c, pointToSide) };

				pointToSide = pixel - triangleV0;
				const float edgeC{ Vector2::Cross(a, pointToSide) };

				const float triangleArea{ edgeA + edgeB + edgeC };
				const float w0{ edgeA / triangleArea };
				const float w1{ edgeB / triangleArea };
				const float w2{ edgeC / triangleArea };

				//check if pixel is inside triangle
				if (w0 > 0.f && w1 > 0.f && w2 > 0.f) {

					//depth test
					float interpolatedDepth{ 1 / ((1 / verts[0].position.z) * w0 + (1 / verts[1].position.z) * w1 + (1 / verts[2].position.z) * w2) };
					if (interpolatedDepth > m_pDepthBuffer[currentPixel]) {
						continue;
					}
					m_pDepthBuffer[currentPixel] = interpolatedDepth;

					//Deciding color
					#ifndef TEXTURE
					ColorRGB interpolatedColor{ (verts[0].color * w0) + (verts[1].color * w1) + (verts[2].color * w2) };
					#endif

					#ifdef TEXTURE
					Vector2 interpolatedUV{
						(((verts[0].uv / verts[0].position.z) * w0) +
						((verts[1].uv / verts[1].position.z) * w1) +
						((verts[2].uv / verts[2].position.z) * w2)) * interpolatedDepth };

					ColorRGB interpolatedColor{ m_pTexture->Sample(interpolatedUV) };
					#endif

					m_pColorBuffer[currentPixel] = interpolatedColor;
				}
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
		std::vector<Vertex_Out> verts{};
		VertexTransformationFunction(m_Triangles[triangleIter], verts);

		#ifndef BOUNDINGBOX
		HandleRenderNoBB(verts, finalColor);
		#endif // BOUNDINGBOX
		#ifdef BOUNDINGBOX
		HandleRenderBB(verts, finalColor);
		#endif // BOUNDINGBOX
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
		const int indice1{ int(mesh.indices[indiceIter]) };
		const int indice2{ int(mesh.indices[indiceIter + 1]) };
		const int indice3{ int(mesh.indices[indiceIter + 2]) };
		std::vector<Vertex> triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };
		#pragma endregion

		std::vector<Vertex_Out> verts{ };
		VertexTransformationFunction(triangleVerts, verts);

		#ifndef BOUNDINGBOX
		HandleRenderNoBB(verts, finalColor);
		#endif // BOUNDINGBOX
		#ifdef BOUNDINGBOX
		HandleRenderBB(verts, finalColor);
		#endif // BOUNDINGBOX
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
		else if (indiceIter < size - 2) {
			indice1 = int(mesh.indices[indiceIter]);
			indice2 = int(mesh.indices[indiceIter + 1]);
			indice3 = int(mesh.indices[indiceIter + 2]);
		}
		std::vector<Vertex> triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };
		#pragma endregion

		std::vector<Vertex_Out> verts{ };
		VertexTransformationFunction(triangleVerts, verts);

		#ifndef BOUNDINGBOX
		HandleRenderNoBB(verts, finalColor);
		#endif // BOUNDINGBOX
		#ifdef BOUNDINGBOX
		HandleRenderBB(verts, finalColor);
		#endif // BOUNDINGBOX
	}
}

/// <summary>
/// Transforms the points of the verts to the correct space and location.
/// </summary>
/// <param name="vertices_in">Original Vertexes</param>
/// <param name="vertices_out">Vertexes to output</param>
void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex_Out>& vertices_out) const
{
	vertices_out.resize(vertices_in.size());

	Matrix viewMatrix{ m_Camera.invViewMatrix };
	for (int i{}; i < vertices_in.size(); i++)
	{

		#ifndef W3
		//Transform points to camera space
		Vector3 transformedVert{ viewMatrix.TransformPoint(vertices_in[i].position) };
		//Project point to 2d view plane (perspective divide)
		float projectedVertexX{ transformedVert.x / transformedVert.z };
		float projectedVertexY{ transformedVert.y / transformedVert.z };
		float projectedVertexZ{ transformedVert.z };

		//Calculate with camera settings
		projectedVertexX = projectedVertexX / (m_Camera.aspectRatio * m_Camera.fov);
		projectedVertexY = projectedVertexY / m_Camera.fov;

		//Convert Points To ScreenSpace (raster space)
		projectedVertexX = ((projectedVertexX + 1) / 2) * m_Width;
		projectedVertexY = ((1 - projectedVertexY) / 2) * m_Height;

		Vertex vert{ Vector3{ projectedVertexX, projectedVertexY , projectedVertexZ }, vertices_in[i].color,  vertices_in[i].uv };
		vertices_out[i] = Vertex_Out{ Vector4{vert.position, projectedVertexZ}, vert.color, vert.uv };
		#endif

		#ifdef W3
		Vector4 point{ vertices_in[i].position, 1 };
		//Add viewmatrix with camera space matrix
		Matrix worldViewProjectionMatrix{ viewMatrix * m_Camera.projectionMatrix };
		//Transform points to correct space
		Vector4 transformedVert{ worldViewProjectionMatrix.TransformPoint(point) };

		//Project point to 2d view plane (perspective divide)
		float projectedVertexX{ transformedVert.x / transformedVert.z };
		float projectedVertexY{ transformedVert.y / transformedVert.z };
		float projectedVertexZ{ transformedVert.z / transformedVert.w };
		float projectedVertexW{ transformedVert.z };

		if (!(projectedVertexX < -1 && projectedVertexX > 1) && !(projectedVertexY < -1 && projectedVertexY > 1)) {
			//if (projectedVertexZ > 0 && projectedVertexZ < 1) {
			Vertex_Out vert{ Vector4{projectedVertexX, projectedVertexY , projectedVertexZ, projectedVertexW}, vertices_in[i].color,  vertices_in[i].uv };
			vertices_out[i] = vert;
			//}
		}
		#endif
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
