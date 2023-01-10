//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"

#include <iostream>

#include "Material.h"
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
	m_Camera.Initialize(45.f, { 0.f,0.f,0.f });
	const float screenWidth{ static_cast<float>(m_Width) };
	const float screenHeight{ static_cast<float>(m_Height) };
	m_Camera.aspectRatio = screenWidth / screenHeight;

	#ifdef TEXTURE
	#ifndef MESH
	m_pTexture = Texture::LoadFromFile("Resources/uv_grid_2.png");
	#endif
	#ifdef MESH
	//m_pTexture = Texture::LoadFromFile("Resources/tuktuk.png");
	m_pTexture = Texture::LoadFromFile("Resources/vehicle_diffuse.png");
	m_pNormals = Texture::LoadFromFile("Resources/vehicle_normal.png");
	m_pGloss = Texture::LoadFromFile("Resources/vehicle_gloss.png");
	m_pSpecular = Texture::LoadFromFile("Resources/vehicle_specular.png");
	#endif
	#endif

	#ifdef MESH
	//Utils::ParseOBJ("Resources/tuktuk.obj",
	//	m_Meshes[0].vertices,
	//	m_Meshes[0].indices
	//);

	Utils::ParseOBJ("Resources/vehicle.obj",
		m_Meshes[0].vertices,
		m_Meshes[0].indices
	);

	Matrix trans{ Matrix::CreateTranslation(0,0,50) };
	Matrix rot{ Matrix::CreateRotationY(14)};
	Matrix scale = Matrix::CreateScale(1, 1, 1);
	m_Meshes[0].rotationTransform = rot;
	m_Meshes[0].scaleTransform = scale;
	m_Meshes[0].translationTransform = trans;
	m_Meshes[0].worldMatrix = scale * rot * trans;
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

	if (m_HasRotation) {
		m_RotationAngle = (cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2 ; 
		//std::cout << m_RotationAngle << "\n";

		for (Mesh& mesh : m_Meshes)
		{
			mesh.rotationTransform = Matrix::CreateRotationY(m_RotationAngle);
			mesh.worldMatrix = mesh.scaleTransform * mesh.rotationTransform * mesh.translationTransform;
		}
	}
}

void Renderer::Render() const
{
	//@START
	//Lock BackBuffer
	SDL_LockSurface(m_pBackBuffer);

	const int size{ m_Width * m_Height };
	for (int i{ 0 }; i < size; i++)
	{
		m_pDepthBuffer[i] = FLT_MAX;
	}

	for (int i{ 0 }; i < size; i++)
	{
		m_pColorBuffer[i] = colors::Gray;
	}
	SDL_FillRect(m_pBackBuffer, nullptr, 0x808080);


	//RENDER LOGIC
	//Loop through meshes and perform correct render based on topology
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

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::HandleRenderBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor) const
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
	const float maxX = std::max(std::max(triangleV2.x, triangleV0.x), std::max(triangleV0.x, triangleV1.x));
	const float minX = std::min(std::min(triangleV0.x, triangleV1.x), std::min(triangleV2.x, triangleV0.x));
	const float maxY = std::max(std::max(triangleV0.y, triangleV1.y), std::max(triangleV2.y, triangleV0.y));
	const float minY = std::min(std::min(triangleV0.y, triangleV1.y), std::min(triangleV2.y, triangleV0.y));

	if (
		((minX >= 0) && (maxX <= (m_Width - 1))) &&
		((minY >= 0) && (maxY <= (m_Height - 1)))) {
		//Loop through bounding box pixels and ceil to remove lines between triangles.
		for (int px{ static_cast<int>(minX) }; px < std::ceil(maxX); ++px)
		{
			for (int py{ static_cast<int>(minY) }; py < std::ceil(maxY); ++py)
			{
				const int currentPixel{ px + (py * m_Width) };
				if (!(currentPixel > 0 && currentPixel < m_Width * m_Height)) continue;
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
					const float interpolatedDepth{ 1 / ((1 / verts[0].position.z) * w0 + (1 / verts[1].position.z) * w1 + (1 / verts[2].position.z) * w2) };
					if (interpolatedDepth > m_pDepthBuffer[currentPixel]) {
						continue;
					}
					m_pDepthBuffer[currentPixel] = interpolatedDepth;

					//Deciding color
					#ifndef TEXTURE
					ColorRGB interpolatedColor{ (verts[0].color * w0) + (verts[1].color * w1) + (verts[2].color * w2) };
					#endif

					#ifdef TEXTURE
					#ifndef W3_AND_UP
					Vector2 interpolatedUV{
						(((verts[0].uv / verts[0].position.z) * w0) +
						((verts[1].uv / verts[1].position.z) * w1) +
						((verts[2].uv / verts[2].position.z) * w2)) * interpolatedDepth };
					m_pColorBuffer[currentPixel] = interpolatedColor;
					#endif					
					#ifdef W3_AND_UP
					float gloss{ 0 };
					ColorRGB specularKS{  };
					Vertex_Out pixelVertexPos{ CalculateVertexWithAttributes(verts, w0, w1, w2, gloss, specularKS)};
					m_pColorBuffer[currentPixel] = PixelShading(pixelVertexPos, gloss, specularKS);

					#endif
					#endif
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
}

Vertex_Out Renderer::CalculateVertexWithAttributes(const std::vector<Vertex_Out>& verts, const float w0, const float w1, const float w2, float& outGloss, ColorRGB& outSpecularKS) const
{
	#pragma region calculate interpolated attributes
	const float interpolatedDepthW{ 1 / (
		(1 / verts[0].position.w) * w0 +
		(1 / verts[1].position.w) * w1 +
		(1 / verts[2].position.w) * w2) };

	const Vector2 interpolatedUV{
		(((verts[0].uv / verts[0].position.w) * w0) +
			((verts[1].uv / verts[1].position.w) * w1) +
			((verts[2].uv / verts[2].position.w) * w2)) * interpolatedDepthW };

	const Vector3 interpolatedNormal{ (
		(verts[0].normal / (verts[0].position.w)) * w0 +
		(verts[1].normal / verts[1].position.w) * w1 +
		(verts[2].normal / verts[2].position.w) * w2) * interpolatedDepthW };

	const Vector3 interpolatedTangent{ (
		(verts[0].tangent / (verts[0].position.w)) * w0 +
		(verts[1].tangent / verts[1].position.w) * w1 +
		(verts[2].tangent / verts[2].position.w) * w2) * interpolatedDepthW };

	const Vector3 viewDirection{ (
		(verts[0].viewDirection / (verts[0].position.w)) * w0 +
		(verts[1].viewDirection / verts[1].position.w) * w1 +
		(verts[2].viewDirection / verts[2].position.w) * w2) * interpolatedDepthW };

	const Vector4 interpolatedPosition{ (
		verts[0].position * w0 +
		verts[1].position * w1 +
		verts[2].position * w2) * interpolatedDepthW };
	#pragma endregion

	//color from diffuse map
	#ifdef TEXTURE
	const ColorRGB currentColor{ m_pTexture->Sample(interpolatedUV) };
	#endif // TEXTURE

	#ifdef MESH
	#pragma region normals
	const auto [Nr, Ng, Nb]{ m_pNormals->Sample(interpolatedUV) };
	const Vector3 binormal = Vector3::Cross(interpolatedNormal, interpolatedTangent);
	const Matrix tangentSpaceAxis{ Matrix{ interpolatedTangent,binormal,interpolatedNormal,Vector3::Zero } };

	Vector3 sampledNormal{ Nr,Ng,Nb };
	sampledNormal = 2.f * sampledNormal - Vector3{1.f, 1.f, 1.f};
	sampledNormal /= 255.f;
	sampledNormal = tangentSpaceAxis.TransformVector(sampledNormal).Normalized();
	#pragma endregion

	#pragma region Phong
	//gloss
	const auto [Gr, Gg, Gb]{ m_pGloss->Sample(interpolatedUV) };
	outGloss = Gr;

	//specular
	outSpecularKS = m_pSpecular->Sample(interpolatedUV);
	#pragma endregion

	return { interpolatedPosition, currentColor, interpolatedUV, m_HasNormalMap ? sampledNormal.Normalized() : interpolatedNormal, interpolatedTangent, viewDirection.Normalized() };
	#endif // MESH
	#ifndef MESH
	return { interpolatedPosition, verts[0].color, interpolatedUV, interpolatedNormal, interpolatedTangent, viewDirection.Normalized() };
	#endif
}

ColorRGB Renderer::PixelShading(const Vertex_Out& v, const float gloss, const ColorRGB specularKS) const
{
	constexpr float kd{ 7.f }; // = diffuse reflectance = diffuse specularity
	constexpr float shininess{ 25.f };
	float ObservedArea{ Vector3::Dot(v.normal, -lightDirection) };
	ObservedArea = Clamp(ObservedArea, 0.f, 1.f);

	switch (m_LightingMode)
	{
		case LightingMode::ObservedArea: {
			return ColorRGB{ ObservedArea,ObservedArea,ObservedArea };
		}

		case LightingMode::Diffuse:{
			Material_Lambert material{ Material_Lambert(v.color, kd) };
			const ColorRGB diffuse{ material.Shade(v) * ObservedArea };
			return diffuse;
		}

		case LightingMode::Specular: {
			const ColorRGB specular{ BRDF::Phong(specularKS, gloss * shininess, lightDirection, v.viewDirection, v.normal) };
			return specular;
		}

		case LightingMode::Combined: {
			Material_Lambert material{ Material_Lambert(v.color, kd) };
			const ColorRGB diffuse{ material.Shade(v) * ObservedArea };
			const ColorRGB specular{ BRDF::Phong(specularKS, gloss * shininess, lightDirection, v.viewDirection, v.normal) };
			const ColorRGB phong{ diffuse + specular };
			return phong;
		}

		default: {
			Material_Lambert material{ Material_Lambert(v.color, kd) };
			const ColorRGB diffuse{ material.Shade(v) * ObservedArea };
			const ColorRGB specular{ BRDF::Phong(specularKS, gloss * shininess, lightDirection, v.viewDirection, v.normal) };
			const ColorRGB phong{ diffuse + specular };
			return phong;
		}
	}
}

void Renderer::RenderMeshTriangleList(const Mesh& mesh) const
{
	ColorRGB finalColor{};
	
	//loop through indices (triangle ID's)
	for (int indiceIter = 0; indiceIter < mesh.indices.size(); indiceIter += 3)
	{
		//for every 3rd indice, calculate the triangle
		#pragma region Calculate the triangles from a mesh
		const int indice1{ static_cast<int>(mesh.indices[indiceIter]) };
		const int indice2{ static_cast<int>(mesh.indices[indiceIter + 1]) };
		const int indice3{ static_cast<int>(mesh.indices[indiceIter + 2]) };
		std::vector triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };
		#pragma endregion

		std::vector<Vertex_Out> verts{ };	
		VertexTransformationFunction(triangleVerts, verts, mesh.worldMatrix);

		HandleRenderBB(verts, finalColor);
	}
}

void Renderer::RenderMeshTriangleStrip(const Mesh& mesh) const
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
			indice1 = static_cast<int>(mesh.indices[indiceIter + 1]);
			indice2 = static_cast<int>(mesh.indices[indiceIter + 2]);
			indice3 = static_cast<int>(mesh.indices[indiceIter + 3]);
		}
		else if (indiceIter < size - 2) {
			indice1 = static_cast<int>(mesh.indices[indiceIter]);
			indice2 = static_cast<int>(mesh.indices[indiceIter + 1]);
			indice3 = static_cast<int>(mesh.indices[indiceIter + 2]);
		}
		std::vector<Vertex_Out> verts{ };
		std::vector triangleVerts{ mesh.vertices[indice1], mesh.vertices[indice2], mesh.vertices[indice3] };
		#pragma endregion

		VertexTransformationFunction(triangleVerts, verts, mesh.worldMatrix);

		HandleRenderBB(verts, finalColor);
	}
}

void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex_Out>& vertices_out, const Matrix& worldMatrix) const
{
	vertices_out.resize(vertices_in.size());

	//Add viewmatrix with camera space matrix
	const Matrix worldViewProjectionMatrix{ worldMatrix * m_Camera.viewMatrix * m_Camera.projectionMatrix };

	for (int i{}; i < vertices_in.size(); i++)
	{
		Vector4 point{ vertices_in[i].position, 1 };

		//Transform points to correct space
		Vector4 transformedVert{ worldViewProjectionMatrix.TransformPoint(point) };

		//Project point to 2d view plane (perspective divide)
		const float projectedVertexW{ transformedVert.w };
		float projectedVertexX{ transformedVert.x / transformedVert.w };
		float projectedVertexY{ transformedVert.y / transformedVert.w };
		float projectedVertexZ{ transformedVert.z / transformedVert.w };
		projectedVertexX = ((projectedVertexX + 1) / 2) * m_Width;
		projectedVertexY = ((1 - projectedVertexY) / 2) * m_Height;

		//transform normals to correct space and solve visibility problem
		const Vector3 normal{ worldMatrix.TransformVector(vertices_in[i].normal) };
		const Vector3 tangent{ worldMatrix.TransformVector(vertices_in[i].tangent) };

		Vector4 pos{ projectedVertexX, projectedVertexY , projectedVertexZ, projectedVertexW };
		const Vector3 viewDirection{ m_Camera.origin - transformedVert };

		if (!(pos.x < -1 && pos.x > 1) && !(pos.y < -1 && pos.y > 1)) {
			vertices_out[i] = { pos, vertices_in[i].color,  vertices_in[i].uv, normal, tangent, viewDirection };
		}
	}
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}

void Renderer::CycleLightingMode() {
	//if it's combined, start from first enum, otherwise take the next
	m_LightingMode == LightingMode::Combined ?
		m_LightingMode = LightingMode(0) :
		m_LightingMode = LightingMode(static_cast<int>(m_LightingMode) + 1);
}