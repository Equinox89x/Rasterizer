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
	m_Meshes[0].worldMatrix = rot * trans;
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
	//float yawAngle{ (cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2 };
	//for (Mesh& mesh : m_Meshes)
	//{
	//	mesh.rotationTransform = Matrix::CreateRotationY(yawAngle);

	//	mesh.totalTranslation += mesh.worldMatrix.GetTranslation();

	//	Matrix totalTrans = Matrix::CreateTranslation(mesh.totalTranslation);
	//	Matrix totalTransNegative = Matrix::CreateTranslation(-mesh.totalTranslation);

	//	//bring the object back to its original position, apply the transform, then bring it back to the new position
	//	//const Matrix finalTransform{ totalTransNegative * mesh.scaleTransform * mesh.rotationTransform * mesh.translationTransform * totalTrans };
	//	mesh.worldMatrix = totalTransNegative * mesh.scaleTransform * mesh.rotationTransform * mesh.translationTransform * totalTrans;
	//}
}

void Renderer::Render()
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
	SDL_FillRect(m_pBackBuffer, NULL, 0x808080);


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

/// <summary>
/// Rendering the triangles without bounding box
/// </summary>
/// <param name="verts">The vertexes to loop through</param>
/// <param name="finalColor">The color to output</param>
void Renderer::HandleRenderNoBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor)
{
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
				#endif					
				#ifdef W3_AND_UP
				const float interpolatedDepthW{ 1 / ((1 / verts[0].position.w) * w0 + (1 / verts[1].position.w) * w1 + (1 / verts[2].position.w) * w2) };
				const Vector2 interpolatedUV{
					(((verts[0].uv / verts[0].position.w) * w0) +
					((verts[1].uv / verts[1].position.w) * w1) +
					((verts[2].uv / verts[2].position.w) * w2)) * interpolatedDepthW };
				#endif

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
void Renderer::HandleRenderBB(std::vector<Vertex_Out>& verts, ColorRGB& finalColor)
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
					float shininess{ 0 };
					float specularKS{ 0 };
					Vertex_Out pixelVertexPos{ CalculateVertexWithAttributes(verts, w0, w1, w2, shininess, specularKS)};
					m_pColorBuffer[currentPixel] = PixelShading(pixelVertexPos, shininess, specularKS);

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

Vertex_Out Renderer::CalculateVertexWithAttributes(const std::vector<Vertex_Out>& verts, const float w0, const float w1, const float w2, float& outShininess, float& outSpecularKS) const
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
	const ColorRGB currentColor{ m_pTexture->Sample(interpolatedUV) };

	#pragma region normals
	const auto [Nr, Ng, Nb]{ m_pNormals->Sample(interpolatedUV) };
	const Vector3 binormal = Vector3::Cross(interpolatedNormal, interpolatedTangent);
	const Matrix tangentSpaceAxis{ Matrix{ interpolatedTangent,binormal,interpolatedNormal,Vector3::Zero } };

	Vector3 sampledNormal{ Nr,Ng,Nb };
	sampledNormal = 2.f * sampledNormal - Vector3{1.f, 1.f, 1.f};
	sampledNormal = tangentSpaceAxis.TransformVector(sampledNormal).Normalized();
	sampledNormal /= 255.f;
	#pragma endregion

	#pragma region Phong
	//gloss
	const auto [Gr, Gg, Gb]{ m_pGloss->Sample(interpolatedUV) };
	outShininess = Gr * 25;

	//specular
	const auto [Sr, Sg, Sb]{ m_pSpecular->Sample(interpolatedUV) };
	const Vector3 ksValue{ Sr, Sg, Sb };
	outSpecularKS = ksValue.Magnitude();
	#pragma endregion

	return { interpolatedPosition, currentColor, interpolatedUV, m_HasNormalMap ? sampledNormal.Normalized() : interpolatedNormal, interpolatedTangent, viewDirection.Normalized() };
}

ColorRGB Renderer::PixelShading(const Vertex_Out& v, const float shininess, const float specularKS) const
{
	constexpr float kd{ 7.f }; // = diffuse reflectance = diffuse specularity
	float ObservedArea{ Vector3::Dot(v.normal, -lightDirection) };
	ObservedArea = Clamp(ObservedArea, 0.f, 1.f);

	switch (m_LightingMode)
	{
		case LightingMode::ObservedArea: {
			return ColorRGB{ ObservedArea,ObservedArea,ObservedArea };
		}

		case LightingMode::Diffuse:{
			Material_Lambert material{ Material_Lambert(v.color, kd) };
			const ColorRGB diffuse{ material.Shade(v) };
			return diffuse * ObservedArea;
		}

		case LightingMode::Specular: {
			Material_LambertPhong material{ Material_LambertPhong(colors::Black, kd, specularKS, shininess) };
			const ColorRGB diffuse{ material.Shade(v, lightDirection, v.viewDirection) };
			return diffuse;
		}

		case LightingMode::Combined: {
			//Material_Lambert material{ Material_Lambert(v.color, kd) };
			//const ColorRGB diffuse{ material.Shade(v) };
			Material_LambertPhong material{ Material_LambertPhong(v.color, kd, specularKS, shininess) };
			const ColorRGB phong{ material.Shade(v, lightDirection, v.viewDirection) };
			//const ColorRGB phong{ diffuse + Phong(kd, 1, lightDirection, v.viewDirection, v.normal) };
			return phong * ObservedArea;
		}

		default: {
			Material_LambertPhong phong{ Material_LambertPhong(v.color, kd, specularKS, shininess) };
			const ColorRGB lambertPhong{ phong.Shade(v) };
			return lambertPhong * ObservedArea;
		}
	}
}

/// <summary>
/// Render the Mesh with List topology
/// </summary>
/// <param name="mesh">The mesh as const ref</param>
void Renderer::RenderMeshTriangleList(const Mesh& mesh)
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
void Renderer::RenderMeshTriangleStrip(const Mesh& mesh)
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
/// <param name="worldMatrix">Worldmatrix from the mesh</param>
void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex_Out>& vertices_out, const Matrix& worldMatrix) const
{
	vertices_out.resize(vertices_in.size());

	//Add viewmatrix with camera space matrix
	const Matrix worldViewProjectionMatrix{ worldMatrix * m_Camera.viewMatrix * m_Camera.projectionMatrix };

	for (int i{}; i < vertices_in.size(); i++)
	{

		#ifndef W3_AND_UP
		//Transform points to camera space
		Vector3 transformedVert{ m_Camera.viewMatrix.TransformPoint(vertices_in[i].position) };
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

		#ifdef W3_AND_UP
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

		const Vector3 viewDirection{ m_Camera.origin - vertices_in[i].position };

		if (!(projectedVertexX < -1 && projectedVertexX > 1) && !(projectedVertexY < -1 && projectedVertexY > 1)) {
			const Vertex_Out vert{ Vector4{projectedVertexX, projectedVertexY , projectedVertexZ, projectedVertexW}, vertices_in[i].color,  vertices_in[i].uv, normal, tangent, viewDirection };
			vertices_out[i] = vert;
		}
		#endif
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