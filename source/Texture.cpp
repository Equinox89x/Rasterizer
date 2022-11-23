#include "Texture.h"
#include "Vector2.h"
#include <SDL_image.h>
#include <iostream>

namespace dae
{
	Texture::Texture(SDL_Surface* pSurface) :
		m_pSurface{ pSurface },
		m_pSurfacePixels{ (uint32_t*)pSurface->pixels }
	{
	}

	Texture::~Texture()
	{
		if (m_pSurface)
		{
			SDL_FreeSurface(m_pSurface);
			m_pSurface = nullptr;
		}
	}

	Texture* Texture::LoadFromFile(const std::string& path)
	{
		//Load SDL_Surface using IMG_LOAD
		SDL_Surface* loadedSurface = IMG_Load(path.c_str());
		
		//Create & Return a new Texture Object (using SDL_Surface)
		return new Texture{ loadedSurface };
	}

	ColorRGB Texture::Sample(const Vector2& uv) const
	{
		SDL_Color rgb{};

		//convert from 0,1 to 0,width;
		const int x{ static_cast<int>(uv.x * m_pSurface->w) };
		const int y{ static_cast<int>(uv.y * m_pSurface->h) };

		//Sample the correct data for the given uv
		const Uint32 p = m_pSurfacePixels[x + y * m_pSurface->w];
		SDL_GetRGB(p, m_pSurface->format, &rgb.r, &rgb.g, &rgb.b);
		
		//change color from range 0,255 to 0,1
		ColorRGB rgb2{ rgb.r, rgb.g, rgb.b};
		return rgb2/255;
	}
}