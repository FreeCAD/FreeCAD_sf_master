/***************************************************************************
 *   Copyright (c) 2024 Shai Seger <shaise at gmail>                       *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include "GuiDisplay.h"
#include "OpenGlWrapper.h"
#include "MillSimulation.h"
#include <cstddef>
#include "GlUtils.h"
#include <stdlib.h>

using namespace MillSim;

static const char* ThumbImg =
	"182005C0C1D5E9F5FC0480FCF5E8D5C109C0D4F60C80F6D307C0E202808AA3B5BC04BFBCB4A38A80FFE105C0E28081A50CBF"
	"A48180E103C0D48081AE0EBFAE81FFD3C0C1F680A410BFA480F6C1D5808A12BF8A80D4E980A312BFA380E8F680B512BFB480"
	"F5FC80BC12BFBC80FC028014BF048014BF048014BF048014BF048014BF048014BF048014BF048014BF048014BF048014BF04"
	"8014BF048014BF0280FC80BC12BFBC80FCF680B512BFB480F5E880A312BFA380E8D5808A11BFBE8A80D4C1F680A410BFA380"
	"F6C1C0D3FF81AE0EBFAE81FFD303C0E18081A40BBFBEA38180E105C0E1FF808AA3B4BC04BFBCB4A38A80FFE107C0D3F60C80"
	"F6D309C0C1D4E8F5FC0480FCF5E8D4C105C0";

static const char* SliderImg =
	"F207C1E2F978807480F9E2C1E278807880E2F980B178BF74BFB180F9028078BF76BF0280F980B178BF74BFB080F9E2788078"
	"80E1C1E2F978807480F9E2C1";

static const char* FasterImg =
	"2120C0D7F6FEF0CB0CC0D6F6FFF0CC0AC0D40480FCCA0AC0D3FF0380FCCB09C0F280A3BC9580F6C409C0F180A1BD9680F7C5"
	"08C0FD80BDBFBD8C80EE09C0FC80BCBFBD8D80EFC107C0028003BFB88580E308C0FE80BE02BFB98680E407C0028004BFB181"
	"80D807C0FE80BE03BFB18180D906C0028005BFA780FDCD06C0FE80BE04BFA880FECF05C0028006BF9A80F9C605C0FE80BE05"
	"BF9B80FAC704C0028006BFBE9080F1C104C0FE80BE05BFBE9080F3C203C0028007BFBB8880E804C0FE80BE06BFBB8880E803"
	"C0028008BFB38280DB03C0FE80BE07BFB58380DC02C0028009BFAB80FED002C0FE80BE08BFAB80FFD2C002800ABF9E80F8C1"
	"C0FE80BE09BF9F80F9C202800BBF8C80DBC0FE80BE0ABF8D80DD02800BBFAB80EFC0FE80BE0ABFAD80F002800BBFBA80FAC0"
	"FE80BE0ABFBB80FC02800BBFB580F5C0FE80BE0ABFB780F702800BBFA180EAC0FE80BE0ABFA380EB02800ABFBC8580D0C0FE"
	"80BE09BFBC8680D202800ABF9480F102C0FE80BE09BF9580F2C0028009BFA180FBC902C0FE80BE08BFA280FCCAC0028008BF"
	"AE80FFD203C0FE80BE07BFAF81FFD402C0028007BFB78580DF04C0FE80BE06BFB88580E003C0028006BFBD8C80EC05C0FE80"
	"BE05BFBD8D80EE04C0028006BF9780F6C305C0FE80BE05BF9880F7C404C0028005BFA380FCCA06C0FE80BE04BFA480FDCC05"
	"C0028004BFB181FFD507C0FE80BE03BFB18180D706C0028003BFB98680E108C0FE80BE02BFB98680E307C0FC80BBBFBE8D80"
	"EE09C0FB80BABFBE8E80EFC107C0EF809DBD9780F7C409C0ED809CBD9880F8C508C0CFFF0380FDCC0AC0CEFE0380FDCD0AC0"
	"D3F4FFF1CD0CC0D2F4FFF2CE0AC0";

static const char* PauseImg =
	"172002C0D3EF02FDEFD307C0D3EF02FDEFD303C0E20680E105C0E20680E1C0D38081A402BBA48180D303C0D38081A402BBA4"
	"8180D3EF80A404BFA480EF03C0EF80A404BFA480EFFC80BB04BFBB80FC03C0FC80BB04BFBB80FC028006BF028003C0028006"
	"BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF04"
	"8006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006"
	"BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF02"
	"8003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003"
	"C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C0028006BF048006BF028003C002"
	"8006BF0280FC80BB04BFBB80FC03C0FC80BB04BFBB80FCEF80A404BFA280EF03C0EF80A404BFA280EFD38081A402BBA281FF"
	"CF03C0D38081A402BBA281FFCFC0E10580FFDE05C0E10580FFDE03C0D3EF02FCEFCF07C0D3EF02FCEFCF02C0";

static const char* PlayImg =
	"1B20C0C1DDF5FDF6DFC213C0C1EF0580F7D212C0DE8087AEBEB08B0280E9C610C0F680B003BFBEA28280FBDA0FC0FE8006BF"
	"B7920280F1CB0DC0028008BFAC8780FEE4C30BC0028009BFBD9D0280F7D20AC002800BBFB38D0280ECC708C002800CBFBEA4"
	"8380FCDB07C002800EBFB9940280F3CD05C0028010BFAC8780FEE4C303C0028011BFBD9D0280F9D402C0028013BFB28C0280"
	"DEC0028014BFBEA281FFD1028016BFA380EF028016BFBC80FC028016BFB880FA028016BF9B80EA028014BFBEA080FECB0280"
	"13BFB08A80FFD9C0028011BFBB9B0280F6D102C0028010BFAA8580FEE2C203C002800EBFB7910280F0CA05C002800CBFBEA2"
	"8280FBD907C002800BBFB08A0280EAC508C0028009BFBB9B0280F6D10AC0028008BFAA8580FEE2C10BC0FE80BE05BFB69002"
	"80EFC90DC0F480AB03BFBEA08280FAD70FC0D98083A7BCAE8A80FFE7C411C0E70580F5CF14C0D6F1FBF6DDC113C0";

static const char* RotateImg =
	"20200AC0C4D5E5F2F9FEFDF8F1E2D2C112C0CAE5FD0A80FADFC50EC0C2E1FE028090A2B1B9BEBDB8AE9E880280F50DC0C8F5"
	"028099B607BFB5960280FEE20CC0CBF9808BB208BFA40280FEE7C80CC0C8FB8091BC09BF9980EEC70DC0C2F38092BE0ABFA5"
	"80E70EC0E38089BC0BBFB180F30DC0C8FE80B307BFB3998902818880FEC10CC0E7809706BFBD980880CA0BC0C2FC80B705BF"
	"BD8F80FDE2CC02C1CCE2E9C20BC0D5808E06BF9880F8CA0CC0C6EDE3C103C0E580A305BFB380FDCA0CC0C7F60280F1C602C0"
	"F080AF05BF9A80E30CC0C7F7808C8780F9CFC0FA80B905BF8980CC0BC0C8F7808DBBBA9080FED8FC80BD05BF8180C10BC0F3"
	"808EBC02BFBE9C80FCFD80BE05BF8180C10AC0C1FF81BC04BFBB80FBF980B805BF8980CC0AC0CE808A05BFB880F8F080AF05"
	"BF9A80E40AC0E5809B05BFAD80F0E580A105BFB480FDCB08C0CDFE80B505BF9F80E2D3808D06BF9A80F9CB06C0CDF9809C06"
	"BF8C80D3C2FC80B505BFBD9080FEE4CEC2C4CEE5FE8092BE05BFB280FAC1C0E5809606BFBD9A08809CBE06BF9580E202C0C6"
	"FD80B207BFB59B8B82848B9BB507BFAF80FCC603C0E18088BB14BFBA8880DD04C0C1F28091BD12BFBD8E80F2C105C0C8FA80"
	"90BC10BFBA8E80F8C507C0CAF98088B10EBFAF8880F8CA09C0C8F3028097B40ABFB4960280F2C60BC0C1E0FE02808EA0AFB7"
	"BEBBB8AEA08D0280FDDEC10EC0C7E4FD0A80FBE3C712C0C2D3E5F0F8FEFBF8F0E3D4C20AC0";

static const char* SingleStepImg =
	"1D2002C0D4F0FDFCEECE06C0D0F2FBF0CD0BC0E20580FFDD04C0CCFE0380FDCD09C0D48082A5BCBBA081FFCF03C0E9809ABC"
	"9980F9C608C0F080A504BFA180EB03C0F480B5BFBE9080F1C107C0FC80BC04BFB880F803C0F780B802BFBB8880E807C08081"
	"05BFBC80FB03C0F780B803BFB48280DA06C0808105BFBC80FB03C0F780B804BFAB80FED005C0808105BFBC80FB03C0F780B8"
	"05BF9E80FBC804C0808105BFBC80FB03C0F780B806BF9380F3C203C0808105BFBC80FB03C0F780B806BFBC8A80EB03C08081"
	"05BFBC80FB03C0F780B807BFB68380DD02C0808105BFBC80FB03C0F780B808BFAE80FFD2C0808105BFBC80FB03C0F780B809"
	"BFA080F9C2808105BFBC80FB03C0F780B80ABF8E80DD808105BFBC80FB03C0F780B80ABFAE80F0808105BFBC80FB03C0F780"
	"B80ABFBC80FC808105BFBC80FB03C0F780B80ABFB880F7808105BFBC80FB03C0F780B80ABFA480EC808105BFBC80FB03C0F7"
	"80B809BFBD8780D2808105BFBC80FB03C0F780B809BF9680F3C0808105BFBC80FB03C0F780B808BFA480FCCAC0808105BFBC"
	"80FB03C0F780B807BFB181FFD502C0808105BFBC80FB03C0F780B806BFB98680E103C0808105BFBC80FB03C0F780B805BFBE"
	"8F80EF04C0808105BFBC80FB03C0F780B805BF9A80F8C504C0808105BFBC80FB03C0F780B804BFA880FDCC05C0808105BFBC"
	"80FB03C0F780B803BFB48280D806C0FC80BC04BFB880F803C0F780B802BFBB8880E507C0F080A504BFA080EB03C0F380B302"
	"BF9180F1C107C0D380FFA302BBA081FFCF03C0E58095BB9B80F9C609C0E0FF0480FEDD04C0C8FC0380FECF0BC0D0EFFDFCEE"
	"CD06C0CCF0FBF1CF0AC0";

GuiItem guiItems[] = {
	{ SliderImg, 0, 0, 1, 36, 0, 0, 400, 554, 0 },
	{ ThumbImg, 0, 0, 1, 1, 0, 0, 488, 540, 0 },
	{ PauseImg, 0, 0, 70, 1, 0, 0, 210, 540, 'P', true },
	{ PlayImg, 0, 0, 100, 1, 0, 0, 210, 540, 'S', false },
	{ SingleStepImg, 0, 0, 134, 1, 0, 0, 250, 540, 'T' },
	{ FasterImg, 0, 0, 172, 1, 0, 0, 290, 540, 'F' },
	{ RotateImg, 0, 0, 210, 1, 0, 0, 330, 540, ' ' },
};

#define NUM_GUI_ITEMS (sizeof(guiItems) / sizeof(GuiItem))
#define TEX_SIZE 256

// parse compressed image into a texture buffer
bool GuiDisplay::ParseImage(GuiItem *guiItem, unsigned int* buffPos, int stride)
{
	mPixPos = guiItem->imageData;
	int width = ReadNextVal();
	if (width < 0)
		return false;
	int height = ReadNextVal();
	if (height < 0)
		return false;
	int buffLen = width * height;
	buffPos += stride * guiItem->ty + guiItem->tx;
	int x = 0;
	unsigned int pixVal;
	int amount;
	while (ReadNextPixel(&pixVal, &amount))
	{
		while (x + amount > width)
		{
			int len = width - x;
			for (int i = x; i < width; i++)
				buffPos[i] = pixVal;
			amount -= len;
			x = 0;
			buffPos += stride;
		}
		int end = x + amount;
		for (; x < end; x++)
			buffPos[x] = pixVal;
		if (x >= width)
		{
			x = 0;
			buffPos += stride;
		}
	}
	guiItem->w = width;
	guiItem->h = height;
	return true;
}

bool GuiDisplay::GenerateGlItem(GuiItem* guiItem)
{
	Vertex2D verts[4];
	int x = guiItem->tx;
	int y = guiItem->ty;
	int w = guiItem->w;
	int h = guiItem->h;

	verts[0] = { 0, (float)h, mTexture.getTexX(x), mTexture.getTexY(y + h) };
	verts[1] = { (float)w, (float)h,  mTexture.getTexX(x + w), mTexture.getTexY(y + h) };
	verts[2] = { 0, 0, mTexture.getTexX(x), mTexture.getTexY(y) };
	verts[3] = { (float)w, 0,  mTexture.getTexX(x + w), mTexture.getTexY(y) };

	// vertex buffer
	glGenBuffers(1, &(guiItem->vbo));
	glBindBuffer(GL_ARRAY_BUFFER, guiItem->vbo);
	glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(Vertex2D), verts, GL_STATIC_DRAW);

	//glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_SHORT, nullptr);
	// vertex array
	glGenVertexArrays(1, &(guiItem->vao));
	glBindVertexArray(guiItem->vao);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex2D), (void*)offsetof(Vertex2D, x));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex2D), (void*)offsetof(Vertex2D, tx));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIbo);
	glBindVertexArray(0);

	return true;
}

bool GuiDisplay::InutGui()
{
	// index buffer		
	glGenBuffers(1, &mIbo);
	GLshort indices[6] = { 0, 2, 3, 0, 3, 1 };
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLushort), indices, GL_STATIC_DRAW);

	int buffsize = TEX_SIZE * TEX_SIZE * sizeof(unsigned int);
	unsigned int* buffer = (unsigned int*)malloc(buffsize);
	if (buffer == nullptr)
		return false;
	memset(buffer, 0, buffsize);
	for (int i = 0; i < NUM_GUI_ITEMS; i++)
		ParseImage(&(guiItems[i]), buffer, TEX_SIZE);
	mTexture.LoadImage(buffer, TEX_SIZE, TEX_SIZE);
	free(buffer);
	for (int i = 0; i < NUM_GUI_ITEMS; i++)
		GenerateGlItem(&(guiItems[i]));

	mThumbStartX = guiItems[eGuiItemSlider].sx - guiItems[eGuiItemThumb].w / 2;
	mThumbMaxMotion = (float)guiItems[eGuiItemSlider].w;

	// shader
	mat4x4 projmat;
	//mat4x4 viewmat;
	mat4x4_ortho(projmat, 0, 800, 600, 0, -1, 1);
	mShader.CompileShader((char*)VertShader2DTex, (char*)FragShader2dTex);
	mShader.UpdateTextureSlot(0);
	mShader.UpdateProjectionMat(projmat);
	return true;
}

void GuiDisplay::RenderItem(int itemId)
{
	GuiItem* item = &(guiItems[itemId]);
	if (item->hidden)
		return;
	mat4x4 model;
	mat4x4_translate(model, (float)item->sx, (float)item->sy, 0);
	mShader.UpdateModelMat(model, nullptr);
	if (itemId == mPressedItem)
		mShader.UpdateObjColor(mPressedColor);
	else if (item->mouseOver)
		mShader.UpdateObjColor(mHighlightColor);
	else
		mShader.UpdateObjColor(mStdColor);

	glBindVertexArray(item->vao);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
}

void GuiDisplay::MouseCursorPos(int x, int y)
{
	for (int i = 1; i < NUM_GUI_ITEMS; i++)
	{
		GuiItem* g = &(guiItems[i]);
		g->mouseOver = (x > g->sx && y > g->sy && x < (g->sx + g->w) && y < (g->sy + g->h));
	}

}

void GuiDisplay::MousePressed(int button, bool isPressed, bool isSimRunning)
{
	if (button == MS_MOUSE_LEFT)
	{
		if (isPressed)
		{
			mPressedItem = eGuiItemMax;
			for (int i = 1; i < NUM_GUI_ITEMS; i++)
			{
				GuiItem* g = &(guiItems[i]);
				if (g->mouseOver && !g->hidden)
				{
					mPressedItem = (eGuiItems)i;
					break;
				}
			}
			if (mPressedItem != eGuiItemMax)
			{
				GuiItem* g = &(guiItems[mPressedItem]);
				if (g->actionKey != 0)
					mMillSim->HandleKeyPress(g->actionKey);
			}
		}
		else // button released
		{
			UpdatePlayState(isSimRunning);
			mPressedItem = eGuiItemMax;
		}
	}
}

void GuiDisplay::MouseDrag(int buttons, int dx, int dy)
{
	if (mPressedItem == eGuiItemThumb)
	{
		GuiItem* g = &(guiItems[eGuiItemThumb]);
		int newx = g->sx + dx;
		if (newx < mThumbStartX)
			newx = mThumbStartX;
		if (newx > ((int)mThumbMaxMotion + mThumbStartX))
			newx = (int)mThumbMaxMotion + mThumbStartX;
		if (newx != g->sx)
		{
			mMillSim->SetSimulationStage((float)(newx - mThumbStartX) / mThumbMaxMotion);
			g->sx = newx;
		}
	}
}

void GuiDisplay::UpdatePlayState(bool isRunning)
{
	guiItems[eGuiItemPause].hidden = !isRunning;
	guiItems[eGuiItemPlay].hidden = isRunning;
}

void GuiDisplay::Render(float progress)
{
	if (mPressedItem != eGuiItemThumb)
		guiItems[eGuiItemThumb].sx = (int)(mThumbMaxMotion * progress) + mThumbStartX;
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	mTexture.Activate();
	mShader.Activate();
	mShader.UpdateTextureSlot(0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	for (int i = 0; i < NUM_GUI_ITEMS; i++)
		RenderItem(i);

	//mat4x4 model;
	//mat4x4_translate(model, 100, 100, 0);
	//mShader.UpdateModelMat(model, nullptr);
	//glBindVertexArray(guiItems[0].vao);
	//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
}

int GuiDisplay::ReadNextVal()
{
	int val = 0;
	for (int i = 0; i < 2; i++)
	{
		val *= 16;
		int c = *mPixPos++;
		if (c == 0)
			return -1;
		if (c <= '9')
			val += c - '0';
		else
			val += c - 'A' + 10;
	}
	return val;
}

bool GuiDisplay::ReadNextPixel(unsigned int* pix, int* amount)
{
	if (*mPixPos == 0)
		return false;

	*amount = 1;
	int val = ReadNextVal();
	if (val < 0)
		return false;
	if (val < 128)
	{
		*amount = val;
		val = ReadNextVal();
		if (val < 128)
			return false;
	}
	if (val < 192)
	{
		val = (val - 128) * 4;
		*pix = 0xFF000000 | (val << 16) | (val << 8) | val;
	}
	else
	{
		val = (val - 192) * 4;
		*pix = val<<24;
	}

	return true;
}
