/***************************************************************************
 *   Copyright (c) 2011 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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


#include "PreCompiled.h"

#ifndef _PreComp_
# ifdef FC_OS_WIN32
#  include <windows.h>
# endif
# ifdef FC_OS_MACOSX
#  include <OpenGL/gl.h>
# else
#  include <GL/gl.h>
# endif
# include <float.h>
# include <algorithm>
# include <Python.h>
# include <Inventor/SoPickedPoint.h>
# include <Inventor/SoPrimitiveVertex.h>
# include <Inventor/actions/SoCallbackAction.h>
# include <Inventor/actions/SoGetBoundingBoxAction.h>
# include <Inventor/actions/SoGetPrimitiveCountAction.h>
# include <Inventor/actions/SoGLRenderAction.h>
# include <Inventor/actions/SoPickAction.h>
# include <Inventor/actions/SoWriteAction.h>
# include <Inventor/bundles/SoMaterialBundle.h>
# include <Inventor/bundles/SoTextureCoordinateBundle.h>
# include <Inventor/elements/SoOverrideElement.h>
# include <Inventor/elements/SoCoordinateElement.h>
# include <Inventor/elements/SoGLCoordinateElement.h>
# include <Inventor/elements/SoGLCacheContextElement.h>
# include <Inventor/elements/SoPointSizeElement.h>
# include <Inventor/errors/SoDebugError.h>
# include <Inventor/errors/SoReadError.h>
# include <Inventor/details/SoFaceDetail.h>
# include <Inventor/details/SoLineDetail.h>
# include <Inventor/misc/SoState.h>
#endif

#include <Inventor/elements/SoCacheElement.h>
#include <Inventor/elements/SoLineWidthElement.h>
#include <Inventor/elements/SoLinePatternElement.h>
#include <Inventor/elements/SoLightModelElement.h>
#include <Inventor/elements/SoShapeStyleElement.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/elements/SoCullElement.h>
#include <Inventor/caches/SoBoundingBoxCache.h>

#include "SoBrepEdgeSet.h"
#include <Gui/SoFCUnifiedSelection.h>
#include <Gui/SoFCSelectionAction.h>
#include <Gui/ViewParams.h>

using namespace PartGui;

SO_NODE_SOURCE(SoBrepEdgeSet)

void SoBrepEdgeSet::initClass()
{
    SO_NODE_INIT_CLASS(SoBrepEdgeSet, SoIndexedLineSet, "IndexedLineSet");
}

SoBrepEdgeSet::SoBrepEdgeSet()
    :selContext(std::make_shared<SelContext>())
    ,selContext2(std::make_shared<SelContext>())
{
    SO_NODE_CONSTRUCTOR(SoBrepEdgeSet);
    SO_NODE_ADD_FIELD(highlightIndices, (-1));
    highlightIndices.setNum(0);
}

void SoBrepEdgeSet::notify(SoNotList * list)
{
    SoField *f = list->getLastField();
    if (f == &this->coordIndex) {
        const int32_t* cindices = this->coordIndex.getValues(0);
        int numcindices = this->coordIndex.getNum();
        this->segments.clear();
        for(int i=0;i<numcindices;i++) {
            if(cindices[i] < 0)
                this->segments.push_back(i);
        }
    }
    SoIndexedLineSet::notify(list);
}

bool SoBrepEdgeSet::isSelected(SelContextPtr ctx) {
    if(ctx) 
        return ctx->isSelected();
    for(auto node : siblings) {
        auto sctx = Gui::SoFCSelectionRoot::getRenderContext<Gui::SoFCSelectionContext>(node);
        if(sctx && sctx->isSelected())
            return true;
    }
    return false;
}

void SoBrepEdgeSet::setSiblings(std::vector<SoNode*> &&s) {
    // No need to ref() here, because we only use the pointer as keys to lookup
    // selection context
    siblings = std::move(s);
}

void SoBrepEdgeSet::GLRender(SoGLRenderAction *action) {

    auto state = action->getState();

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Copied from SoShape::shouldGLRender(). Put here for early render skipping
    // TODO: check SoShape::shouldGLRender() code in case we want to render shadow
    const SoShapeStyleElement * shapestyle = SoShapeStyleElement::get(state);
    unsigned int shapestyleflags = shapestyle->getFlags();
    if (shapestyleflags & SoShapeStyleElement::INVISIBLE)
        return;
    if (getBoundingBoxCache() && !state->isCacheOpen() && !SoCullElement::completelyInside(state)) {
        if (getBoundingBoxCache()->isValid(state)) {
            if (SoCullElement::cullTest(state, getBoundingBoxCache()->getProjectedBox())) {
                return;
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////

    selCounter.checkCache(state);

    SelContextPtr ctx2;
    SelContextPtr ctx = Gui::SoFCSelectionRoot::getRenderContext<SelContext>(this,selContext,ctx2);
    if(ctx2 && !ctx2->isSelected())
        return;

    if(selContext2->checkGlobal(ctx)) {
        SoCacheElement::invalidate(state);
        ctx = selContext2;
    }

    Gui::FCDepthFunc depthGuard;
    if(!action->isRenderingDelayedPaths())
        depthGuard.set(GL_LEQUAL);

    if(ctx && ctx->isHighlightAll() && !highlightIndices.getNum()) {
        if(ctx2 && !ctx2->isSelectAll()) {
            ctx2->selectionColor = ctx->highlightColor;
            renderSelection(action,ctx2); 
        } else
            renderHighlight(action,ctx);
        return;
    }

    int pass = 2;

    if((!ctx2 || !ctx2->isSelectAll())
       && Gui::ViewParams::instance()->getShowSelectionOnTop()
       && (!ctx || !ctx->isSelectAll() || highlightIndices.getNum())
       && !Gui::SoFCUnifiedSelection::getShowSelectionBoundingBox()) 
    {
        // If we are rendering on top, we shall perform a two pass rendering.
        // The first pass keep depth test disabled (default in on top
        // rendering), and default transparency override, with an optional
        // selection line pattern (default 0xff00). This pass is for rendering
        // hidden lines.
        //
        // The second pass renables depth test, and set depth function to
        // GL_LEQUAL to render the outline.
        if(action->isRenderingDelayedPaths())
            pass = 0;
        else if (isSelected(ctx)) {
            // If we are selected but not rendering inside the group on top.
            // Just skip the rendering
            return;
        }
    }

    SoColorPacker packer;
    float trans = 0.0;
    int oldPattern = 0;
    float oldWidth = 0.0;

    for(;pass<=2;++pass) {
        if(pass==0) {
            int pattern = Gui::ViewParams::instance()->getSelectionLinePattern();
            if(pattern) {
                oldPattern = SoLinePatternElement::get(state);
                SoLinePatternElement::set(state, pattern);
            }
            float width = Gui::ViewParams::instance()->getSelectionHiddenLineWidth();
            if(width>0.0) {
                oldWidth = SoLineWidthElement::get(state);
                SoLineWidthElement::set(state,width);
            }
        } else if(pass==1) {
            depthGuard.set(GL_LEQUAL);
            if(oldPattern)
                SoLinePatternElement::set(state, oldPattern);
            if(oldWidth>0.0)
                SoLineWidthElement::set(state, oldWidth);
            if(!Gui::SoFCSwitch::testTraverseState(Gui::SoFCSwitch::TraverseInvisible)) {
                // If we are visible, disable transparency to get a solid
                // outline, or else on top rendering will have some default
                // transprency, which will give a fainted appearance that is
                // ideal of drawing hidden line, or indicating we are invisible
                // (but forced to shown by on top rendering)
                SoLazyElement::setTransparency(state,this,1,&trans,&packer);
            }
            pass = 2;
        }

        if(ctx && ctx->isSelected()) {
            if(!highlightIndices.getNum() && ctx->isSelectAll() && ctx->hasSelectionColor()) {
                if(ctx2 && !ctx2->isSelectAll()) {
                    ctx2->selectionColor = ctx->selectionColor;
                    renderSelection(action,ctx2); 
                }else if(ctx->isSelectAll())
                    renderSelection(action,ctx); 
                renderHighlight(action,ctx);
                continue;
            }
        }
        if(ctx2 && ctx2->isSelected())
            renderSelection(action,ctx2,false);
        else
            inherited::GLRender(action);

        if(ctx && ctx->isSelected() && ctx->hasSelectionColor())
            renderSelection(action,ctx);
        renderHighlight(action,ctx);
    }
}

void SoBrepEdgeSet::GLRenderBelowPath(SoGLRenderAction * action)
{
    inherited::GLRenderBelowPath(action);
}

void SoBrepEdgeSet::getBoundingBox(SoGetBoundingBoxAction * action) {

    auto state = action->getState();
    selCounter.checkCache(state,true);

    SelContextPtr ctx2 = Gui::SoFCSelectionRoot::getSecondaryActionContext<SelContext>(action,this);
    if(!ctx2 || ctx2->isSelectAll()) {
        inherited::getBoundingBox(action);
        return;
    }

    if(!ctx2->isSelected())
        return;

    auto coords = SoCoordinateElement::getInstance(state);
    const SbVec3f *coords3d = coords->getArrayPtr3();

    const int32_t* indices = this->coordIndex.getValues(0);
    int numindices = this->coordIndex.getNum();

    SbBox3f bbox;

    for(auto &v : ctx2->selectionIndex) {
        int idx = v.first;
        if(idx < 0 || idx >= (int)segments.size())
            break;
        int offset = idx==0 ? 0 : segments[idx-1];
        int num = segments[idx];
        if(num >= numindices)
            break;
        int32_t i;
        const int32_t *cindices = indices + offset;
        const int32_t *end = indices + num;
        while (cindices < end) {
            bbox.extendBy(coords3d[*cindices++]);
            i = (cindices < end) ? *cindices++ : -1;
            while (i >= 0) {
                bbox.extendBy(coords3d[i]);
                i = cindices < end ? *cindices++ : -1;
            }
        }
    }

    if(!bbox.isEmpty())
        action->extendBy(bbox);
}

static inline void renderLines(const SoGLCoordinateElement * const coords,
                               const int32_t *cindices, int numindices)
{

    const SbVec3f * coords3d = coords->getArrayPtr3();
    int num = coords->getNum();

    int32_t i;
    int previ;
    const int32_t *end = cindices + numindices;
    while (cindices < end) {
        previ = *cindices++;
        if(previ < 0 || previ >= num)
            continue;
        i = (cindices < end) ? *cindices++ : -1;
        glBegin(GL_LINE_STRIP);
        while (i >= 0) {
            if(i >= num)
                break;
            glVertex3fv((const GLfloat*) (coords3d + previ));
            glVertex3fv((const GLfloat*) (coords3d + i));
            previ = i;
            i = cindices < end ? *cindices++ : -1;
        }
        glEnd();
    }
}

static FC_COIN_THREAD_LOCAL std::vector<int> RenderIndices;

void SoBrepEdgeSet::renderHighlight(SoGLRenderAction *action, SelContextPtr ctx)
{
    if(!ctx || !ctx->isHighlighted())
        return;

    RenderIndices.clear();
    if(ctx->isHighlightAll()) {
        if(highlightIndices.getNum()) {
            auto indices = highlightIndices.getValues(0);
            RenderIndices.insert(RenderIndices.end(), indices, indices + highlightIndices.getNum());
        }
    } else
        RenderIndices.insert(RenderIndices.end(), ctx->highlightIndex.begin(), ctx->highlightIndex.end());

    _renderSelection(action, ctx->highlightColor, 0xFFFF, true);
}

void SoBrepEdgeSet::renderSelection(SoGLRenderAction *action, SelContextPtr ctx, bool push)
{
    if(!ctx || !ctx->isSelected())
        return;
    RenderIndices.clear();
    if(!ctx->isSelectAll()) {
        for(auto &v : ctx->selectionIndex)
            RenderIndices.push_back(v.first);
    } else if(highlightIndices.getNum()) {
        auto indices = highlightIndices.getValues(0);
        RenderIndices.insert(RenderIndices.end(), indices, indices + highlightIndices.getNum());
    }
    _renderSelection(action, ctx->selectionColor, 0, push);
}

void SoBrepEdgeSet::_renderSelection(SoGLRenderAction *action, 
        const SbColor &selectionColor, unsigned pattern, bool push)
{
    SoState * state = action->getState();
    uint32_t color;
    if(push){
        state->push();
        color = selectionColor.getPackedValue(0.0);
        Gui::SoFCSelectionRoot::setupSelectionLineRendering(state,this,&color);
        if(pattern)
            SoLinePatternElement::set(state, this, pattern);
    }

    const SoCoordinateElement * coords;
    const SbVec3f * normals;
    const int32_t * cindices;
    int numcindices;
    const int32_t * nindices;
    const int32_t * tindices;
    const int32_t * mindices;
    SbBool normalCacheUsed;

    this->getVertexData(state, coords, normals, cindices, nindices,
        tindices, mindices, numcindices, false, normalCacheUsed);

    SoMaterialBundle mb(action);
    mb.sendFirst(); // make sure we have the correct material

    if(RenderIndices.empty())
        renderLines(static_cast<const SoGLCoordinateElement*>(coords), cindices, numcindices);
    else {
        for(int idx : RenderIndices) {
            if (idx < 0 || idx >= (int)segments.size() || segments[idx] >= numcindices)
                break;

            int offset = idx==0 ? 0 : segments[idx-1];
            int num = segments[idx] - offset;
            renderLines(static_cast<const SoGLCoordinateElement*>(coords), cindices + offset, num);
        }
    }
    if(push) state->pop();
}

void SoBrepEdgeSet::doAction(SoAction* action)
{
    if (Gui::SoFCSelectionRoot::handleSelectionAction(
                action, this, Gui::SoFCDetail::Edge, selContext, selCounter))
        return;

    inherited::doAction(action);
}

SoDetail * SoBrepEdgeSet::createLineSegmentDetail(SoRayPickAction * action,
                                                  const SoPrimitiveVertex * v1,
                                                  const SoPrimitiveVertex * v2,
                                                  SoPickedPoint * pp)
{
    SoDetail* detail = inherited::createLineSegmentDetail(action, v1, v2, pp);
    SoLineDetail* line_detail = static_cast<SoLineDetail*>(detail);
    int index = line_detail->getLineIndex();
    line_detail->setPartIndex(index);
    return detail;
}

