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
# include <Inventor/elements/SoLineWidthElement.h>
# include <Inventor/elements/SoPointSizeElement.h>
# include <Inventor/errors/SoDebugError.h>
# include <Inventor/errors/SoReadError.h>
# include <Inventor/details/SoFaceDetail.h>
# include <Inventor/details/SoLineDetail.h>
# include <Inventor/misc/SoState.h>
#endif

#include <Inventor/elements/SoLightModelElement.h>
#include <Inventor/elements/SoCacheElement.h>
#include <Inventor/elements/SoShapeStyleElement.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/elements/SoCullElement.h>
#include <Inventor/caches/SoBoundingBoxCache.h>

#include "SoBrepPointSet.h"
#include <Gui/SoFCUnifiedSelection.h>
#include <Gui/SoFCSelectionAction.h>
#include <Gui/ViewParams.h>

using namespace PartGui;

SO_NODE_SOURCE(SoBrepPointSet)

void SoBrepPointSet::initClass()
{
    SO_NODE_INIT_CLASS(SoBrepPointSet, SoPointSet, "PointSet");
}

SoBrepPointSet::SoBrepPointSet()
    :selContext(std::make_shared<SelContext>())
    ,selContext2(std::make_shared<SelContext>())
{
    SO_NODE_CONSTRUCTOR(SoBrepPointSet);
    SO_NODE_ADD_FIELD(highlightIndices, (-1));
    highlightIndices.setNum(0);
}

bool SoBrepPointSet::isSelected(SelContextPtr ctx) {
    if(ctx) 
        return ctx->isSelected();
    for(auto node : siblings) {
        auto sctx = Gui::SoFCSelectionRoot::getRenderContext<Gui::SoFCSelectionContext>(node);
        if(sctx && sctx->isSelected())
            return true;
    }
    return false;
}

void SoBrepPointSet::setSiblings(std::vector<SoNode*> &&s) {
    // No need to ref() here, because we only use the pointer as keys to lookup
    // selection context
    siblings = std::move(s);
}

void SoBrepPointSet::GLRender(SoGLRenderAction *action)
{
    auto state = action->getState();

    const SoCoordinateElement* coords = SoCoordinateElement::getInstance(state);
    int num = coords->getNum() - this->startIndex.getValue();
    if (num < 0) {
        // Fixes: #0000545: Undo revolve causes crash 'illegal storage'
        return;
    }

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
    if(ctx2 && ctx2->selectionIndex.empty())
        return;

    if(selContext2->checkGlobal(ctx)) {
        SoCacheElement::invalidate(state);
        ctx = selContext2;
    }

    Gui::FCDepthFunc depthGuard;
    if(!action->isRenderingDelayedPaths())
        depthGuard.set(GL_LEQUAL);

    if(!highlightIndices.getNum() && ctx && ctx->isHighlightAll()) {
        if(ctx2) {
            ctx2->selectionColor = ctx->highlightColor;
            renderSelection(action,ctx2); 
        } else
            renderHighlight(action,ctx);
        return;
    }

    if(Gui::ViewParams::instance()->getShowSelectionOnTop()
            && !Gui::SoFCUnifiedSelection::getShowSelectionBoundingBox()
            && !action->isRenderingDelayedPaths()
            && !ctx2 
            && isSelected(ctx))
    {
        // If 'ShowSelectionOnTop' is enabled, and we are not rendering on top
        // (!isRenderingDelayedPaths) and we are not doing partial rendering
        // (!ctx2), and we are selected, just skip render to avoid duplicate
        // rendering in group on top.
        return;
    }

    if(ctx && ctx->selectionIndex.size()) {
        if(!highlightIndices.getNum() && ctx->isSelectAll() && ctx->hasSelectionColor()) {
            if(ctx2 && ctx2->selectionIndex.size()) {
                ctx2->selectionColor = ctx->selectionColor;
                renderSelection(action,ctx2); 
            }else
                renderSelection(action,ctx); 

            renderHighlight(action,ctx);
            return;
        }
    }

    if(ctx2 && ctx2->isSelected())
        renderSelection(action,ctx2,false);
    else
        inherited::GLRender(action);

    if(ctx && ctx->selectionIndex.size() && ctx->hasSelectionColor())
        renderSelection(action,ctx);

    renderHighlight(action,ctx);
}

void SoBrepPointSet::GLRenderBelowPath(SoGLRenderAction * action)
{
    inherited::GLRenderBelowPath(action);
}

void SoBrepPointSet::getBoundingBox(SoGetBoundingBoxAction * action) {
    auto state = action->getState();
    selCounter.checkCache(state, true);

    SelContextPtr ctx2 = Gui::SoFCSelectionRoot::getSecondaryActionContext<SelContext>(action,this);
    if(!ctx2 || ctx2->isSelectAll()) {
        inherited::getBoundingBox(action);
        return;
    }

    if(ctx2->selectionIndex.empty())
        return;

    auto coords = SoCoordinateElement::getInstance(state);
    const SbVec3f *coords3d = coords->getArrayPtr3();
    int numverts = coords->getNum();
    int startIndex = this->startIndex.getValue();

    SbBox3f bbox;
    for(auto &v : ctx2->selectionIndex) {
        int idx = v.first;
        if(idx >= startIndex && idx < numverts)
            bbox.extendBy(coords3d[idx]);
    }

    if(!bbox.isEmpty())
        action->extendBy(bbox);
}

static inline void setupRendering(
        SoState *state, SoNode *node, const uint32_t *color) 
{
    float ps = SoPointSizeElement::get(state);
    if (ps < 4.0f) SoPointSizeElement::set(state, node, 4.0f);

    SoLightModelElement::set(state,SoLightModelElement::BASE_COLOR);
    SoMaterialBindingElement::set(state,SoMaterialBindingElement::OVERALL);
    SoLazyElement::setPacked(state, node,1, color, false);
}

static FC_COIN_THREAD_LOCAL std::vector<int> RenderIndices;

void SoBrepPointSet::renderHighlight(SoGLRenderAction *action, SelContextPtr ctx)
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

    _renderSelection(action, ctx->highlightColor, true);
}

void SoBrepPointSet::renderSelection(SoGLRenderAction *action, SelContextPtr ctx, bool push)
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
    _renderSelection(action, ctx->selectionColor, push);
}

void SoBrepPointSet::_renderSelection(SoGLRenderAction *action, const SbColor &selectionColor, bool push)
{
    SoState * state = action->getState();
    uint32_t color;
    if(push) {
        state->push();
        color = selectionColor.getPackedValue(0.0);
        setupRendering(state,this,&color);
    }

    const SoCoordinateElement * coords;
    const SbVec3f * normals;

    this->getVertexData(state, coords, normals, false);

    SoMaterialBundle mb(action);
    mb.sendFirst(); // make sure we have the correct material

    int startIndex = this->startIndex.getValue();
    const SbVec3f * coords3d = coords->getArrayPtr3();

    if(coords3d) {
        glBegin(GL_POINTS);
        if(RenderIndices.empty()) {
            for(int idx=startIndex;idx<coords->getNum();++idx)
                glVertex3fv((const GLfloat*) (coords3d + idx));
        } else {
            for(int idx : RenderIndices) {
                if(idx >= startIndex && idx < coords->getNum())
                    glVertex3fv((const GLfloat*) (coords3d + idx));
            }
        }
        glEnd();
    }
    if(push) state->pop();
}

void SoBrepPointSet::doAction(SoAction* action)
{
    if (Gui::SoFCSelectionRoot::handleSelectionAction(
                action, this, Gui::SoFCDetail::Vertex, selContext, selCounter))
        return;

    inherited::doAction(action);
}

