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

#include "PreCompiled.h"

#include "DlgCAMSimulator.h"
#include "MillSimulation.h"
#include <QtGui/QMatrix4x4>
#include <QtGui/qscreen.h>
#include <QDateTime>
#include <QSurfaceFormat>
#include <QMouseEvent>
#include <QPoint>

using namespace CAMSimulator;
using namespace MillSim;

QOpenGLContext *gOpenGlContext;

using namespace MillSim;

namespace CAMSimulator
{

    DlgCAMSimulator::DlgCAMSimulator(QWindow* parent)
        : QWindow(parent)
    {
        setSurfaceType(QWindow::OpenGLSurface);
        mMillSimulator = new MillSimulation();
    }

    void DlgCAMSimulator::render(QPainter* painter)
    {
        Q_UNUSED(painter);
    }

    void DlgCAMSimulator::render()
    {
        mMillSimulator->ProcessSim((unsigned int)(QDateTime::currentMSecsSinceEpoch()));
    }

    void DlgCAMSimulator::renderLater()
    {
        requestUpdate();
    }

    bool DlgCAMSimulator::event(QEvent* event)
    {
        switch (event->type()) {
            case QEvent::UpdateRequest:
                renderNow();
                return true;
            default:
                return QWindow::event(event);
        }
    }

    void DlgCAMSimulator::exposeEvent(QExposeEvent* event)
    {
        Q_UNUSED(event);

        if (isExposed()) {
            renderNow();
        }
    }

    void DlgCAMSimulator::mouseMoveEvent(QMouseEvent* ev)
    {
        mMillSimulator->MouseMove(ev->x(), ev->y());
    }

    void DlgCAMSimulator::mousePressEvent(QMouseEvent* ev)
    {
        mMillSimulator->MousePress(ev->button(), true, ev->x(), ev->y());
    }

    void DlgCAMSimulator::mouseReleaseEvent(QMouseEvent* ev)
    {
        mMillSimulator->MousePress(ev->button(), false, ev->x(), ev->y());
    }

    void DlgCAMSimulator::ResetSimulation()
    {
        mMillSimulator->Clear();
    }

    void DlgCAMSimulator::AddGcodeCommand(const char* cmd)
    {
        mMillSimulator->AddGcodeLine(cmd);
    }

    void DlgCAMSimulator::AddTool(const float* toolProfilePoints, int numPoints, int toolNumber, float diameter, float resolution)
    {
        std::string toolCmd = "T" + std::to_string(toolNumber);
        mMillSimulator->AddGcodeLine(toolCmd.c_str());
        if (!mMillSimulator->ToolExists(toolNumber))
            mMillSimulator->AddTool(toolProfilePoints, numPoints, toolNumber, diameter, 16);
    }

    void DlgCAMSimulator::hideEvent(QHideEvent* ev)
    {
        mAnimating = false;
    }

    void DlgCAMSimulator::StartSimulation(const cStock* stock)
    {
        mStock = *stock;
        mNeedsInitialize = true;
        show();
        setAnimating(true);
    }

    void DlgCAMSimulator::initialize()
    {
        mMillSimulator->InitSimulation();
        // gMillSimulator->SetBoxStock(0, 0, -8.7f, 50, 50, 8.7f);
        mMillSimulator->SetBoxStock(mStock.mPx, mStock.mPy, mStock.mPz, mStock.mLx, mStock.mLy, mStock.mLz);
        mMillSimulator->InitDisplay();

        const qreal retinaScale = devicePixelRatio();
        glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    }

    void DlgCAMSimulator::CheckInitialization()
    {
        if (!mContext) {
            mContext = new QOpenGLContext(this);
            mContext->setFormat(requestedFormat());
            mContext->create();
            gOpenGlContext = mContext;
            mNeedsInitialize = true;
        }

        mContext->makeCurrent(this);

        if (mNeedsInitialize) {
            initializeOpenGLFunctions();
            initialize();
            mNeedsInitialize = false;
        }
    }

    void DlgCAMSimulator::renderNow()
    {
        if (!isExposed()) {
            return;
        }

        CheckInitialization();

        render();

        mContext->swapBuffers(this);

        if (mAnimating) {
            renderLater();
        }
    }

    void DlgCAMSimulator::setAnimating(bool animating)
    {
        mAnimating = animating;

        if (animating) {
            renderLater();
        }
    }

    DlgCAMSimulator* DlgCAMSimulator::GetInstance()
    {
        if (mInstance == nullptr)
        {
            QSurfaceFormat format;
            format.setSamples(16);
            format.setSwapInterval(1);
            mInstance = new DlgCAMSimulator();
            mInstance->setFormat(format);
            mInstance->resize(800, 600);
            mInstance->show();
        }
        return mInstance;
    }

    DlgCAMSimulator* DlgCAMSimulator::mInstance = nullptr;

    //************************************************************************************************************
    // stock
    //************************************************************************************************************
    cStock::cStock(float px, float py, float pz, float lx, float ly, float lz, float res)
        : mPx(px), mPy(py), mPz(pz + 0.005 * lz), mLx(lx), mLy(ly), mLz(1.01 * lz)
    {}

    cStock::~cStock()
    {}

}  // namespace CAMSimulator