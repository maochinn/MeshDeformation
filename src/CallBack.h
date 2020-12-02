/************************************************************************
	 File:        CallBacks.H

	 Author:
				  Michael Gleicher, gleicher@cs.wisc.edu
	 Modifier
				  Yu-Chi Lai, yu-chi@cs.wisc.edu

	 Comment:     Header file to define callback functions.
						define the callbacks for the TrainWindow

						these are little functions that get called when the
						various widgets
						get accessed (or the fltk timer ticks). these
						functions are used
						when TrainWindow sets itself up.

	 Platform:    Visio Studio.Net 2003/2005

*************************************************************************/
#pragma once

#include <time.h>
#include <math.h>

#include "MyWindow.h"
#include "MyView.h"
#include "CallBack.h"

#pragma warning(push)
#pragma warning(disable:4312)
#pragma warning(disable:4311)
#include <Fl/Fl_File_Chooser.H>
#include <Fl/math.h>
#pragma warning(pop)

#include "ARAP/TriangulationCgal.h"

//***************************************************************************
//
// * any time something changes, you need to force a redraw
//===========================================================================
void damageCB(Fl_Widget*, MyWindow* mw)
{
	mw->damageMe();
}

static unsigned long lastRedraw = 0;
//***************************************************************************
//
// * Callback for idling - if things are sitting, this gets called
// if the run button is pushed, then we need to make the train go.
// This is taken from the old "RunButton" demo.
// another nice problem to have - most likely, we'll be too fast
// don't draw more than 30 times per second
//===========================================================================
void idleCB(MyWindow* mw)
//===========================================================================
{
	if (mw != NULL)
	{
		//float fps = CLOCKS_PER_SEC / (float)(clock() - lastRedraw);
		//if (fps < 30.0)
		if (clock() - lastRedraw > CLOCKS_PER_SEC / 30)
		{
			//system("cls");
			//std::cout << "FPS:" << fps << std::endl;

			lastRedraw = clock();
			mw->damageMe();
		}

	}
}

//void testingCB(Fl_Widget*, MyWindow* mw)
//{
//	mw->myView->gl_mesh->degenerationMeshToLine(mw->degeneration_slider->value());
//	mw->damageMe();
//}

void loadImageCB(Fl_Widget*, MyWindow* mw)
{
	//const char* fname =
	//	fl_file_chooser("Pick a OBJ File", "*.obj", "../MeshSimplification/Models/neptune_50k_hk_normalize.obj");
	//if (fname) {
	//	mw->myView->gl_mesh->Init(fname);
	//	mw->damageMe();
	//}

	MyWindow::Mode_Display& Current_Display = mw->Current_Display;
	vavImage* ImageEdge = mw->ImageEdge;

	Current_Display.openImg = 1;
	ImageEdge->ReadImage("gingerbread_man.bmp");
	*ImageEdge = (ImageEdge->CannyEdge());

	std::cout << "Load img :" << ImageEdge->GetHeight() << "*" << ImageEdge->GetWidth() << std::endl;

	//hkoglPanelControl1->Invalidate();
	mw->damageMe();
}

void triangulateCB(Fl_Widget*, MyWindow* mw)
{
	MyWindow::Mode_Display& Current_Display = mw->Current_Display;
	vavImage* ImageEdge = mw->ImageEdge;
	TriangulationCgal* Triangulate = mw->Triangulate;
	TriMesh2D* test_1 = mw->test_1;
	ShapeView* ShapeView_Object = mw->ShapeView_Object;
	ArapInteractor* Arap = mw->Arap;

	if (Current_Display.openImg)
	{
		Current_Display.triangulation = 1;
		Current_Display.openImg = 0;
		Triangulate = new TriangulationCgal;
		Triangles Tris;
		Vector2s meshPointset;
		Vector2s ContourPoint = ImageEdge->GetContour();

		for (int i = 0; i < ContourPoint.size(); i += 15)
		{
			Triangulate->AddPoint(ContourPoint[i][0], ContourPoint[i][1]);
		}

		Triangulate->DelaunayMesher2();
		meshPointset = Triangulate->MeshPointSet();

		for (int i = 0; i < meshPointset.size(); i++)
		{
			test_1->vertices.push_back(Point2D(meshPointset[i][0], meshPointset[i][1]));
		}

		Tris = Triangulate->GetTriangles();
		std::cout << "Tris.size() :" << Tris.size() << std::endl;
		Tri v;
		for (int i = 0; i < Tris.size(); i++)
		{
			if (!ImageEdge->IsinsidePoint(Tris[Tris.size() - 1 - i].m_Points[0][0], Tris[Tris.size() - 1 - i].m_Points[0][1],
				Tris[Tris.size() - 1 - i].m_Points[1][0], Tris[Tris.size() - 1 - i].m_Points[1][1],
				Tris[Tris.size() - 1 - i].m_Points[2][0], Tris[Tris.size() - 1 - i].m_Points[2][1]))
				continue;
			for (int j = 0; j < 3; j++)
			{
				v[j] = Triangulate->getVertexID(Tris[Tris.size() - 1 - i].m_Points[j][0], Tris[Tris.size() - 1 - i].m_Points[j][1]);
			}
			test_1->tris.push_back(v);
		}
		Arap = new ArapInteractor(ShapeView_Object, *test_1);

	}
	//hkoglPanelControl1->Invalidate();
	mw->damageMe();
}
