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
#include "GuiView.h"

#pragma warning(push)
#pragma warning(disable:4312)
#pragma warning(disable:4311)
#include <Fl/Fl_File_Chooser.H>
#include <Fl/math.h>
#pragma warning(pop)
//***************************************************************************
//
// * any time something changes, you need to force a redraw
//===========================================================================
void damageCB(Fl_Widget*, MyWindow* mw)
{
	mw->damageMe();
}

void importPresetCB(Fl_Widget*, MyWindow* mw)
{
	const char* fname =
		fl_file_chooser("Pick control point preset", "*.{preset}", PROJECT_DIR "/Models/data/presets/");

	if (fname) {
		mw->myView->gl_mesh->importPreset(fname);
		//mw->guiView->damage(1);
	}
}
void exportPresetCB(Fl_Widget*, MyWindow* mw)
{
	const char* fname =
		fl_file_chooser("Pick control point preset", "*.{preset}", PROJECT_DIR "/Models/data/presets/output.preset");

	if (fname) {
		mw->myView->gl_mesh->exportPreset(fname);
	}
}

void importCB(Fl_Widget*, MyWindow* mw)
{
	const char* fname =
		fl_file_chooser("Pick input file", "*.{txt,bmp,jpg,png,obj}", PROJECT_DIR "/Models/data/");
	if (fname) {
		mw->myView->gl_mesh->Init(fname);
		mw->damageMe();
		//mw->guiView->damage(1);
	}
}

void exportCB(Fl_Widget*, MyWindow* mw)
{
	const char* fname =
		fl_file_chooser("Pick output file", "*.{obj}", PROJECT_DIR "/Models/data/output.obj");

	if (fname) {
		mw->myView->gl_mesh->exportMesh(fname);
		//mw->damageMe();
	}
}

void toggleWeightCB(Fl_Widget*, MyWindow* mw)
{
	if (!mw->weightModeButton->value()) {
		mw->myView->gl_mesh->applyTriangleWeights();
	}

	mw->damageMe();
}

void resetCB(Fl_Widget*, MyWindow* mw)
{
	mw->myView->translation = glm::vec3(0, 0, 0);
	mw->myView->gl_mesh->resetMesh();
	mw->damageMe();
}

// Scrollbar changed: show scrollbar's value when changed
void scrollbarCB(Fl_Widget*, MyWindow* mw) {
	char s[20];
	sprintf(s, "%d", mw->frame_scrollbar->value());
	mw->frame_number->value(s);

	mw->myView->gl_mesh->loadFrameControlPoint(mw->frame_scrollbar->value());
	mw->damageMe();
}

void framePlayCB(Fl_Widget* w, MyWindow* mw)
{
	
}

void frameSetCB(Fl_Widget*, MyWindow* mw)
{
	mw->myView->gl_mesh->setFrameControlPoint(mw->frame_scrollbar->value());
	//mw->damageMe();
}

void framesRecordCB(Fl_Widget*, MyWindow* mw)
{

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

			if (mw->frame_play->value()) {
				mw->frameIncrease();
			}

			lastRedraw = clock();
			mw->damageMe();
		}

	}
}

