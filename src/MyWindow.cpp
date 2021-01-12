/************************************************************************
	 File:        MyWindow.cpp (from MyWindow.cpp)

	 Author:
				  Michael Gleicher, gleicher@cs.wisc.edu

	 Modifier
				  Yu-Chi Lai, yu-chi@cs.wisc.edu
				  Maochinn, m10815023@gapps.edu.tw

	 Comment:
						this class defines the window in which the project
						runs - its the outer windows that contain all of
						the widgets, including the "TrainView" which has the
						actual OpenGL window in which the train is drawn

						You might want to modify this class to add new widgets
						for controlling	your train

						This takes care of lots of things - including installing
						itself into the FlTk "idle" loop so that we get periodic
						updates (if we're running the train).


	 Platform:    Visio Studio 2019

*************************************************************************/

#include <FL/fl.h>
#include <FL/Fl_Box.h>


// for using the real time clock
#include <time.h>

#include "MyWindow.h"
#include "MyView.h"
#include "GuiView.h"
#include "CallBack.h"


//************************************************************************
//
// * Constructor
//========================================================================
MyWindow::
MyWindow(const int x, const int y)
	: Fl_Double_Window(x, y, 800, 600, "Mesh Deformation")
	//========================================================================
{
	// make all of the widgets
	begin();	// add to this widget
	{
		int pty = 5;			// where the last widgets were drawn

		myView = new MyView(5, 5, 590, 590);
		myView->mw = this;
		//trainView->m_pTrack = &m_Track;
		this->resizable(myView);

		// to make resizing work better, put all the widgets in a group
		widgets = new Fl_Group(600, 5, 190, 590);
		widgets->begin();

		//renderMeshButton = new Fl_Button(605, pty, 60, 20, "Mesh");
		//togglify(renderMeshButton, 1);

		renderWeightButton = new Fl_Button(730, pty, 65, 20, "Weight");
		togglify(renderWeightButton, 0);
		renderWeightButton->callback((Fl_Callback*)toggleWeightCB, this);

		pty += 25;

		// camera buttons - in a radio button group
		Fl_Group* camGroup = new Fl_Group(600, pty, 195, 20);
		camGroup->begin();
		world_cam = new Fl_Button(605, pty, 60, 20, "World");
		world_cam->type(FL_RADIO_BUTTON);		// radio button
		world_cam->value(0);					// turned off
		world_cam->selection_color((Fl_Color)3); // yellow when pressed
		world_cam->callback((Fl_Callback*)damageCB, this);

		top_cam = new Fl_Button(735, pty, 60, 20, "Top");
		top_cam->type(FL_RADIO_BUTTON);
		top_cam->value(1);
		top_cam->selection_color((Fl_Color)3);
		top_cam->callback((Fl_Callback*)damageCB, this);
		camGroup->end();
		pty += 30;

		// reset the points
		Fl_Button* reset = new Fl_Button(605, pty, 60, 20, "Reset");
		reset->callback((Fl_Callback*)resetCB, this);
		Fl_Button* importMesh = new Fl_Button(675, pty, 60, 20, "Import");
		importMesh->callback((Fl_Callback*)importCB, this);
		Fl_Button* exportMesh = new Fl_Button(735, pty, 60, 20, "Export");
		exportMesh->callback((Fl_Callback*)exportCB, this);

		pty += 65;


		guiView = new GuiView(600, pty, 196, 196);
		guiView->mw = this;
		pty += 201;

		Fl_Button* importPreset = new Fl_Button(605, pty, 90, 18, "Import Preset");
		importPreset->callback((Fl_Callback*)importPresetCB, this);
		Fl_Button* exportPreset = new Fl_Button(705, pty, 90, 18, "Export Preset");
		exportPreset->callback((Fl_Callback*)exportPresetCB, this);
		pty += 35;

		/*Fl_Button* Simplification = new Fl_Button(605, pty, 60, 20, "Simplification");
		Simplification->callback((Fl_Callback*)simplificationCB, this);
		pty += 25;*/

		/*simplification_slider = new Fl_Value_Slider(655, pty, 140, 20, "S");
		simplification_slider->range(0.0, 1.0);
		simplification_slider->value(1.0);
		simplification_slider->align(FL_ALIGN_LEFT);
		simplification_slider->type(FL_HORIZONTAL);
		simplification_slider->callback((Fl_Callback*)simplificationSlideCB, this);
		pty += 30;*/

		/*WL0 = new Fl_Value_Input(675, pty, 60, 20, "WL0"); WL0->value(0.001); pty += 25;
		WH0 = new Fl_Value_Input(675, pty, 60, 20, "WH0"); WH0->value(1.0); pty += 25;
		SL = new Fl_Value_Input(675, pty, 60, 20, "SL"); SL->value(4.0); pty += 25;*/


		/*Fl_Button* Skeleton = new Fl_Button(605, pty, 60, 20, "Skeleton");
		Skeleton->callback((Fl_Callback*)SkeletonCB, this);
		pty += 25;*/

		frame_scrollbar = new Fl_Scrollbar(605, pty, 195, 20, "Scrollbar");
		frame_scrollbar->type(FL_HORIZONTAL);
		frame_scrollbar->slider_size(.1);              // the fractional size of the scrollbar's tab, 1/2 scollbar's size
		frame_scrollbar->bounds(0, 100);              // min/max value of the slider's positions
		((Fl_Valuator*)frame_scrollbar)->value(0); // the initial value (in fltk1.3+ you can use scrollbar->value(150); 
		frame_scrollbar->step(1);                     // force step rate to 1 (slider move changes value: 100, 110, 120..)
		frame_scrollbar->linesize(1);
		frame_scrollbar->callback((Fl_Callback*)scrollbarCB, this);

		pty += 40;

		// Create output to show scrollbar's value
		frame_number = new Fl_Output(655, pty, 100, 40, "Frame:");
		frame_number->textsize(24);
		frame_number->value("0");

		pty += 50;

		frame_play = new Fl_Button(605, pty, 60, 20, "Play");
		togglify(frame_play, 0);
		frame_play->callback((Fl_Callback*)framePlayCB, this);
		frame_set = new Fl_Button(675, pty, 60, 20, "Set");
		frame_set->callback((Fl_Callback*)frameSetCB, this);
		frames_record = new Fl_Button(735, pty, 60, 20, "Record");
		frames_record->callback((Fl_Callback*)framesRecordCB, this);
		togglify(frames_record, 0);


#ifdef EXAMPLE_SOLUTION
		makeExampleWidgets(this, pty);
#endif

		// we need to make a little phantom widget to have things resize correctly
		Fl_Box* resizebox = new Fl_Box(600, 595, 200, 5);
		widgets->resizable(resizebox);

		widgets->end();
	}
	end();	// done adding to this widget

	// set up callback on idle
	Fl::add_idle((void (*)(void*))idleCB, this);
}

//************************************************************************
//
// * handy utility to make a button into a toggle
//========================================================================
void MyWindow::
togglify(Fl_Button* b, int val)
//========================================================================
{
	b->type(FL_TOGGLE_BUTTON);		// toggle
	b->value(val);		// turned off
	b->selection_color((Fl_Color)3); // yellow when pressed	
	b->callback((Fl_Callback*)damageCB, this);
}

//************************************************************************
//
// *
//========================================================================
void MyWindow::
damageMe()
//========================================================================
{
	myView->damage(1);
}


void 
MyWindow::frameIncrease()
{
	int now = this->frame_scrollbar->value();
	int max = this->frame_scrollbar->maximum();
	int next = (now + 1) % (max + 1);
	this->frame_scrollbar->value(next);

	scrollbarCB(nullptr, this);
}
