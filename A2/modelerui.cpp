// generated by Fast Light User Interface Designer (fluid) version 1.0106

#include "modelerui.h"

inline void ModelerUserInterface::cb_m_controlsWindow_i(Fl_Double_Window*, void*) {
  exit(0);
}
void ModelerUserInterface::cb_m_controlsWindow(Fl_Double_Window* o, void* v) {
  ((ModelerUserInterface*)(o->user_data()))->cb_m_controlsWindow_i(o,v);
}

inline void ModelerUserInterface::cb_Save_i(Fl_Menu_*, void*) {
  char *filename = NULL;
filename = fl_file_chooser("Save BMP File", "*.bmp", NULL);
if (filename)
{
	int x = m_modelerView->x();
	int y = m_modelerView->y();
	int w = m_modelerView->w();
	int h = m_modelerView->h();

	m_modelerWindow->show();
//	do {Sleep(10); }
//	while (!m_modelerWindow->shown());
//	m_modelerView->draw();
	m_modelerView->make_current();
m_modelerView->draw();
	
		
	unsigned char *imageBuffer = new unsigned char[3*w*h];

        // Tell openGL to read from the front buffer when capturing
        // out paint strokes
        glReadBuffer(GL_BACK);

        glPixelStorei( GL_PACK_ALIGNMENT, 1 );
        glPixelStorei( GL_PACK_ROW_LENGTH, w );
        
        glReadPixels( 0, 0, w, h, 
                GL_RGB, GL_UNSIGNED_BYTE, 
                imageBuffer );


	writeBMP(filename, w,h, imageBuffer);

	delete [] imageBuffer;
};
}
void ModelerUserInterface::cb_Save(Fl_Menu_* o, void* v) {
  ((ModelerUserInterface*)(o->parent()->user_data()))->cb_Save_i(o,v);
}

inline void ModelerUserInterface::cb_Open_i(Fl_Menu_*, void*) {
  char *filename = NULL;
	filename = fl_file_chooser("Open .pos File", "*.pos", NULL);

	if (filename)
	{
		std::ifstream ifs( filename );
		if( !ifs ) {
			std::cerr << "Error: couldn't read position file " << filename << std::endl;
			return;
		}
		
		int controlNum; 
		float value;
		while( ifs >> controlNum >> value )
		{
			if( controlNum >= ModelerApplication::Instance()->GetNumControls() ) {
				break;
			}
			
			ModelerApplication::Instance()->SetControlValue(controlNum, value);
		}

		m_modelerView->redraw();
	};
}
void ModelerUserInterface::cb_Open(Fl_Menu_* o, void* v) {
  ((ModelerUserInterface*)(o->parent()->user_data()))->cb_Open_i(o,v);
}

inline void ModelerUserInterface::cb_Save1_i(Fl_Menu_*, void*) {
  char *filename = NULL;
	filename = fl_file_chooser("Save .pos File", "*.pos", NULL);

	if (filename)
	{
		FILE* m_posFile = fopen(filename, "w");

		float elevation, azimuth, dolly, twist;

		double value;
        	for(int i = 0; i < ModelerApplication::Instance()->GetNumControls(); i++)
		{
			value = ModelerApplication::Instance()->GetControlValue(i);

			fprintf(m_posFile, "%d %f\n", i, value);
		}

		fclose(m_posFile);
	};
}
void ModelerUserInterface::cb_Save1(Fl_Menu_* o, void* v) {
  ((ModelerUserInterface*)(o->parent()->user_data()))->cb_Save1_i(o,v);
}

inline void ModelerUserInterface::cb_Exit_i(Fl_Menu_*, void*) {
  m_controlsWindow->hide();
m_modelerWindow->hide();
}
void ModelerUserInterface::cb_Exit(Fl_Menu_* o, void* v) {
  ((ModelerUserInterface*)(o->parent()->user_data()))->cb_Exit_i(o,v);
}

inline void ModelerUserInterface::cb_m_controlsAnimOnMenu_i(Fl_Menu_*, void*) {
  ModelerApplication::Instance()->m_animating = (m_controlsAnimOnMenu->value() == 0) ? false : true;
}
void ModelerUserInterface::cb_m_controlsAnimOnMenu(Fl_Menu_* o, void* v) {
  ((ModelerUserInterface*)(o->parent()->user_data()))->cb_m_controlsAnimOnMenu_i(o,v);
}

Fl_Menu_Item ModelerUserInterface::menu_m_controlsMenuBar[] = {
 {"File", 0,  0, 0, 64, 0, 0, 14, 56},
 {"Save Bitmap File", 0,  (Fl_Callback*)ModelerUserInterface::cb_Save, 0, 128, 0, 0, 14, 56},
 {"Open Position File", 0,  (Fl_Callback*)ModelerUserInterface::cb_Open, 0, 0, 0, 0, 14, 56},
 {"Save Position File", 0,  (Fl_Callback*)ModelerUserInterface::cb_Save1, 0, 128, 0, 0, 14, 56},
 {"Exit", 0,  (Fl_Callback*)ModelerUserInterface::cb_Exit, 0, 0, 0, 0, 14, 56},
 {0,0,0,0,0,0,0,0,0},
 {"Animate", 0,  0, 0, 64, 0, 0, 14, 56},
 {"Enable", 0,  (Fl_Callback*)ModelerUserInterface::cb_m_controlsAnimOnMenu, 0, 2, 0, 0, 14, 56},
 {0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0}
};
Fl_Menu_Item* ModelerUserInterface::m_controlsAnimOnMenu = ModelerUserInterface::menu_m_controlsMenuBar + 7;

inline void ModelerUserInterface::cb_m_controlsBrowser_i(Fl_Browser*, void*) {
  for (int i=0; i<ModelerApplication::Instance()->m_numControls; i++) {
	if (m_controlsBrowser->selected(i+1))
		ModelerApplication::Instance()->ShowControl(i);
	else
		ModelerApplication::Instance()->HideControl(i);
};
}
void ModelerUserInterface::cb_m_controlsBrowser(Fl_Browser* o, void* v) {
  ((ModelerUserInterface*)(o->parent()->user_data()))->cb_m_controlsBrowser_i(o,v);
}

inline void ModelerUserInterface::cb_m_modelerWindow_i(Fl_Double_Window*, void*) {
  exit(0);
}
void ModelerUserInterface::cb_m_modelerWindow(Fl_Double_Window* o, void* v) {
  ((ModelerUserInterface*)(o->user_data()))->cb_m_modelerWindow_i(o,v);
}

ModelerUserInterface::ModelerUserInterface() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = m_controlsWindow = new Fl_Double_Window(395, 325, "Assignment 2 Controls");
    w = o;
    o->callback((Fl_Callback*)cb_m_controlsWindow, (void*)(this));
    o->when(FL_WHEN_NEVER);
    { Fl_Menu_Bar* o = m_controlsMenuBar = new Fl_Menu_Bar(0, 0, 395, 25);
      o->menu(menu_m_controlsMenuBar);
    }
    { Fl_Browser* o = m_controlsBrowser = new Fl_Browser(0, 25, 140, 300, "Controls");
      o->type(3);
      o->textsize(10);
      o->callback((Fl_Callback*)cb_m_controlsBrowser);
      Fl_Group::current()->resizable(o);
    }
    { Fl_Scroll* o = m_controlsScroll = new Fl_Scroll(145, 25, 250, 300);
      o->type(6);
      o->when(FL_WHEN_CHANGED);
      { Fl_Pack* o = m_controlsPack = new Fl_Pack(145, 25, 225, 300);
        o->end();
      }
      o->end();
    }
    o->end();
  }
  { Fl_Double_Window* o = m_modelerWindow = new Fl_Double_Window( 800, 800, "Assignment 2 Model");
    w = o;
    o->callback((Fl_Callback*)cb_m_modelerWindow, (void*)(this));
    o->when(FL_WHEN_NEVER);
    { ModelerView* o = m_modelerView = new ModelerView(0, 0, 800, 800, "ModelerView");
      o->box(FL_NO_BOX);
      o->color(FL_BACKGROUND_COLOR);
      o->selection_color(FL_BACKGROUND_COLOR);
      o->labeltype(FL_NORMAL_LABEL);
      o->labelfont(0);
      o->labelsize(14);
      o->labelcolor(FL_BLACK);
      o->align(FL_ALIGN_CENTER);
      o->when(FL_WHEN_RELEASE);
      Fl_Group::current()->resizable(o);
    }
    o->end();
  }
}

void ModelerUserInterface::show() {
  m_controlsWindow->show();
m_modelerWindow->show();
m_modelerView->show();
}
