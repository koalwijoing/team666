#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
static Window search(Display*d,Window w,unsigned long pid,unsigned depth=0){
 if(depth>12)return None;
 Atom type;int format;unsigned long count,left;unsigned char* data=nullptr;
 const Atom prop=XInternAtom(d,"_NET_WM_PID",False);
 unsigned long owner=0;
 if(XGetWindowProperty(d,w,prop,0,1,False,XA_CARDINAL,&type,&format,&count,&left,&data)==Success&&data&&format==32&&count==1)owner=*reinterpret_cast<unsigned long*>(data);
 if(data)XFree(data);
 char* name=nullptr;XFetchName(d,w,&name);
 bool matched=owner==pid&&name&&std::strcmp(name,"FPGA Heterogeneous Vision")==0;
 if(name)XFree(name);if(matched)return w;
 Window root,parent,*children=nullptr;unsigned n=0;
 if(!XQueryTree(d,w,&root,&parent,&children,&n))return None;
 Window found=None;for(unsigned i=0;i<n&&!found;++i)found=search(d,children[i],pid,depth+1);
 if(children)XFree(children);return found;
}
int main(int argc,char**argv){
 if(argc<2)return 2;unsigned long pid=std::strtoul(argv[1],nullptr,10);if(pid<2)return 2;
 Display*d=XOpenDisplay(nullptr);if(!d){std::fprintf(stderr,"no X display\n");return 3;}
 Window w=search(d,DefaultRootWindow(d),pid);if(!w){std::fprintf(stderr,"no exact PID/name window\n");XCloseDisplay(d);return 4;}
 std::printf("GUI_TARGET pid=%lu window=%lu exact_name=FPGA Heterogeneous Vision\n",pid,w);
 if(argc>2&&std::strcmp(argv[2],"stop")==0){
  XEvent e{};e.xkey.display=d;e.xkey.window=w;e.xkey.root=DefaultRootWindow(d);e.xkey.subwindow=None;e.xkey.time=CurrentTime;
  e.xkey.x=1;e.xkey.y=1;e.xkey.x_root=1;e.xkey.y_root=1;e.xkey.same_screen=True;e.xkey.keycode=XKeysymToKeycode(d,XK_q);
  e.type=KeyPress;int a=XSendEvent(d,w,True,KeyPressMask,&e);e.type=KeyRelease;int b=XSendEvent(d,w,True,KeyReleaseMask,&e);
  XSync(d,False);if(!a||!b){XCloseDisplay(d);return 5;}
  std::printf("GUI_QUIT_REQUEST_SENT target_only=1 controller_registers_written=0\n");
 }
 XCloseDisplay(d);return 0;
}
