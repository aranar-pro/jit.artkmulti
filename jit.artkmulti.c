/* 
 
 ARToolkit for Max/MSP + Jitter
 By Andrew Roth - aroth21@yorku.ca
 
 
Max/MSP	
 Copyright 2001-2005 - Cycling '74
	Joshua Kit Clayton jkc@cycling74.com	
 
 
 
 ARToolkit 
 Philip Lamb
 HIT Lab NZ
 http://www.hitlabnz.org
 2007-02-07
 
 The ARToolKit license model.
 ----------------------------
 
 ARToolKit is made available under a dual-license model. As it has been since the first public release of version 1.0, ARToolKit is freely available for non-commercial use under the terms of the GNU General Public License. A proprietary version of ARToolKit developed in parallel for professional commercial use by ARToolworks, Inc. is made available under different license terms, to suit end-user need.
 
 We believe this model offers the best of both worlds. Persons developing their software under an open source model are freely availed of the ARToolKit source code, and the ARToolKit open source project benefits in return from enhancements, bug reports, and external development ideas. Those persons developing under a proprietary source model, from whom the ARToolKit code base does not benefit through reciprocal openness, contribute financially instead. License fees fund research and development of today's and tomorrow's versions of the ARToolKit through research activities at partner institutions.
 
 The GPL-licensed toolkit.:
 
 ARToolKit versions 1.0 through 2.x are available under the GPL. Please read the full text of the GPL prior to downloading and/or using the ARToolKit source code. Your downloading and/or use of the code signifies acceptance of the terms and conditions of the license. If you are unable to comply with the license terms, please immediately destroy all copies of the code in your possession.
 
 Please be aware that while the GPL allows you to freely use the source code, it also imposes certain restrictions on the way in which you can use the code. Your attention is drawn particularly to section 2b of the GPL: "You must cause any work that you distribute or publish, that in whole or in part contains or is derived from the Program or any part thereof, to be licensed as a whole at no charge to all third parties under the terms of this License.", i.e. your software incorporating or linking to ARToolKit must also be open-source software, licensed under the GPL. Use of the ARToolKit in breach of the terms of the GPL will be subject to legal action by the copyright holders.
 
 Licenses for professional and commercial use.:
 
 In response to demand from toolkit users, the holders of the copyright on much of the ARToolKit version 1.0 - 2.x code have elected to make this code and other substantially advanced ARToolKit and AR code available under proprietary licenses for professional and commercial use by persons for whom the GPL license is not ideal. These license arrangements are managed by ARToolworks, Inc., Seattle, WA, USA. A variety of license types are available at reasonable cost. Please contact ARToolworks, Inc. directly for information on products, license terms, and pricing.
 
 ARToolworks also provide a variety of value-added services, including professional support, customization and other modifications, and end-user turnkey applications.
 
 loadMultiple Example from ARToolkit source, documentation online:
 
 http://www.hitl.washington.edu/artoolkit/documentation/devmulti.htm
 
*/

#include <stdio.h>
#include <stdlib.h>
#ifndef __APPLE__
#include <gl.h>
#include <glut.h>
#endif
#include <gsub.h>
#include <gsub_lite.h>
#include <video.h>
#include <param.h>
#include <math.h>
#include <ar.h>
#include <arMulti.h>
#include <matrix.h>

#include "jit.common.h"
#include "ext_systhread.h"

#include "object.h"

//#define COLLIDE_DIST 30000.0

#define QUAT_ACCURACY_MAX (1.000001)
#define QUAT_ACCURACY_MIN (0.999999)
#define QUAT_EPSILON (0.000001)
#define QUAT_RAD2DEG (57.29577951308)
#define QUAT_DEG2RAD (0.01745329252)
#define QUAT_DEG2RAD_BY2 (0.00872664626)
#define QUAT_PIBY2 (1.57079632679)


/* Object Data */

ObjectData_T    *object;
int             objectnum;
int             marker_num;

int             count = 0;
int             thresh = 100;
int             patt_id;
double          patt_width     = 80.0;
double          patt_center[2] = {0.0, 0.0}; //this will affect offset X and Y to marker
double          patt_trans[3][4];
double			marker_coord[4][2];
double			gl_para[16];
int				xsize, ysize = 0;
ARParam         cparam;
ARParam			wparam;
int				normcount;
double			tmpqt[4];
double			qt[4];
double			err_trns;
double			*aa, *ax, *ay, *az;
double			a_a, a_x, a_y, a_z;
float			collide_dist = 30000.0;

ARMultiMarkerInfoT  *config;
 
typedef struct _jit_artkmulti 
{
	t_object	ob;
	float		m1[16];
	float		m2[16];
	float		m3[16];
	float		m4[16];
	
	long		m1count;
	long		m2count;
	long		m3count;
	long		m4count;
	long		detMar[16];
	
} t_jit_artkmulti;

t_jit_err jit_artkmulti_init(void); 
t_jit_err jit_artkmulti_matrix_calc(t_jit_artkmulti *x, void *inputs, void *outputs);
void jit_artkmulti_offset(t_jit_artkmulti *x,  t_symbol *s, long argc, t_atom *argv);
void jit_artkmulti_collide(t_jit_artkmulti *x,  t_symbol *s, long argc, t_atom *argv);
void Quat_normalize();
void Quat_inverse();
void Quat_toAxisAngle(double * aa, double * ax, double * ay, double * az);
void *_jit_artkmulti_class;
static int checkCollisions( ObjectData_T object0, ObjectData_T object1, float collide_dist);

t_jit_artkmulti *jit_artkmulti_new(void);
void jit_artkmulti_free(t_jit_artkmulti *x);
t_jit_err *jit_artkmulti_param_load(void);

t_jit_err jit_artkmulti_init(void) 
{
		
 long attrflags=0;
	t_jit_object *attr,*mop;
	
	_jit_artkmulti_class = jit_class_new("jit_artkmulti",(method)jit_artkmulti_new,(method)jit_artkmulti_free,
		sizeof(t_jit_artkmulti),0L);

	//add mop
	mop = jit_object_new(_jit_sym_jit_mop,1,0);
	jit_class_addadornment(_jit_artkmulti_class,mop);
	//add methods
	jit_class_addmethod(_jit_artkmulti_class, (method)jit_artkmulti_matrix_calc, 		"matrix_calc", 		A_CANT, 0L);
	jit_class_addmethod(_jit_artkmulti_class, (method)jit_artkmulti_offset, 		"offset", 		A_GIMME, 0L);
	jit_class_addmethod(_jit_artkmulti_class, (method)jit_artkmulti_collide, 		"collide", 		A_GIMME, 0L);

	//add attributes	
	attrflags = JIT_ATTR_SET_OPAQUE_USER | JIT_ATTR_GET_DEFER_LOW;
	attr = jit_object_new(_jit_sym_jit_attr_offset_array, "m1", _jit_sym_float32,16,attrflags, (method)0L, (method)0L, calcoffset(t_jit_artkmulti,m1count), calcoffset(t_jit_artkmulti,m1));
	jit_class_addattr(_jit_artkmulti_class, attr);
	attr = jit_object_new(_jit_sym_jit_attr_offset_array, "m2", _jit_sym_float32,16,attrflags, (method)0L, (method)0L, calcoffset(t_jit_artkmulti,m2count), calcoffset(t_jit_artkmulti,m2));
	jit_class_addattr(_jit_artkmulti_class, attr);
	attr = jit_object_new(_jit_sym_jit_attr_offset_array, "m3", _jit_sym_float32,16,attrflags, (method)0L, (method)0L, calcoffset(t_jit_artkmulti,m3count), calcoffset(t_jit_artkmulti,m3));
	jit_class_addattr(_jit_artkmulti_class, attr);
	attr = jit_object_new(_jit_sym_jit_attr_offset_array, "m4", _jit_sym_float32,16,attrflags, (method)0L, (method)0L, calcoffset(t_jit_artkmulti,m4count), calcoffset(t_jit_artkmulti,m4));
	jit_class_addattr(_jit_artkmulti_class, attr);
	
	jit_class_register(_jit_artkmulti_class);

	return JIT_ERR_NONE;
}

void jit_artkmulti_offset(t_jit_artkmulti *x,  t_symbol *s, long argc, t_atom *argv){
	long ia;
	t_atom* ap;
	int mkr;
	mkr = (int) argv[0].a_w.w_float+0.5;
	for (ia = 0, ap = argv; ia < argc; ia++, ap++) {       // increment ap each time to get to the next atom
		switch (argv[ia].a_type) {
			case A_LONG:
				//post("Long: %ld: %ld",ia+1,argv[ia].a_w.w_long);
				break;
			case A_FLOAT:
				if (ia == 1) {
				//post("Float %ld: %.2f",ia+1,argv[ia].a_w.w_float);
				object[mkr-1].marker_center[0] = (double) argv[ia].a_w.w_float;
				} else if (ia == 2){
				object[mkr-1].marker_center[1] = (double) argv[ia].a_w.w_float;
				}
				//post("Float %ld: %.2f",ia+1,argv[ia].a_w.w_float);
				//if (ia < 2)
					//post("Float %ld: %.2f", ia+1, argv[ia].a_w.w_float);
					//object[ia].marker_center = (double) argv[ia].a_w.w_float;
				break;
			case A_SYM:
				//post("Sym %ld: %s",ia+1, argv[ia].a_w.w_sym->s_name);
				break;
			default:
				//post("%ld: unknown atom type (%ld)", ia+1, argv[ia].a_w.w_sym->s_name);
				break;
		}
	}
}

void jit_artkmulti_collide(t_jit_artkmulti *x,  t_symbol *s, long argc, t_atom *argv){
	long ia;
	t_atom* ap;
	//int mkr;
	//mkr = (int) argv[0].a_w.w_float+0.5;
	for (ia = 0, ap = argv; ia < argc; ia++, ap++) {       // increment ap each time to get to the next atom
		switch (argv[ia].a_type) {
			case A_LONG:
				break;
			case A_FLOAT:
				if(ia == 0) {
					collide_dist = (double) argv[ia].a_w.w_float *1000;
					post("collide radius : %f", collide_dist);
				}
				break;
			case A_SYM:
				break;
			default:
				break;
		}
	}
}


t_jit_err jit_artkmulti_matrix_calc(t_jit_artkmulti *x, void *inputs, void *outputs)
{
	t_jit_err err=JIT_ERR_NONE;
	long in_savelock;
	t_jit_matrix_info in_minfo;
	uchar *in_bp;
	long dimcount,dim[JIT_MATRIX_MAX_DIMCOUNT];
	void *in_matrix;
	
	//artk stuff
	ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             i, j, k, n, iz, jz;
	float			l[4] = {0.0, 0.0, 0.0, 0.0};
	

	aa = &a_a;
	ax = &a_x;
	ay = &a_y;
	az = &a_z;

	
	in_matrix 	= jit_object_method(inputs,_jit_sym_getindex,0);

	if (x&&in_matrix) {
		
		in_savelock = (long) jit_object_method(in_matrix,_jit_sym_lock,1);
		jit_object_method(in_matrix,_jit_sym_getinfo,&in_minfo);
		jit_object_method(in_matrix,_jit_sym_getdata,&in_bp);
		
		if ((in_minfo.dim[0]!= xsize)||(in_minfo.dim[1]!= ysize)) {
			arVideoCapStop();
			arVideoClose();
			argCleanup();
			xsize = in_minfo.dim[0];
			ysize = in_minfo.dim[1]; 
			post("input dimensions changed");
			err=JIT_ERR_INVALID_INPUT; 
			jit_artkmulti_param_load();
			goto out;
			
			
		}
		
		
		// grab a video frame, in_bp is data from incoming Jitter matrix
			if( (dataPtr = (ARUint8 *)in_bp) == NULL ) {
			arUtilSleep(2);
			return(0);
		}

		if( count == 0 ) arUtilTimerReset();
		count++;
		
		// detect the markers in the video frame 
		
		if( arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0 ) {
			arVideoCapStop();
			arVideoClose();
			argCleanup();
			jit_artkmulti_new();
		}
		
		double pos[3] = {0,0,0};
		double glpos[3] = {0,0,0};
	
		
		/* check for known patterns */
		for( i = 0; i < objectnum; i++ ) {
			k = -1;
			for( j = 0; j < marker_num; j++ ) {
				if( object[i].id == marker_info[j].id) {
					
					/* you've found a pattern */
					if( k == -1 ) k = j;
					else /* make sure you have the best pattern (highest confidence factor) */
						if( marker_info[k].cf < marker_info[j].cf ) k = j;
				}
			}	
			//post("k %d: id = %d", k, object[i].id);
			int p;
			p = object[i].id;
			if( k > -1 ) {
				l[p] = 1.0f;
				
			}
			
			if( k == -1 ) {
				l[p] = 0.0f;
				object[i].visible = 0;
				continue;
			}
			 
			/* calculate the transform for each marker */
			if( object[i].visible == 0 ) {
				
				arGetTransMat(&marker_info[k],
							  object[i].marker_center, object[i].marker_width,
							  object[i].trans);
	
				//this needs to be sorted into multi-dim arrays and more dynamic quat and Q->aa functions
			
				arUtilMat2QuatPos(object[i].trans, qt, pos);
				Quat_inverse();
				Quat_toAxisAngle(aa, ax, ay, az);
				
				glpos[0] = (float)pos[0];
				glpos[1] = -(float)pos[1];
				glpos[2] = -(float)pos[2];
				 
				x->m1[0] = l[0];
				x->m2[0] = l[1];
				x->m3[0] = l[2];
				x->m4[0] = l[3];
				if (object[i].id == 0) {
				
					x->m1[1] = *aa;
					x->m1[2] = *az;
					x->m1[3] = *ay;
					x->m1[4] = *ax;
					x->m1[5] = glpos[0];
					x->m1[6] = glpos[1];
					x->m1[7] = glpos[2];
				} else if (object[i].id == 1) {
					
					x->m2[1] = *aa;
					x->m2[2] = *az;
					x->m2[3] = *ay;
					x->m2[4] = *ax;
					x->m2[5] = glpos[0];
					x->m2[6] = glpos[1];
					x->m2[7] = glpos[2];
				} else if (object[i].id == 2) {
					
					x->m3[1] = *aa;
					x->m3[2] = *az;
					x->m3[3] = *ay;
					x->m3[4] = *ax;
					x->m3[5] = glpos[0];
					x->m3[6] = glpos[1];
					x->m3[7] = glpos[2];
				} else if (object[i].id == 3) {
					
					x->m4[1] = *aa;
					x->m4[2] = *az;
					x->m4[3] = *ay;
					x->m4[4] = *ax;
					x->m4[5] = glpos[0];
					x->m4[6] = glpos[1];
					x->m4[7] = glpos[2];
				}
				
						}
			else {
				arGetTransMatCont(&marker_info[k], object[i].trans,
								  object[i].marker_center, object[i].marker_width,
								  object[i].trans);
				//this needs to be sorted into multi-dim arrays and more dynamic quat and Q->aa functions
				arUtilMat2QuatPos(object[i].trans, qt, pos);
				Quat_inverse();
				Quat_toAxisAngle(aa, ax, ay, az);
				
				//argConvGlpara(object[i].trans, gl_para);

				glpos[0] = (float)pos[0];
				glpos[1] = -(float)pos[1];
				glpos[2] = -(float)pos[2];
				
				x->m1[0] = l[0];
				x->m2[0] = l[1];
				x->m3[0] = l[2];
				x->m4[0] = l[3];
				
				if (object[i].id == 0) {
					x->m1[1] = *aa;
					x->m1[2] = *az;
					x->m1[3] = *ay;
					x->m1[4] = *ax;
					x->m1[5] = glpos[0];
					x->m1[6] = glpos[1];
					x->m1[7] = glpos[2];
				} else if (object[i].id == 1) {
					
					x->m2[1] = *aa;
					x->m2[2] = *az;
					x->m2[3] = *ay;
					x->m2[4] = *ax;
					x->m2[5] = glpos[0];
					x->m2[6] = glpos[1];
					x->m2[7] = glpos[2];
				} else if (object[i].id == 2) {
					
					x->m3[1] = *aa;
					x->m3[2] = *az;
					x->m3[3] = *ay;
					x->m3[4] = *ax;
					x->m3[5] = glpos[0];
					x->m3[6] = glpos[1];
					x->m3[7] = glpos[2];
				} else if (object[i].id == 3) {
					
					x->m4[1] = *aa;
					x->m4[2] = *az;
					x->m4[3] = *ay;
					x->m4[4] = *ax;
					x->m4[5] = glpos[0];
					x->m4[6] = glpos[1];
					x->m4[7] = glpos[2];
				}
			}
			//post("object %d value of visible: %d", i, object[i].visible);
			object[i].visible = 1;
		
			//begin collision tests
			for(iz=0;iz<4;iz++){
				for(jz=0;jz<4;jz++){
				//check for object collisions between marker 0 and 1
					if(iz!=jz){
						//post("%d | %d", iz, jz);
						if(object[iz].visible && object[jz].visible){
							if(checkCollisions(object[iz],object[jz],collide_dist)){
								if(iz==0){
									x->m1[8] = (float)jz+1;
								}
								if(iz==1){
									x->m2[8] = (float)jz+1;
								}
								if(iz==2){
									x->m3[8] = (float)jz+1;
								}
								if(iz==3){
									x->m4[8] = (float)jz+1;
								}
								if(jz==0){
									x->m1[8] = (float)iz+1;
								}
								if(jz==1){
									x->m2[8] = (float)iz+1;
								}
								if(jz==2){
									x->m3[8] = (float)iz+1;
								}
								if(jz==3){
									x->m4[8] = (float)iz+1;
								}
								
							}
							else{
								if(iz==0){
									x->m1[8] = -1.f;
								}
								if(iz==1){
									x->m2[8] = -1.f;
								}
								if(iz==2){
									x->m3[8] = -1.f;
								}
								if(iz==3){
									x->m4[8] = -1.f;
								}

							}
						}
					}
				}
			}
			 //end collisions here^^
		}
		
		argSwapBuffers();
		
		if (!in_bp) { err=JIT_ERR_INVALID_INPUT; goto out;}
		
		//get dimensions/planecount 
	
		
		dimcount    = in_minfo.dimcount;
		for (i=0;i<dimcount;i++) {
			dim[i] = in_minfo.dim[i];
		}	
		
	} else {
		return JIT_ERR_INVALID_PTR;
	}
	
out:
	jit_object_method(in_matrix,_jit_sym_lock,in_savelock);
	return err;
	
}
	
t_jit_artkmulti *jit_artkmulti_new(void)
{
		xsize = 640;	
		ysize = 480;

		t_jit_artkmulti *x;
		if (x=(t_jit_artkmulti *)jit_object_alloc(_jit_artkmulti_class)) {
			x->m1count = 16;
			x->m2count = 16;
			x->m3count = 16;
			x->m4count = 16;
			jit_artkmulti_param_load();
	} else {
		x = NULL;
	}	
	return x;
}

t_jit_err *jit_artkmulti_param_load(void) {
	
	//Paths for marker and camera.dat files
	char 				caminputstring[MAX_PATH_CHARS] = "camera_para.dat";
	char 				camfilename[MAX_PATH_CHARS] = "camera_para.dat";
	char				camfilefullpath[MAX_PATH_CHARS] = " ";
	char				camfilepathtemp[MAX_PATH_CHARS] = " ";
	char 				markinputstring[MAX_PATH_CHARS] = "object_data2";
	char 				markfilename[MAX_PATH_CHARS] = "object_data2";
	char				markfilefullpath[MAX_PATH_CHARS] = " ";
	char				markfilepathtemp[MAX_PATH_CHARS] = " ";
	char				markfilepathfolder[MAX_PATH_CHARS] = " ";
	short 				fullpath, path1, path2, err, fileflag;
	long 				type1 = 'DATA', outtype1;
	long 				type2 = 'TEXT', outtype2;
	char				*cparam_name;
	char				*model_name;
	char				* pch;
    // Check for Environment (Max or Standalone) by Timothy Place of Tap Tools.  Thanks Tim!!
	char	appname[256];
	char	appath[256];
	int		isMax, is_standalone = 0;
	appname[0] = ' ';
		
#ifdef MAC_VERSION
	CFBundleRef	bun = CFBundleGetMainBundle();
	CFURLRef	url = CFBundleCopyBundleURL(bun);
	CFStringRef string = CFURLCopyLastPathComponent(url);
	CFStringRef stringfull = CFURLCopyFileSystemPath(url, 0);
	CFStringGetCString(string, appname, 256, 0);
	CFStringGetCString(stringfull, appath, 256, 0);
	isMax =	strcmp(appname, "MaxMSP.app");
	//post("%s", appath);
		//strcmp 0 is true
		//post("ARTK IsMax: %i AppName: %s", isMax, appname);
	CFRelease(string);
	CFRelease(stringfull);
#else
	char	*appname_win;
	HMODULE hMod;
	hMod = GetModuleHandle(NULL);
	GetModuleFileName(hMod, (LPCH)appname, sizeof(appname));
		
	appname_win = strrchr(appname, '\\');
	isMax =	strcmp(appname_win+1, "max.exe");	// Max 4 had a lower case first letter
	if(isMax)	// the above did not match
		isMax =	strcmp(appname_win+1, "Max.exe");	// Max 5 has an upper case first letter
		//post("ARTK IsMax: %i AppName: %s", isMax, appname_win+1);
#endif
	is_standalone = (isMax != 0);

	if (path_frompathname(caminputstring, &path1, camfilename)) {
		strcpy(camfilename, caminputstring);
		locatefile_extended(camfilename, &path1, &outtype1, &type1, 1);
		path_topotentialname(path1, camfilename, camfilefullpath, fileflag);
		path_nameconform(camfilefullpath, camfilepathtemp, PATH_STYLE_SLASH, PATH_TYPE_BOOT);
		//post("camfilefullpath: %s", camfilepathtemp);
		if (locatefile_extended(camfilename, &path1, &outtype1, &type1, 1)) {
			//jit_object_error((t_object *)x,"jit.textfile: file not found");
			error("%s not in the Max Search Path.", camfilename);
			return 0;
		}
	}
	if (path_frompathname(markinputstring, &path2, markfilename)) {
		strcpy(markfilename, markinputstring);
		locatefile_extended(markfilename, &path2, &outtype2, &type2, 1);
		path_topotentialname(path2, markfilename, markfilefullpath, fileflag);
		path_nameconform(markfilefullpath, markfilepathtemp, PATH_STYLE_SLASH, PATH_TYPE_BOOT);
		//post("markfilefullpath: %s", markfilepathtemp);
		if (locatefile_extended(markfilename, &path2, &outtype2, &type2, 1)) {
			//jit_object_error((t_object *)x,"jit.textfile: file not found");
			error("%s not in the Max Search Path.", markfilename);
			return 0;
		}
	}
	
	
	strcpy(markfilepathfolder, markfilepathtemp);
	//post("path to file %s", markfilepathfolder);
	pch = strstr(markfilepathfolder,markfilename);
	strncpy(pch,"\0",2);

	//post("enclosing folder %s", markfilepathfolder);
	
	
	//The multimarker dat file doesn't understand spaces in the file names, so the location for the markers has to be somewhere other than the Cycling '74 folder for support.
	//char			*cparam_name    = "/Applications/Max5/artk support/camera_para.dat"; //this must be an absolute path
	//char			*cparam_name    = "/Applications/Max5/artk.support/camera_para.dat"; //this must be an absolute path
	cparam_name		= camfilepathtemp;
	
	
	//char			*cparam_name    = "/Applications/Max5/artk.support/camera_para_isight2.dat"; //for isight on Macbook Pro (would require overhaul of lens_angle, camera distance and screen dimensions).
	//char          *model_name		= "/Applications/Max5/artk.support/multi/object_data2";
	model_name		= markfilepathtemp;

	//2 lines below VALID - FOR QUICKTIME in object
	//if( arVideoOpen( vconf ) < 0 ) post ("videoOpen not working");
	//if( arVideoInqSize(&xsize, &ysize) < 0 ) post ("camera fail");
	//t_jit_artkmulti *x;
	post("*** Initializing ARToolkit for Max/MSP***\n");
	post("ARToolkit Property of ARToolworks\n");
	post("Max External by Andrew Roth -- aroth21@yorku.ca\n");
	post("Alpha Release Only - r003 \n");
	post("Image size (x,y) = (%d,%d)\n", xsize, ysize);
	//post("Camera Params at: %s", cparam_name);
	
	if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
		post("Camera parameter load error!!\n");
	}
	arParamChangeSize( &wparam, xsize, ysize, &cparam );
	arInitCparam( &cparam );
	arParamDisp( &cparam );

	object =read_ObjData(model_name, &objectnum, markfilepathfolder);
	post("--->>> Objectfile num = %d\n", objectnum);
	
	//post("->>Object name = %c\n", object[i].name);
	
   // argInit( &cparam, 1.0, 0, 640, 480, 0 );
	return 0;
}

void jit_artkmulti_free(t_jit_artkmulti *x)
{
	arVideoCapStop();
	arVideoClose();
	argCleanup();
}

void Quat_inverse() {
	Quat_normalize();
	qt[0] = qt[0];
	qt[1] = -qt[1];
	qt[2] = qt[2];
	qt[3] = qt[3];
}


void Quat_normalize() {
	normcount = 0;
	double unit = qt[0]*qt[0] + qt[1]*qt[1] + qt[2]*qt[2] + qt[3]*qt[3];
	if (unit*unit < QUAT_EPSILON) { 
		// unit too close to epsilon, set to default transform
		qt[0] = 1.0; 
		qt[1] = qt[2] = qt[3] = 0.0;
		return;
	}
	if (unit > QUAT_ACCURACY_MAX || unit < QUAT_ACCURACY_MIN) {
		double invmag = 1.0/sqrt(unit);
		qt[0] *= invmag; 
		qt[1] *= invmag; 
		qt[2] *= invmag;
		qt[3] *= invmag;
	}
}


void Quat_toAxisAngle(double *aa, double *ax, double *ay, double *az) {
	//to normalize
	normcount = 0;
	double unit = qt[0]*qt[0] + qt[1]*qt[1] + qt[2]*qt[2] + qt[3]*qt[3];
	if (unit*unit < QUAT_EPSILON) { 
		// unit too close to epsilon, set to default transform
		qt[0] = 1.0; 
		qt[1] = qt[2] = qt[3] = 0.0;
		return;
	}
	if (unit > QUAT_ACCURACY_MAX || unit < QUAT_ACCURACY_MIN) {
		double invmag = 1.0/sqrt(unit);
		qt[0] *= invmag;
		qt[1] *= invmag; 
		qt[2] *= invmag;
		qt[3] *= invmag;
	}
	//convert
	unit = 0.;
	unit = qt[0]*qt[0];
	if (unit > QUAT_ACCURACY_MAX || unit < QUAT_ACCURACY_MIN)  {
		double invSinAngle = 1.0/sqrt(1.0 - unit);
		*aa = acos(qt[0]) * 114.59155902616; // * 2 * 180 / pi
		*ax = qt[1] * invSinAngle;
		*ay = qt[2] * invSinAngle;
		*az = qt[3] * invSinAngle;
	} else {
		*aa = -0.0;
		*ax = qt[1];
		*ay = -qt[2];
		*az = qt[3];
	}
}


/* check collision between two markers */
static int checkCollisions( ObjectData_T object0, ObjectData_T object1, float collide_dist)
{
	float x1,y1,z1;
	float x2,y2,z2;
	float dist;
	
	x1 = object0.trans[0][3];
	y1 = object0.trans[1][3];
	z1 = object0.trans[2][3];
	
	x2 = object1.trans[0][3];
	y2 = object1.trans[1][3];
	z2 = object1.trans[2][3];
	
	dist = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2);
	
	//post("Dist = %3.2f\n",dist);
	
	if(dist < collide_dist)
		return 1;
	else 
		return 0;
}

