/* 
	Copyright 2001-2005 - Cycling '74
	Joshua Kit Clayton jkc@cycling74.com	
*/

#include "jit.common.h"
#include "max.jit.mop.h"

typedef struct _max_jit_artkmulti 
{
	t_object			ob;
	void				*obex;
	void				*mark1;
	void				*mark2;
	void				*mark3;
	void				*mark4;
	t_atom				*av;

} t_max_jit_artkmulti;

t_jit_err jit_artkmulti_init(void); 

void *max_jit_artkmulti_new(t_symbol *s, long argc, t_atom *argv);
void max_jit_artkmulti_free(t_max_jit_artkmulti *x);
void max_jit_artkmulti_assist(t_max_jit_artkmulti *x, void *b, long m, long a, char *s);
//void max_jit_artkmulti_bang(t_max_jit_artkmulti *x);
void max_jit_artkmulti_mproc(t_max_jit_artkmulti *x, void *mop);
void *max_jit_artkmulti_class;

t_symbol *ps_getm1;
t_symbol *ps_getm2;
t_symbol *ps_getm3;
t_symbol *ps_getm4;
		 	
void main(void)
{	
	void *p,*q;
	
	jit_artkmulti_init();	
	setup((t_messlist **)&max_jit_artkmulti_class, (method)max_jit_artkmulti_new, (method)max_jit_artkmulti_free, (short)sizeof(t_max_jit_artkmulti), 0L, A_GIMME, 0);
	p = max_jit_classex_setup(calcoffset(t_max_jit_artkmulti,obex));
	q = jit_class_findbyname(gensym("jit_artkmulti"));    
    max_jit_classex_mop_wrap(p,q,MAX_JIT_MOP_FLAGS_OWN_BANG|MAX_JIT_MOP_FLAGS_OWN_OUTPUTMATRIX); //custom bang/outputmatrix 		
    max_jit_classex_mop_mproc(p,q,max_jit_artkmulti_mproc); 	//custom mproc
    max_jit_classex_standard_wrap(p,q,0); 	
 	addmess((method)max_jit_artkmulti_assist,			"assist",			A_CANT,0);
	
	ps_getm1 = gensym("getm1");
	ps_getm2 = gensym("getm2");
	ps_getm3 = gensym("getm3");
	ps_getm4 = gensym("getm4");

}

void max_jit_artkmulti_mproc(t_max_jit_artkmulti *x, void *mop)
{
	t_jit_err err;
	long ac = 0;
	void *o;
	
	if (max_jit_mop_getoutputmode(x)&&x->av){
	//Get pointer to Jitter object
	o = max_jit_obex_jitob_get(x);
	if (err=(t_jit_err) jit_object_method(o,_jit_sym_matrix_calc,jit_object_method(mop,_jit_sym_getinputlist),jit_object_method(mop,_jit_sym_getoutputlist)))    //Call matrix_calc method
		{	
		jit_error_code(x,err); 
	} else {
		jit_object_method(o,ps_getm1,&ac,&(x->av));
		//this is all in cv.jit.moments
		switch(ac) {
			case 1:
				outlet_float(x->mark1,jit_atom_getfloat(x->av));
				break;
			default:			
				outlet_anything(x->mark1,_jit_sym_list,ac,(x->av));
				break;
				if (x->av) jit_freebytes(x->av,(ac)*sizeof(t_atom));
				x->av=NULL; ac=0;
		}
		jit_object_method(o,ps_getm2,&ac,&(x->av));
		//this is all in cv.jit.moments
		switch(ac) {
			case 1:
				outlet_float(x->mark2,jit_atom_getfloat(x->av));
				break;
			default:			
				outlet_anything(x->mark2,_jit_sym_list,ac,(x->av));
				break;
				if (x->av) jit_freebytes((x->av),(ac)*sizeof(t_atom));
				x->av=NULL; ac=0;
			}
		jit_object_method(o,ps_getm3,&ac,&(x->av));
		//this is all in cv.jit.moments
		switch(ac) {
			case 1:
				outlet_float(x->mark3,jit_atom_getfloat(x->av));
				break;
			default:			
				outlet_anything(x->mark3,_jit_sym_list,ac,(x->av));
				break;
				if (x->av) jit_freebytes((x->av),(ac)*sizeof(t_atom));
				x->av=NULL; ac=0;
		}
		jit_object_method(o,ps_getm4,&ac,&(x->av));
		//this is all in cv.jit.moments
		switch(ac) {
			case 1:
				outlet_float(x->mark4,jit_atom_getfloat(x->av));
				break;
			default:			
				outlet_anything(x->mark4,_jit_sym_list,ac,(x->av));
				break;
				if (x->av) jit_freebytes((x->av),(ac)*sizeof(t_atom));
				x->av=NULL; ac=0;
		}
		
		}
	}
}

void max_jit_artkmulti_assist(t_max_jit_artkmulti *x, void *b, long m, long a, char *s)
{
	if (m == 1) { //input
		max_jit_mop_assist(x,b,m,a,s);
	} else { //output
		switch (a) {
		case 0:
			sprintf(s,"(list) patt{1}, rot{quat}, pos{axis angle}");
			break; 
		case 1:
			sprintf(s,"(list) patt{2}, rot{quat}, pos{axis angle}");
			break; 	
		case 2:
			sprintf(s,"(list) patt{3}, rot{quat}, pos{axis angle}");
			break; 	
		case 3:
			sprintf(s,"(list) patt{4}, rot{quat}, pos{axis angle}");
			break; 	
		case 4:
			sprintf(s,"dumpout");
			break; 			
		}
	}
}

void max_jit_artkmulti_free(t_max_jit_artkmulti *x)
{
	max_jit_mop_free(x);
	jit_object_free(max_jit_obex_jitob_get(x));
	max_jit_obex_free(x);
}

void *max_jit_artkmulti_new(t_symbol *s, long argc, t_atom *argv)
{
	t_max_jit_artkmulti *x;
	void *o;
		//x->av = NULL;

	if (x=(t_max_jit_artkmulti *)max_jit_obex_new(max_jit_artkmulti_class,gensym("jit_artkmulti"))) {
		if (o=jit_object_new(gensym("jit_artkmulti"))) {
			max_jit_mop_setup_simple(x,o,argc,argv);			
			//add additional non-matrix outputs
			x->mark4 	= outlet_new(x,0L);
			x->mark3 	= outlet_new(x,0L);	
			x->mark2 	= outlet_new(x,0L);
			x->mark1 	= outlet_new(x,0L);
			
			
			x->av		= jit_getbytes(sizeof(t_atom)*JIT_MATRIX_MAX_PLANECOUNT);
			max_jit_attr_args(x,argc,argv);
		} else {
			error("jit.artkmulti: could not allocate object");
			freeobject((t_object *)x);
			x=NULL;
		}
	}
	return (x);
}

