/* zamulticomp.c  ZaMultiComp mono multiband compressor
 * Copyright (C) 2013  Damien Zammit
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <math.h>
#include <stdlib.h>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

#define ZAMULTICOMP_URI "http://zamaudio.com/lv2/zamulticomp"
#define MAX_FILT 8
#define MAX_COMP 3
#define ONEOVERROOT2 0.7071068f
#define ROOT2 1.4142135f

typedef enum {
	ZAMCOMP_INPUT = 0,
	ZAMCOMP_OUTPUT = 1,

	ZAMCOMP_ATTACK1 = 2,
	ZAMCOMP_RELEASE1 = 3,
	ZAMCOMP_KNEE1 = 4,
	ZAMCOMP_RATIO1 = 5,
	ZAMCOMP_THRESHOLD1 = 6,
	ZAMCOMP_MAKEUP1 = 7,
	
	ZAMCOMP_ATTACK2 = 8,
	ZAMCOMP_RELEASE2 = 9,
	ZAMCOMP_KNEE2 = 10,
	ZAMCOMP_RATIO2 = 11,
	ZAMCOMP_THRESHOLD2 = 12,
	ZAMCOMP_MAKEUP2 = 13,
	
	ZAMCOMP_ATTACK3 = 14,
	ZAMCOMP_RELEASE3 = 15,
	ZAMCOMP_KNEE3 = 16,
	ZAMCOMP_RATIO3 = 17,
	ZAMCOMP_THRESHOLD3 = 18,
	ZAMCOMP_MAKEUP3 = 19,
	
	ZAMCOMP_XOVER1 = 20,
	ZAMCOMP_XOVER2 = 21,

	ZAMCOMP_GAINR1 = 22,
	ZAMCOMP_GAINR2 = 23,
	ZAMCOMP_GAINR3 = 24,

	ZAMCOMP_TOGGLE1 = 25,
	ZAMCOMP_TOGGLE2 = 26,
	ZAMCOMP_TOGGLE3 = 27,
	ZAMCOMP_GAINGLOBAL = 28
} PortIndex;


typedef struct {
	float* input;
	float* output;
  
	float* attack1;
	float* release1;
	float* knee1;
	float* ratio1;
	float* threshold1;
	float* makeup1;
	float* gainr1;
	float* toggle1;
 
	float* attack2;
	float* release2;
	float* knee2;
	float* ratio2;
	float* threshold2;
	float* makeup2;
	float* gainr2;
	float* toggle2;

	float* attack3;
	float* release3;
	float* knee3;
	float* ratio3;
	float* threshold3;
	float* makeup3;
	float* gainr3;
	float* toggle3;

	float* xover1;
	float* xover2;

	float* gainglobal;

	float srate;
	float old_yl[MAX_COMP];
	float old_y1[MAX_COMP];

	// Crossover filter coefficients
	float a0[MAX_FILT];
	float a1[MAX_FILT];
	float a2[MAX_FILT];
	float b1[MAX_FILT];
	float b2[MAX_FILT];

	//Crossover filter states
	float w1[MAX_FILT];
	float w2[MAX_FILT];
  
} ZamCOMP;

static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
            double rate,
            const char* bundle_path,
            const LV2_Feature* const* features)
{
	ZamCOMP* zamcomp = (ZamCOMP*)malloc(sizeof(ZamCOMP));
	zamcomp->srate = rate;
  
	int i;
	for (i = 0; i < MAX_COMP; i++) {
		zamcomp->old_yl[i]=zamcomp->old_y1[i]=0.f;
	}
	
	for (i = 0; i < MAX_FILT; i++) {
		zamcomp->a0[i] = zamcomp->a1[i] = zamcomp->a2[i] = 0.f;
		zamcomp->b1[i] = zamcomp->b2[i] = 0.f;
		zamcomp->w1[i] = zamcomp->w2[i] = 0.f;
	}

	return (LV2_Handle)zamcomp;
}

static void
connect_port(LV2_Handle instance,
             uint32_t port,
             void* data)
{
	ZamCOMP* zamcomp = (ZamCOMP*)instance;
  
	switch ((PortIndex)port) {
	case ZAMCOMP_INPUT:
		zamcomp->input = (float*)data;
  	break;
	case ZAMCOMP_OUTPUT:
		zamcomp->output = (float*)data;
  	break;

	case ZAMCOMP_ATTACK1:
		zamcomp->attack1 = (float*)data;
	break;
	case ZAMCOMP_RELEASE1:
		zamcomp->release1 = (float*)data;
	break;
	case ZAMCOMP_KNEE1:
		zamcomp->knee1 = (float*)data;
	break;
	case ZAMCOMP_RATIO1:
		zamcomp->ratio1 = (float*)data;
	break;
	case ZAMCOMP_THRESHOLD1:
		zamcomp->threshold1 = (float*)data;
	break;
	case ZAMCOMP_MAKEUP1:
		zamcomp->makeup1 = (float*)data;
	break;
	case ZAMCOMP_GAINR1:
		zamcomp->gainr1 = (float*)data;
	break;
	case ZAMCOMP_TOGGLE1:
		zamcomp->toggle1 = (float*)data;
	break;

	case ZAMCOMP_ATTACK2:
		zamcomp->attack2 = (float*)data;
	break;
	case ZAMCOMP_RELEASE2:
		zamcomp->release2 = (float*)data;
	break;
	case ZAMCOMP_KNEE2:
		zamcomp->knee2 = (float*)data;
	break;
	case ZAMCOMP_RATIO2:
		zamcomp->ratio2 = (float*)data;
	break;
	case ZAMCOMP_THRESHOLD2:
		zamcomp->threshold2 = (float*)data;
	break;
	case ZAMCOMP_MAKEUP2:
		zamcomp->makeup2 = (float*)data;
	break;
	case ZAMCOMP_GAINR2:
		zamcomp->gainr2 = (float*)data;
	break;
	case ZAMCOMP_TOGGLE2:
		zamcomp->toggle2 = (float*)data;
	break;

	case ZAMCOMP_ATTACK3:
		zamcomp->attack3 = (float*)data;
	break;
	case ZAMCOMP_RELEASE3:
		zamcomp->release3 = (float*)data;
	break;
	case ZAMCOMP_KNEE3:
		zamcomp->knee3 = (float*)data;
	break;
	case ZAMCOMP_RATIO3:
		zamcomp->ratio3 = (float*)data;
	break;
	case ZAMCOMP_THRESHOLD3:
		zamcomp->threshold3 = (float*)data;
	break;
	case ZAMCOMP_MAKEUP3:
		zamcomp->makeup3 = (float*)data;
	break;
	case ZAMCOMP_GAINR3:
		zamcomp->gainr3 = (float*)data;
	break;
	case ZAMCOMP_TOGGLE3:
		zamcomp->toggle3 = (float*)data;
	break;

	case ZAMCOMP_XOVER1:
		zamcomp->xover1 = (float*)data;
	break;
	case ZAMCOMP_XOVER2:
		zamcomp->xover2 = (float*)data;
	break;
	
	case ZAMCOMP_GAINGLOBAL:
		zamcomp->gainglobal = (float*)data;
	break;
	}
}

// Works on little-endian machines only
static inline bool
is_nan(float& value ) {
	if (((*(uint32_t *) &value) & 0x7fffffff) > 0x7f800000) {
		return true;
	}
return false;
}

// Force already-denormal float value to zero
static inline void
sanitize_denormal(float& value) {
	if (is_nan(value)) {
		value = 0.f;
	}
}

static inline float
from_dB(float gdb) {
	return (exp(gdb/20.f*log(10.f)));
};

static inline float
to_dB(float g) {
	return (20.f*log10(g));
}

static void
activate(LV2_Handle instance)
{
}

float run_filter(LV2_Handle lv2, int i, float in)
{
	ZamCOMP* zamcomp = (ZamCOMP*)lv2;	
	sanitize_denormal(in);
	sanitize_denormal(zamcomp->w1[i]); 
	sanitize_denormal(zamcomp->w2[i]);

	float tmp = in - zamcomp->w1[i] * zamcomp->b1[i] - zamcomp->w2[i] * zamcomp->b2[i];
	float out = tmp * zamcomp->a0[i] + zamcomp->w1[i] * zamcomp->a1[i] + zamcomp->w2[i] * zamcomp->a2[i];
	zamcomp->w2[i] = zamcomp->w1[i];
	zamcomp->w1[i] = tmp;
	return out;
}

static
inline void set_lp_coeffs(LV2_Handle lv2, float fc, float q, float sr, int i, float gain = 1.0)
{
	ZamCOMP* zamcomp = (ZamCOMP*)lv2;	
	float omega=(float)(2.f*M_PI*fc/sr);
	float sn=sin(omega);
	float cs=cos(omega);
	float alpha=(float)(sn/(2.f*q));
	float inv=(float)(1.0/(1.0+alpha));

	zamcomp->a2[i] =  zamcomp->a0[i] =  (float)(gain*inv*(1.f - cs)*0.5f);
	zamcomp->a1[i] =  zamcomp->a0[i] + zamcomp->a0[i];
	zamcomp->b1[i] =  (float)(-2.f*cs*inv);
	zamcomp->b2[i] =  (float)((1.f - alpha)*inv);
}

static
inline void set_hp_coeffs(LV2_Handle lv2, float fc, float q, float sr, int i, float gain=1.0)
{
	ZamCOMP* zamcomp = (ZamCOMP*)lv2;	
	float omega=(float)(2.f*M_PI*fc/sr);
	float sn=sin(omega);
	float cs=cos(omega);
	float alpha=(float)(sn/(2.f*q));

	float inv=(float)(1.f/(1.f+alpha));

	zamcomp->a0[i] =  (float)(gain*inv*(1.f + cs)/2.f);
	zamcomp->a1[i] =  -2.f * zamcomp->a0[i];
	zamcomp->a2[i] =  zamcomp->a0[i];
	zamcomp->b1[i] =  (float)(-2.f*cs*inv);
	zamcomp->b2[i] =  (float)((1.f - alpha)*inv);
}

static
inline float run_comp(LV2_Handle lv2, int k, float attack, float release, float knee,
	float ratio, float threshold, float makeup, float* gainr, float in)
{
	ZamCOMP* zamcomp = (ZamCOMP*)lv2;	
	makeup = from_dB(makeup);
	float width=(knee-0.99f)*6.f;
	float attack_coeff = exp(-1000.f/(attack * zamcomp->srate));
	float release_coeff = exp(-1000.f/(release * zamcomp->srate));

	float cdb=0.f;
	float gain = 1.f;
	float xg, xl, yg, yl, y1;
	float out;

	yg=0.f;
	xg = (in==0.f) ? -160.f : to_dB(fabs(in));
	sanitize_denormal(xg);


	if (2.f*(xg-threshold)<-width) {
		yg = xg;
	} else if (2.f*fabs(xg-threshold)<=width) {
		yg = xg + (1.f/ratio-1.f)*(xg-threshold+width/2.f)*(xg-threshold+width/2.f)/(2.f*width);
	} else if (2.f*(xg-threshold)>width) {
		yg = threshold + (xg-threshold)/ratio;
	}

	sanitize_denormal(yg);

	xl = xg - yg;
	sanitize_denormal(zamcomp->old_y1[k]);
	sanitize_denormal(zamcomp->old_yl[k]);

	y1 = fmaxf(xl, release_coeff * zamcomp->old_y1[k]+(1.f-release_coeff)*xl);
	yl = attack_coeff * zamcomp->old_yl[k]+(1.f-attack_coeff)*y1;
	sanitize_denormal(y1);
	sanitize_denormal(yl);

	cdb = -yl;
	gain = from_dB(cdb);

	*gainr = yl;

	out = in;
	out *= gain * makeup;

	zamcomp->old_yl[k] = yl; 
	zamcomp->old_y1[k] = y1;
	return out;
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
	ZamCOMP* zamcomp = (ZamCOMP*)instance;
  
	const float* const input = zamcomp->input;
	float* const output = zamcomp->output;
  
	int toggle1 = (*(zamcomp->toggle1) > 0.f) ? 1 : 0;
	int toggle2 = (*(zamcomp->toggle2) > 0.f) ? 1 : 0;
	int toggle3 = (*(zamcomp->toggle3) > 0.f) ? 1 : 0;

	set_lp_coeffs(zamcomp, *(zamcomp->xover1), ONEOVERROOT2, zamcomp->srate, 0);
	set_lp_coeffs(zamcomp, *(zamcomp->xover1), ONEOVERROOT2, zamcomp->srate, 1);
	set_hp_coeffs(zamcomp, *(zamcomp->xover1), ONEOVERROOT2, zamcomp->srate, 2);
	set_hp_coeffs(zamcomp, *(zamcomp->xover1), ONEOVERROOT2, zamcomp->srate, 3);
	set_lp_coeffs(zamcomp, *(zamcomp->xover2), ONEOVERROOT2, zamcomp->srate, 4);
	set_lp_coeffs(zamcomp, *(zamcomp->xover2), ONEOVERROOT2, zamcomp->srate, 5);
	set_hp_coeffs(zamcomp, *(zamcomp->xover2), ONEOVERROOT2, zamcomp->srate, 6);
	set_hp_coeffs(zamcomp, *(zamcomp->xover2), ONEOVERROOT2, zamcomp->srate, 7);

	for (uint32_t i = 0; i < n_samples; ++i) {

		float tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, fil1, fil2, fil3, fil4;
		output[i] = input[i];

		fil1 = run_filter(zamcomp, 0, input[i]);
		tmp1 = run_filter(zamcomp, 1, fil1);
		tmp2 = toggle1 ? 
			run_comp(zamcomp, 0, 
				*(zamcomp->attack1), 
				*(zamcomp->release1), 
				*(zamcomp->knee1),
				*(zamcomp->ratio1), 
				*(zamcomp->threshold1), 
				*(zamcomp->makeup1), 
				zamcomp->gainr1,
				tmp1) : tmp1;
		fil2 = run_filter(zamcomp, 2, input[i]);
		tmp3 = run_filter(zamcomp, 3, fil2);
		fil3 = run_filter(zamcomp, 4, tmp3);
		tmp4 = run_filter(zamcomp, 5, fil3);
		tmp3 = toggle2 ? 
			run_comp(zamcomp, 1, 
				*(zamcomp->attack2), 
				*(zamcomp->release2), 
				*(zamcomp->knee2),
				*(zamcomp->ratio2), 
				*(zamcomp->threshold2), 
				*(zamcomp->makeup2), 
				zamcomp->gainr2,
				tmp4) : tmp4;
		fil4 = run_filter(zamcomp, 6, input[i]);
		tmp5 = run_filter(zamcomp, 7, fil4);
		tmp6 = toggle3 ? 
			run_comp(zamcomp, 2, 
				*(zamcomp->attack3), 
				*(zamcomp->release3), 
				*(zamcomp->knee3),
				*(zamcomp->ratio3), 
				*(zamcomp->threshold3), 
				*(zamcomp->makeup3), 
				zamcomp->gainr3,
				tmp5) : tmp5;
		output[i] = tmp2 + tmp3 + tmp6;
		output[i] *= from_dB(*(zamcomp->gainglobal));
	}
}

static void
deactivate(LV2_Handle instance)
{
}

static void
cleanup(LV2_Handle instance)
{
	free(instance);
}

const void*
extension_data(const char* uri)
{
	return NULL;
}

static const LV2_Descriptor descriptor = {
	ZAMULTICOMP_URI,
	instantiate,
	connect_port,
	activate,
	run,
	deactivate,
	cleanup,
	extension_data
};

LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
	switch (index) {
	case 0:
		return &descriptor;
	default:
		return NULL;
	}
}
