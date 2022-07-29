/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 16];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 16 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 16 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 16 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 16 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 16 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[lRun1 * 16 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[lRun1 * 16 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[lRun1 * 16 + 8];
nmpcWorkspace.state[9] = nmpcVariables.x[lRun1 * 16 + 9];
nmpcWorkspace.state[10] = nmpcVariables.x[lRun1 * 16 + 10];
nmpcWorkspace.state[11] = nmpcVariables.x[lRun1 * 16 + 11];
nmpcWorkspace.state[12] = nmpcVariables.x[lRun1 * 16 + 12];
nmpcWorkspace.state[13] = nmpcVariables.x[lRun1 * 16 + 13];
nmpcWorkspace.state[14] = nmpcVariables.x[lRun1 * 16 + 14];
nmpcWorkspace.state[15] = nmpcVariables.x[lRun1 * 16 + 15];

nmpcWorkspace.state[336] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[337] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[338] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[339] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.state[340] = nmpcVariables.od[lRun1 * 9];
nmpcWorkspace.state[341] = nmpcVariables.od[lRun1 * 9 + 1];
nmpcWorkspace.state[342] = nmpcVariables.od[lRun1 * 9 + 2];
nmpcWorkspace.state[343] = nmpcVariables.od[lRun1 * 9 + 3];
nmpcWorkspace.state[344] = nmpcVariables.od[lRun1 * 9 + 4];
nmpcWorkspace.state[345] = nmpcVariables.od[lRun1 * 9 + 5];
nmpcWorkspace.state[346] = nmpcVariables.od[lRun1 * 9 + 6];
nmpcWorkspace.state[347] = nmpcVariables.od[lRun1 * 9 + 7];
nmpcWorkspace.state[348] = nmpcVariables.od[lRun1 * 9 + 8];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 16] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 16 + 16];
nmpcWorkspace.d[lRun1 * 16 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 16 + 17];
nmpcWorkspace.d[lRun1 * 16 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 16 + 18];
nmpcWorkspace.d[lRun1 * 16 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 16 + 19];
nmpcWorkspace.d[lRun1 * 16 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 16 + 20];
nmpcWorkspace.d[lRun1 * 16 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 16 + 21];
nmpcWorkspace.d[lRun1 * 16 + 6] = nmpcWorkspace.state[6] - nmpcVariables.x[lRun1 * 16 + 22];
nmpcWorkspace.d[lRun1 * 16 + 7] = nmpcWorkspace.state[7] - nmpcVariables.x[lRun1 * 16 + 23];
nmpcWorkspace.d[lRun1 * 16 + 8] = nmpcWorkspace.state[8] - nmpcVariables.x[lRun1 * 16 + 24];
nmpcWorkspace.d[lRun1 * 16 + 9] = nmpcWorkspace.state[9] - nmpcVariables.x[lRun1 * 16 + 25];
nmpcWorkspace.d[lRun1 * 16 + 10] = nmpcWorkspace.state[10] - nmpcVariables.x[lRun1 * 16 + 26];
nmpcWorkspace.d[lRun1 * 16 + 11] = nmpcWorkspace.state[11] - nmpcVariables.x[lRun1 * 16 + 27];
nmpcWorkspace.d[lRun1 * 16 + 12] = nmpcWorkspace.state[12] - nmpcVariables.x[lRun1 * 16 + 28];
nmpcWorkspace.d[lRun1 * 16 + 13] = nmpcWorkspace.state[13] - nmpcVariables.x[lRun1 * 16 + 29];
nmpcWorkspace.d[lRun1 * 16 + 14] = nmpcWorkspace.state[14] - nmpcVariables.x[lRun1 * 16 + 30];
nmpcWorkspace.d[lRun1 * 16 + 15] = nmpcWorkspace.state[15] - nmpcVariables.x[lRun1 * 16 + 31];

for (lRun2 = 0; lRun2 < 256; ++lRun2)
nmpcWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 256))] = nmpcWorkspace.state[lRun2 + 16];


nmpcWorkspace.evGu[lRun1 * 64] = nmpcWorkspace.state[272];
nmpcWorkspace.evGu[lRun1 * 64 + 1] = nmpcWorkspace.state[273];
nmpcWorkspace.evGu[lRun1 * 64 + 2] = nmpcWorkspace.state[274];
nmpcWorkspace.evGu[lRun1 * 64 + 3] = nmpcWorkspace.state[275];
nmpcWorkspace.evGu[lRun1 * 64 + 4] = nmpcWorkspace.state[276];
nmpcWorkspace.evGu[lRun1 * 64 + 5] = nmpcWorkspace.state[277];
nmpcWorkspace.evGu[lRun1 * 64 + 6] = nmpcWorkspace.state[278];
nmpcWorkspace.evGu[lRun1 * 64 + 7] = nmpcWorkspace.state[279];
nmpcWorkspace.evGu[lRun1 * 64 + 8] = nmpcWorkspace.state[280];
nmpcWorkspace.evGu[lRun1 * 64 + 9] = nmpcWorkspace.state[281];
nmpcWorkspace.evGu[lRun1 * 64 + 10] = nmpcWorkspace.state[282];
nmpcWorkspace.evGu[lRun1 * 64 + 11] = nmpcWorkspace.state[283];
nmpcWorkspace.evGu[lRun1 * 64 + 12] = nmpcWorkspace.state[284];
nmpcWorkspace.evGu[lRun1 * 64 + 13] = nmpcWorkspace.state[285];
nmpcWorkspace.evGu[lRun1 * 64 + 14] = nmpcWorkspace.state[286];
nmpcWorkspace.evGu[lRun1 * 64 + 15] = nmpcWorkspace.state[287];
nmpcWorkspace.evGu[lRun1 * 64 + 16] = nmpcWorkspace.state[288];
nmpcWorkspace.evGu[lRun1 * 64 + 17] = nmpcWorkspace.state[289];
nmpcWorkspace.evGu[lRun1 * 64 + 18] = nmpcWorkspace.state[290];
nmpcWorkspace.evGu[lRun1 * 64 + 19] = nmpcWorkspace.state[291];
nmpcWorkspace.evGu[lRun1 * 64 + 20] = nmpcWorkspace.state[292];
nmpcWorkspace.evGu[lRun1 * 64 + 21] = nmpcWorkspace.state[293];
nmpcWorkspace.evGu[lRun1 * 64 + 22] = nmpcWorkspace.state[294];
nmpcWorkspace.evGu[lRun1 * 64 + 23] = nmpcWorkspace.state[295];
nmpcWorkspace.evGu[lRun1 * 64 + 24] = nmpcWorkspace.state[296];
nmpcWorkspace.evGu[lRun1 * 64 + 25] = nmpcWorkspace.state[297];
nmpcWorkspace.evGu[lRun1 * 64 + 26] = nmpcWorkspace.state[298];
nmpcWorkspace.evGu[lRun1 * 64 + 27] = nmpcWorkspace.state[299];
nmpcWorkspace.evGu[lRun1 * 64 + 28] = nmpcWorkspace.state[300];
nmpcWorkspace.evGu[lRun1 * 64 + 29] = nmpcWorkspace.state[301];
nmpcWorkspace.evGu[lRun1 * 64 + 30] = nmpcWorkspace.state[302];
nmpcWorkspace.evGu[lRun1 * 64 + 31] = nmpcWorkspace.state[303];
nmpcWorkspace.evGu[lRun1 * 64 + 32] = nmpcWorkspace.state[304];
nmpcWorkspace.evGu[lRun1 * 64 + 33] = nmpcWorkspace.state[305];
nmpcWorkspace.evGu[lRun1 * 64 + 34] = nmpcWorkspace.state[306];
nmpcWorkspace.evGu[lRun1 * 64 + 35] = nmpcWorkspace.state[307];
nmpcWorkspace.evGu[lRun1 * 64 + 36] = nmpcWorkspace.state[308];
nmpcWorkspace.evGu[lRun1 * 64 + 37] = nmpcWorkspace.state[309];
nmpcWorkspace.evGu[lRun1 * 64 + 38] = nmpcWorkspace.state[310];
nmpcWorkspace.evGu[lRun1 * 64 + 39] = nmpcWorkspace.state[311];
nmpcWorkspace.evGu[lRun1 * 64 + 40] = nmpcWorkspace.state[312];
nmpcWorkspace.evGu[lRun1 * 64 + 41] = nmpcWorkspace.state[313];
nmpcWorkspace.evGu[lRun1 * 64 + 42] = nmpcWorkspace.state[314];
nmpcWorkspace.evGu[lRun1 * 64 + 43] = nmpcWorkspace.state[315];
nmpcWorkspace.evGu[lRun1 * 64 + 44] = nmpcWorkspace.state[316];
nmpcWorkspace.evGu[lRun1 * 64 + 45] = nmpcWorkspace.state[317];
nmpcWorkspace.evGu[lRun1 * 64 + 46] = nmpcWorkspace.state[318];
nmpcWorkspace.evGu[lRun1 * 64 + 47] = nmpcWorkspace.state[319];
nmpcWorkspace.evGu[lRun1 * 64 + 48] = nmpcWorkspace.state[320];
nmpcWorkspace.evGu[lRun1 * 64 + 49] = nmpcWorkspace.state[321];
nmpcWorkspace.evGu[lRun1 * 64 + 50] = nmpcWorkspace.state[322];
nmpcWorkspace.evGu[lRun1 * 64 + 51] = nmpcWorkspace.state[323];
nmpcWorkspace.evGu[lRun1 * 64 + 52] = nmpcWorkspace.state[324];
nmpcWorkspace.evGu[lRun1 * 64 + 53] = nmpcWorkspace.state[325];
nmpcWorkspace.evGu[lRun1 * 64 + 54] = nmpcWorkspace.state[326];
nmpcWorkspace.evGu[lRun1 * 64 + 55] = nmpcWorkspace.state[327];
nmpcWorkspace.evGu[lRun1 * 64 + 56] = nmpcWorkspace.state[328];
nmpcWorkspace.evGu[lRun1 * 64 + 57] = nmpcWorkspace.state[329];
nmpcWorkspace.evGu[lRun1 * 64 + 58] = nmpcWorkspace.state[330];
nmpcWorkspace.evGu[lRun1 * 64 + 59] = nmpcWorkspace.state[331];
nmpcWorkspace.evGu[lRun1 * 64 + 60] = nmpcWorkspace.state[332];
nmpcWorkspace.evGu[lRun1 * 64 + 61] = nmpcWorkspace.state[333];
nmpcWorkspace.evGu[lRun1 * 64 + 62] = nmpcWorkspace.state[334];
nmpcWorkspace.evGu[lRun1 * 64 + 63] = nmpcWorkspace.state[335];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 16;
const real_t* od = in + 20;
/* Vector of auxiliary variables; number of elements: 80. */
real_t* a = nmpcWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (od[3]-xd[0]);
a[1] = (od[4]-xd[1]);
a[2] = (sqrt(((a[0]*a[0])+(a[1]*a[1]))));
a[3] = (a[2]+(real_t)(1.0000000000000000e-04));
a[4] = (((real_t)(1.0000000000000000e+00)/a[3])*(((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[7])*xd[7]))-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*((xd[6]*xd[7])+(xd[9]*xd[8])))*a[1])));
a[5] = (od[5]-xd[2]);
a[6] = (((od[6]*a[0])+(od[7]*a[1]))+(od[8]*a[5]));
a[7] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[8] = (1.0/sqrt(((a[0]*a[0])+(a[1]*a[1]))));
a[9] = (a[8]*(real_t)(5.0000000000000000e-01));
a[10] = (((a[7]*a[0])+(a[0]*a[7]))*a[9]);
a[11] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[12] = (a[11]*a[11]);
a[13] = ((((real_t)(0.0000000000000000e+00)-(a[10]*a[12]))*(((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[7])*xd[7]))-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*((xd[6]*xd[7])+(xd[9]*xd[8])))*a[1])))+(((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[7])*xd[7]))-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[7])));
a[14] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[15] = (((a[14]*a[1])+(a[1]*a[14]))*a[9]);
a[16] = ((((real_t)(0.0000000000000000e+00)-(a[15]*a[12]))*(((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[7])*xd[7]))-(((real_t)(2.0000000000000000e+00)*xd[8])*xd[8]))*a[0])+(((real_t)(2.0000000000000000e+00)*((xd[6]*xd[7])+(xd[9]*xd[8])))*a[1])))+(((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*((xd[6]*xd[7])+(xd[9]*xd[8])))*a[14])));
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*xd[7])*a[1]));
a[22] = (((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[7])+((real_t)(2.0000000000000000e+00)*xd[7])))*a[0])+(((real_t)(2.0000000000000000e+00)*xd[6])*a[1])));
a[23] = (((real_t)(1.0000000000000000e+00)/a[3])*((((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[8])+((real_t)(2.0000000000000000e+00)*xd[8])))*a[0])+(((real_t)(2.0000000000000000e+00)*xd[9])*a[1])));
a[24] = (((real_t)(1.0000000000000000e+00)/a[3])*(((real_t)(2.0000000000000000e+00)*xd[8])*a[1]));
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[32] = (((a[31]*a[0])+(a[0]*a[31]))*a[9]);
a[33] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[34] = (((a[33]*a[1])+(a[1]*a[33]))*a[9]);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(0.0000000000000000e+00);
a[37] = (real_t)(0.0000000000000000e+00);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (real_t)(0.0000000000000000e+00);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[50] = (od[6]*a[49]);
a[51] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[52] = (od[7]*a[51]);
a[53] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[54] = (od[8]*a[53]);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = (real_t)(0.0000000000000000e+00);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (real_t)(0.0000000000000000e+00);
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (real_t)(0.0000000000000000e+00);
a[69] = (real_t)(0.0000000000000000e+00);
a[70] = (real_t)(0.0000000000000000e+00);
a[71] = (real_t)(0.0000000000000000e+00);
a[72] = (real_t)(0.0000000000000000e+00);
a[73] = (real_t)(0.0000000000000000e+00);
a[74] = (real_t)(0.0000000000000000e+00);
a[75] = (real_t)(0.0000000000000000e+00);
a[76] = (real_t)(0.0000000000000000e+00);
a[77] = (real_t)(0.0000000000000000e+00);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = a[4];
out[11] = a[3];
out[12] = a[6];
out[13] = u[0];
out[14] = u[1];
out[15] = u[2];
out[16] = u[3];
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(1.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(1.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(1.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(1.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(1.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(1.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(1.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(1.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = a[13];
out[178] = a[16];
out[179] = a[17];
out[180] = a[18];
out[181] = a[19];
out[182] = a[20];
out[183] = a[21];
out[184] = a[22];
out[185] = a[23];
out[186] = a[24];
out[187] = a[25];
out[188] = a[26];
out[189] = a[27];
out[190] = a[28];
out[191] = a[29];
out[192] = a[30];
out[193] = a[32];
out[194] = a[34];
out[195] = a[35];
out[196] = a[36];
out[197] = a[37];
out[198] = a[38];
out[199] = a[39];
out[200] = a[40];
out[201] = a[41];
out[202] = a[42];
out[203] = a[43];
out[204] = a[44];
out[205] = a[45];
out[206] = a[46];
out[207] = a[47];
out[208] = a[48];
out[209] = a[50];
out[210] = a[52];
out[211] = a[54];
out[212] = a[55];
out[213] = a[56];
out[214] = a[57];
out[215] = a[58];
out[216] = a[59];
out[217] = a[60];
out[218] = a[61];
out[219] = a[62];
out[220] = a[63];
out[221] = a[64];
out[222] = a[65];
out[223] = a[66];
out[224] = a[67];
out[225] = (real_t)(0.0000000000000000e+00);
out[226] = (real_t)(0.0000000000000000e+00);
out[227] = (real_t)(0.0000000000000000e+00);
out[228] = (real_t)(0.0000000000000000e+00);
out[229] = (real_t)(0.0000000000000000e+00);
out[230] = (real_t)(0.0000000000000000e+00);
out[231] = (real_t)(0.0000000000000000e+00);
out[232] = (real_t)(0.0000000000000000e+00);
out[233] = (real_t)(0.0000000000000000e+00);
out[234] = (real_t)(0.0000000000000000e+00);
out[235] = (real_t)(0.0000000000000000e+00);
out[236] = (real_t)(0.0000000000000000e+00);
out[237] = (real_t)(0.0000000000000000e+00);
out[238] = (real_t)(0.0000000000000000e+00);
out[239] = (real_t)(0.0000000000000000e+00);
out[240] = (real_t)(0.0000000000000000e+00);
out[241] = (real_t)(0.0000000000000000e+00);
out[242] = (real_t)(0.0000000000000000e+00);
out[243] = (real_t)(0.0000000000000000e+00);
out[244] = (real_t)(0.0000000000000000e+00);
out[245] = (real_t)(0.0000000000000000e+00);
out[246] = (real_t)(0.0000000000000000e+00);
out[247] = (real_t)(0.0000000000000000e+00);
out[248] = (real_t)(0.0000000000000000e+00);
out[249] = (real_t)(0.0000000000000000e+00);
out[250] = (real_t)(0.0000000000000000e+00);
out[251] = (real_t)(0.0000000000000000e+00);
out[252] = (real_t)(0.0000000000000000e+00);
out[253] = (real_t)(0.0000000000000000e+00);
out[254] = (real_t)(0.0000000000000000e+00);
out[255] = (real_t)(0.0000000000000000e+00);
out[256] = (real_t)(0.0000000000000000e+00);
out[257] = (real_t)(0.0000000000000000e+00);
out[258] = (real_t)(0.0000000000000000e+00);
out[259] = (real_t)(0.0000000000000000e+00);
out[260] = (real_t)(0.0000000000000000e+00);
out[261] = (real_t)(0.0000000000000000e+00);
out[262] = (real_t)(0.0000000000000000e+00);
out[263] = (real_t)(0.0000000000000000e+00);
out[264] = (real_t)(0.0000000000000000e+00);
out[265] = (real_t)(0.0000000000000000e+00);
out[266] = (real_t)(0.0000000000000000e+00);
out[267] = (real_t)(0.0000000000000000e+00);
out[268] = (real_t)(0.0000000000000000e+00);
out[269] = (real_t)(0.0000000000000000e+00);
out[270] = (real_t)(0.0000000000000000e+00);
out[271] = (real_t)(0.0000000000000000e+00);
out[272] = (real_t)(0.0000000000000000e+00);
out[273] = (real_t)(0.0000000000000000e+00);
out[274] = (real_t)(0.0000000000000000e+00);
out[275] = (real_t)(0.0000000000000000e+00);
out[276] = (real_t)(0.0000000000000000e+00);
out[277] = (real_t)(0.0000000000000000e+00);
out[278] = (real_t)(0.0000000000000000e+00);
out[279] = (real_t)(0.0000000000000000e+00);
out[280] = (real_t)(0.0000000000000000e+00);
out[281] = (real_t)(0.0000000000000000e+00);
out[282] = (real_t)(0.0000000000000000e+00);
out[283] = (real_t)(0.0000000000000000e+00);
out[284] = (real_t)(0.0000000000000000e+00);
out[285] = (real_t)(0.0000000000000000e+00);
out[286] = (real_t)(0.0000000000000000e+00);
out[287] = (real_t)(0.0000000000000000e+00);
out[288] = (real_t)(0.0000000000000000e+00);
out[289] = (real_t)(0.0000000000000000e+00);
out[290] = (real_t)(0.0000000000000000e+00);
out[291] = (real_t)(0.0000000000000000e+00);
out[292] = (real_t)(0.0000000000000000e+00);
out[293] = (real_t)(0.0000000000000000e+00);
out[294] = (real_t)(0.0000000000000000e+00);
out[295] = (real_t)(0.0000000000000000e+00);
out[296] = (real_t)(0.0000000000000000e+00);
out[297] = (real_t)(0.0000000000000000e+00);
out[298] = (real_t)(0.0000000000000000e+00);
out[299] = (real_t)(0.0000000000000000e+00);
out[300] = (real_t)(0.0000000000000000e+00);
out[301] = (real_t)(0.0000000000000000e+00);
out[302] = (real_t)(0.0000000000000000e+00);
out[303] = (real_t)(0.0000000000000000e+00);
out[304] = (real_t)(0.0000000000000000e+00);
out[305] = (real_t)(0.0000000000000000e+00);
out[306] = (real_t)(0.0000000000000000e+00);
out[307] = (real_t)(0.0000000000000000e+00);
out[308] = (real_t)(0.0000000000000000e+00);
out[309] = (real_t)(0.0000000000000000e+00);
out[310] = (real_t)(0.0000000000000000e+00);
out[311] = (real_t)(0.0000000000000000e+00);
out[312] = (real_t)(0.0000000000000000e+00);
out[313] = (real_t)(0.0000000000000000e+00);
out[314] = (real_t)(0.0000000000000000e+00);
out[315] = (real_t)(0.0000000000000000e+00);
out[316] = (real_t)(0.0000000000000000e+00);
out[317] = (real_t)(0.0000000000000000e+00);
out[318] = (real_t)(0.0000000000000000e+00);
out[319] = (real_t)(0.0000000000000000e+00);
out[320] = (real_t)(0.0000000000000000e+00);
out[321] = (real_t)(0.0000000000000000e+00);
out[322] = (real_t)(0.0000000000000000e+00);
out[323] = (real_t)(0.0000000000000000e+00);
out[324] = (real_t)(0.0000000000000000e+00);
out[325] = (real_t)(0.0000000000000000e+00);
out[326] = (real_t)(0.0000000000000000e+00);
out[327] = (real_t)(0.0000000000000000e+00);
out[328] = (real_t)(0.0000000000000000e+00);
out[329] = a[68];
out[330] = a[69];
out[331] = a[70];
out[332] = a[71];
out[333] = a[72];
out[334] = a[73];
out[335] = a[74];
out[336] = a[75];
out[337] = a[76];
out[338] = a[77];
out[339] = a[78];
out[340] = a[79];
out[341] = (real_t)(1.0000000000000000e+00);
out[342] = (real_t)(0.0000000000000000e+00);
out[343] = (real_t)(0.0000000000000000e+00);
out[344] = (real_t)(0.0000000000000000e+00);
out[345] = (real_t)(0.0000000000000000e+00);
out[346] = (real_t)(1.0000000000000000e+00);
out[347] = (real_t)(0.0000000000000000e+00);
out[348] = (real_t)(0.0000000000000000e+00);
out[349] = (real_t)(0.0000000000000000e+00);
out[350] = (real_t)(0.0000000000000000e+00);
out[351] = (real_t)(1.0000000000000000e+00);
out[352] = (real_t)(0.0000000000000000e+00);
out[353] = (real_t)(0.0000000000000000e+00);
out[354] = (real_t)(0.0000000000000000e+00);
out[355] = (real_t)(0.0000000000000000e+00);
out[356] = (real_t)(1.0000000000000000e+00);
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
}

void nmpc_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 17; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 17; ++lRun3)
{
t += + tmpFx[(lRun3 * 16) + (lRun1)]*tmpObjS[(lRun3 * 17) + (lRun2)];
}
tmpQ2[(lRun1 * 17) + (lRun2)] = t;
}
}
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 17; ++lRun3)
{
t += + tmpQ2[(lRun1 * 17) + (lRun3)]*tmpFx[(lRun3 * 16) + (lRun2)];
}
tmpQ1[(lRun1 * 16) + (lRun2)] = t;
}
}
}

void nmpc_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[4]*tmpObjS[17] + tmpFu[8]*tmpObjS[34] + tmpFu[12]*tmpObjS[51] + tmpFu[16]*tmpObjS[68] + tmpFu[20]*tmpObjS[85] + tmpFu[24]*tmpObjS[102] + tmpFu[28]*tmpObjS[119] + tmpFu[32]*tmpObjS[136] + tmpFu[36]*tmpObjS[153] + tmpFu[40]*tmpObjS[170] + tmpFu[44]*tmpObjS[187] + tmpFu[48]*tmpObjS[204] + tmpFu[52]*tmpObjS[221] + tmpFu[56]*tmpObjS[238] + tmpFu[60]*tmpObjS[255] + tmpFu[64]*tmpObjS[272];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[4]*tmpObjS[18] + tmpFu[8]*tmpObjS[35] + tmpFu[12]*tmpObjS[52] + tmpFu[16]*tmpObjS[69] + tmpFu[20]*tmpObjS[86] + tmpFu[24]*tmpObjS[103] + tmpFu[28]*tmpObjS[120] + tmpFu[32]*tmpObjS[137] + tmpFu[36]*tmpObjS[154] + tmpFu[40]*tmpObjS[171] + tmpFu[44]*tmpObjS[188] + tmpFu[48]*tmpObjS[205] + tmpFu[52]*tmpObjS[222] + tmpFu[56]*tmpObjS[239] + tmpFu[60]*tmpObjS[256] + tmpFu[64]*tmpObjS[273];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[4]*tmpObjS[19] + tmpFu[8]*tmpObjS[36] + tmpFu[12]*tmpObjS[53] + tmpFu[16]*tmpObjS[70] + tmpFu[20]*tmpObjS[87] + tmpFu[24]*tmpObjS[104] + tmpFu[28]*tmpObjS[121] + tmpFu[32]*tmpObjS[138] + tmpFu[36]*tmpObjS[155] + tmpFu[40]*tmpObjS[172] + tmpFu[44]*tmpObjS[189] + tmpFu[48]*tmpObjS[206] + tmpFu[52]*tmpObjS[223] + tmpFu[56]*tmpObjS[240] + tmpFu[60]*tmpObjS[257] + tmpFu[64]*tmpObjS[274];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[4]*tmpObjS[20] + tmpFu[8]*tmpObjS[37] + tmpFu[12]*tmpObjS[54] + tmpFu[16]*tmpObjS[71] + tmpFu[20]*tmpObjS[88] + tmpFu[24]*tmpObjS[105] + tmpFu[28]*tmpObjS[122] + tmpFu[32]*tmpObjS[139] + tmpFu[36]*tmpObjS[156] + tmpFu[40]*tmpObjS[173] + tmpFu[44]*tmpObjS[190] + tmpFu[48]*tmpObjS[207] + tmpFu[52]*tmpObjS[224] + tmpFu[56]*tmpObjS[241] + tmpFu[60]*tmpObjS[258] + tmpFu[64]*tmpObjS[275];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[4]*tmpObjS[21] + tmpFu[8]*tmpObjS[38] + tmpFu[12]*tmpObjS[55] + tmpFu[16]*tmpObjS[72] + tmpFu[20]*tmpObjS[89] + tmpFu[24]*tmpObjS[106] + tmpFu[28]*tmpObjS[123] + tmpFu[32]*tmpObjS[140] + tmpFu[36]*tmpObjS[157] + tmpFu[40]*tmpObjS[174] + tmpFu[44]*tmpObjS[191] + tmpFu[48]*tmpObjS[208] + tmpFu[52]*tmpObjS[225] + tmpFu[56]*tmpObjS[242] + tmpFu[60]*tmpObjS[259] + tmpFu[64]*tmpObjS[276];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[4]*tmpObjS[22] + tmpFu[8]*tmpObjS[39] + tmpFu[12]*tmpObjS[56] + tmpFu[16]*tmpObjS[73] + tmpFu[20]*tmpObjS[90] + tmpFu[24]*tmpObjS[107] + tmpFu[28]*tmpObjS[124] + tmpFu[32]*tmpObjS[141] + tmpFu[36]*tmpObjS[158] + tmpFu[40]*tmpObjS[175] + tmpFu[44]*tmpObjS[192] + tmpFu[48]*tmpObjS[209] + tmpFu[52]*tmpObjS[226] + tmpFu[56]*tmpObjS[243] + tmpFu[60]*tmpObjS[260] + tmpFu[64]*tmpObjS[277];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[4]*tmpObjS[23] + tmpFu[8]*tmpObjS[40] + tmpFu[12]*tmpObjS[57] + tmpFu[16]*tmpObjS[74] + tmpFu[20]*tmpObjS[91] + tmpFu[24]*tmpObjS[108] + tmpFu[28]*tmpObjS[125] + tmpFu[32]*tmpObjS[142] + tmpFu[36]*tmpObjS[159] + tmpFu[40]*tmpObjS[176] + tmpFu[44]*tmpObjS[193] + tmpFu[48]*tmpObjS[210] + tmpFu[52]*tmpObjS[227] + tmpFu[56]*tmpObjS[244] + tmpFu[60]*tmpObjS[261] + tmpFu[64]*tmpObjS[278];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[4]*tmpObjS[24] + tmpFu[8]*tmpObjS[41] + tmpFu[12]*tmpObjS[58] + tmpFu[16]*tmpObjS[75] + tmpFu[20]*tmpObjS[92] + tmpFu[24]*tmpObjS[109] + tmpFu[28]*tmpObjS[126] + tmpFu[32]*tmpObjS[143] + tmpFu[36]*tmpObjS[160] + tmpFu[40]*tmpObjS[177] + tmpFu[44]*tmpObjS[194] + tmpFu[48]*tmpObjS[211] + tmpFu[52]*tmpObjS[228] + tmpFu[56]*tmpObjS[245] + tmpFu[60]*tmpObjS[262] + tmpFu[64]*tmpObjS[279];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[4]*tmpObjS[25] + tmpFu[8]*tmpObjS[42] + tmpFu[12]*tmpObjS[59] + tmpFu[16]*tmpObjS[76] + tmpFu[20]*tmpObjS[93] + tmpFu[24]*tmpObjS[110] + tmpFu[28]*tmpObjS[127] + tmpFu[32]*tmpObjS[144] + tmpFu[36]*tmpObjS[161] + tmpFu[40]*tmpObjS[178] + tmpFu[44]*tmpObjS[195] + tmpFu[48]*tmpObjS[212] + tmpFu[52]*tmpObjS[229] + tmpFu[56]*tmpObjS[246] + tmpFu[60]*tmpObjS[263] + tmpFu[64]*tmpObjS[280];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[4]*tmpObjS[26] + tmpFu[8]*tmpObjS[43] + tmpFu[12]*tmpObjS[60] + tmpFu[16]*tmpObjS[77] + tmpFu[20]*tmpObjS[94] + tmpFu[24]*tmpObjS[111] + tmpFu[28]*tmpObjS[128] + tmpFu[32]*tmpObjS[145] + tmpFu[36]*tmpObjS[162] + tmpFu[40]*tmpObjS[179] + tmpFu[44]*tmpObjS[196] + tmpFu[48]*tmpObjS[213] + tmpFu[52]*tmpObjS[230] + tmpFu[56]*tmpObjS[247] + tmpFu[60]*tmpObjS[264] + tmpFu[64]*tmpObjS[281];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[4]*tmpObjS[27] + tmpFu[8]*tmpObjS[44] + tmpFu[12]*tmpObjS[61] + tmpFu[16]*tmpObjS[78] + tmpFu[20]*tmpObjS[95] + tmpFu[24]*tmpObjS[112] + tmpFu[28]*tmpObjS[129] + tmpFu[32]*tmpObjS[146] + tmpFu[36]*tmpObjS[163] + tmpFu[40]*tmpObjS[180] + tmpFu[44]*tmpObjS[197] + tmpFu[48]*tmpObjS[214] + tmpFu[52]*tmpObjS[231] + tmpFu[56]*tmpObjS[248] + tmpFu[60]*tmpObjS[265] + tmpFu[64]*tmpObjS[282];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[4]*tmpObjS[28] + tmpFu[8]*tmpObjS[45] + tmpFu[12]*tmpObjS[62] + tmpFu[16]*tmpObjS[79] + tmpFu[20]*tmpObjS[96] + tmpFu[24]*tmpObjS[113] + tmpFu[28]*tmpObjS[130] + tmpFu[32]*tmpObjS[147] + tmpFu[36]*tmpObjS[164] + tmpFu[40]*tmpObjS[181] + tmpFu[44]*tmpObjS[198] + tmpFu[48]*tmpObjS[215] + tmpFu[52]*tmpObjS[232] + tmpFu[56]*tmpObjS[249] + tmpFu[60]*tmpObjS[266] + tmpFu[64]*tmpObjS[283];
tmpR2[12] = + tmpFu[0]*tmpObjS[12] + tmpFu[4]*tmpObjS[29] + tmpFu[8]*tmpObjS[46] + tmpFu[12]*tmpObjS[63] + tmpFu[16]*tmpObjS[80] + tmpFu[20]*tmpObjS[97] + tmpFu[24]*tmpObjS[114] + tmpFu[28]*tmpObjS[131] + tmpFu[32]*tmpObjS[148] + tmpFu[36]*tmpObjS[165] + tmpFu[40]*tmpObjS[182] + tmpFu[44]*tmpObjS[199] + tmpFu[48]*tmpObjS[216] + tmpFu[52]*tmpObjS[233] + tmpFu[56]*tmpObjS[250] + tmpFu[60]*tmpObjS[267] + tmpFu[64]*tmpObjS[284];
tmpR2[13] = + tmpFu[0]*tmpObjS[13] + tmpFu[4]*tmpObjS[30] + tmpFu[8]*tmpObjS[47] + tmpFu[12]*tmpObjS[64] + tmpFu[16]*tmpObjS[81] + tmpFu[20]*tmpObjS[98] + tmpFu[24]*tmpObjS[115] + tmpFu[28]*tmpObjS[132] + tmpFu[32]*tmpObjS[149] + tmpFu[36]*tmpObjS[166] + tmpFu[40]*tmpObjS[183] + tmpFu[44]*tmpObjS[200] + tmpFu[48]*tmpObjS[217] + tmpFu[52]*tmpObjS[234] + tmpFu[56]*tmpObjS[251] + tmpFu[60]*tmpObjS[268] + tmpFu[64]*tmpObjS[285];
tmpR2[14] = + tmpFu[0]*tmpObjS[14] + tmpFu[4]*tmpObjS[31] + tmpFu[8]*tmpObjS[48] + tmpFu[12]*tmpObjS[65] + tmpFu[16]*tmpObjS[82] + tmpFu[20]*tmpObjS[99] + tmpFu[24]*tmpObjS[116] + tmpFu[28]*tmpObjS[133] + tmpFu[32]*tmpObjS[150] + tmpFu[36]*tmpObjS[167] + tmpFu[40]*tmpObjS[184] + tmpFu[44]*tmpObjS[201] + tmpFu[48]*tmpObjS[218] + tmpFu[52]*tmpObjS[235] + tmpFu[56]*tmpObjS[252] + tmpFu[60]*tmpObjS[269] + tmpFu[64]*tmpObjS[286];
tmpR2[15] = + tmpFu[0]*tmpObjS[15] + tmpFu[4]*tmpObjS[32] + tmpFu[8]*tmpObjS[49] + tmpFu[12]*tmpObjS[66] + tmpFu[16]*tmpObjS[83] + tmpFu[20]*tmpObjS[100] + tmpFu[24]*tmpObjS[117] + tmpFu[28]*tmpObjS[134] + tmpFu[32]*tmpObjS[151] + tmpFu[36]*tmpObjS[168] + tmpFu[40]*tmpObjS[185] + tmpFu[44]*tmpObjS[202] + tmpFu[48]*tmpObjS[219] + tmpFu[52]*tmpObjS[236] + tmpFu[56]*tmpObjS[253] + tmpFu[60]*tmpObjS[270] + tmpFu[64]*tmpObjS[287];
tmpR2[16] = + tmpFu[0]*tmpObjS[16] + tmpFu[4]*tmpObjS[33] + tmpFu[8]*tmpObjS[50] + tmpFu[12]*tmpObjS[67] + tmpFu[16]*tmpObjS[84] + tmpFu[20]*tmpObjS[101] + tmpFu[24]*tmpObjS[118] + tmpFu[28]*tmpObjS[135] + tmpFu[32]*tmpObjS[152] + tmpFu[36]*tmpObjS[169] + tmpFu[40]*tmpObjS[186] + tmpFu[44]*tmpObjS[203] + tmpFu[48]*tmpObjS[220] + tmpFu[52]*tmpObjS[237] + tmpFu[56]*tmpObjS[254] + tmpFu[60]*tmpObjS[271] + tmpFu[64]*tmpObjS[288];
tmpR2[17] = + tmpFu[1]*tmpObjS[0] + tmpFu[5]*tmpObjS[17] + tmpFu[9]*tmpObjS[34] + tmpFu[13]*tmpObjS[51] + tmpFu[17]*tmpObjS[68] + tmpFu[21]*tmpObjS[85] + tmpFu[25]*tmpObjS[102] + tmpFu[29]*tmpObjS[119] + tmpFu[33]*tmpObjS[136] + tmpFu[37]*tmpObjS[153] + tmpFu[41]*tmpObjS[170] + tmpFu[45]*tmpObjS[187] + tmpFu[49]*tmpObjS[204] + tmpFu[53]*tmpObjS[221] + tmpFu[57]*tmpObjS[238] + tmpFu[61]*tmpObjS[255] + tmpFu[65]*tmpObjS[272];
tmpR2[18] = + tmpFu[1]*tmpObjS[1] + tmpFu[5]*tmpObjS[18] + tmpFu[9]*tmpObjS[35] + tmpFu[13]*tmpObjS[52] + tmpFu[17]*tmpObjS[69] + tmpFu[21]*tmpObjS[86] + tmpFu[25]*tmpObjS[103] + tmpFu[29]*tmpObjS[120] + tmpFu[33]*tmpObjS[137] + tmpFu[37]*tmpObjS[154] + tmpFu[41]*tmpObjS[171] + tmpFu[45]*tmpObjS[188] + tmpFu[49]*tmpObjS[205] + tmpFu[53]*tmpObjS[222] + tmpFu[57]*tmpObjS[239] + tmpFu[61]*tmpObjS[256] + tmpFu[65]*tmpObjS[273];
tmpR2[19] = + tmpFu[1]*tmpObjS[2] + tmpFu[5]*tmpObjS[19] + tmpFu[9]*tmpObjS[36] + tmpFu[13]*tmpObjS[53] + tmpFu[17]*tmpObjS[70] + tmpFu[21]*tmpObjS[87] + tmpFu[25]*tmpObjS[104] + tmpFu[29]*tmpObjS[121] + tmpFu[33]*tmpObjS[138] + tmpFu[37]*tmpObjS[155] + tmpFu[41]*tmpObjS[172] + tmpFu[45]*tmpObjS[189] + tmpFu[49]*tmpObjS[206] + tmpFu[53]*tmpObjS[223] + tmpFu[57]*tmpObjS[240] + tmpFu[61]*tmpObjS[257] + tmpFu[65]*tmpObjS[274];
tmpR2[20] = + tmpFu[1]*tmpObjS[3] + tmpFu[5]*tmpObjS[20] + tmpFu[9]*tmpObjS[37] + tmpFu[13]*tmpObjS[54] + tmpFu[17]*tmpObjS[71] + tmpFu[21]*tmpObjS[88] + tmpFu[25]*tmpObjS[105] + tmpFu[29]*tmpObjS[122] + tmpFu[33]*tmpObjS[139] + tmpFu[37]*tmpObjS[156] + tmpFu[41]*tmpObjS[173] + tmpFu[45]*tmpObjS[190] + tmpFu[49]*tmpObjS[207] + tmpFu[53]*tmpObjS[224] + tmpFu[57]*tmpObjS[241] + tmpFu[61]*tmpObjS[258] + tmpFu[65]*tmpObjS[275];
tmpR2[21] = + tmpFu[1]*tmpObjS[4] + tmpFu[5]*tmpObjS[21] + tmpFu[9]*tmpObjS[38] + tmpFu[13]*tmpObjS[55] + tmpFu[17]*tmpObjS[72] + tmpFu[21]*tmpObjS[89] + tmpFu[25]*tmpObjS[106] + tmpFu[29]*tmpObjS[123] + tmpFu[33]*tmpObjS[140] + tmpFu[37]*tmpObjS[157] + tmpFu[41]*tmpObjS[174] + tmpFu[45]*tmpObjS[191] + tmpFu[49]*tmpObjS[208] + tmpFu[53]*tmpObjS[225] + tmpFu[57]*tmpObjS[242] + tmpFu[61]*tmpObjS[259] + tmpFu[65]*tmpObjS[276];
tmpR2[22] = + tmpFu[1]*tmpObjS[5] + tmpFu[5]*tmpObjS[22] + tmpFu[9]*tmpObjS[39] + tmpFu[13]*tmpObjS[56] + tmpFu[17]*tmpObjS[73] + tmpFu[21]*tmpObjS[90] + tmpFu[25]*tmpObjS[107] + tmpFu[29]*tmpObjS[124] + tmpFu[33]*tmpObjS[141] + tmpFu[37]*tmpObjS[158] + tmpFu[41]*tmpObjS[175] + tmpFu[45]*tmpObjS[192] + tmpFu[49]*tmpObjS[209] + tmpFu[53]*tmpObjS[226] + tmpFu[57]*tmpObjS[243] + tmpFu[61]*tmpObjS[260] + tmpFu[65]*tmpObjS[277];
tmpR2[23] = + tmpFu[1]*tmpObjS[6] + tmpFu[5]*tmpObjS[23] + tmpFu[9]*tmpObjS[40] + tmpFu[13]*tmpObjS[57] + tmpFu[17]*tmpObjS[74] + tmpFu[21]*tmpObjS[91] + tmpFu[25]*tmpObjS[108] + tmpFu[29]*tmpObjS[125] + tmpFu[33]*tmpObjS[142] + tmpFu[37]*tmpObjS[159] + tmpFu[41]*tmpObjS[176] + tmpFu[45]*tmpObjS[193] + tmpFu[49]*tmpObjS[210] + tmpFu[53]*tmpObjS[227] + tmpFu[57]*tmpObjS[244] + tmpFu[61]*tmpObjS[261] + tmpFu[65]*tmpObjS[278];
tmpR2[24] = + tmpFu[1]*tmpObjS[7] + tmpFu[5]*tmpObjS[24] + tmpFu[9]*tmpObjS[41] + tmpFu[13]*tmpObjS[58] + tmpFu[17]*tmpObjS[75] + tmpFu[21]*tmpObjS[92] + tmpFu[25]*tmpObjS[109] + tmpFu[29]*tmpObjS[126] + tmpFu[33]*tmpObjS[143] + tmpFu[37]*tmpObjS[160] + tmpFu[41]*tmpObjS[177] + tmpFu[45]*tmpObjS[194] + tmpFu[49]*tmpObjS[211] + tmpFu[53]*tmpObjS[228] + tmpFu[57]*tmpObjS[245] + tmpFu[61]*tmpObjS[262] + tmpFu[65]*tmpObjS[279];
tmpR2[25] = + tmpFu[1]*tmpObjS[8] + tmpFu[5]*tmpObjS[25] + tmpFu[9]*tmpObjS[42] + tmpFu[13]*tmpObjS[59] + tmpFu[17]*tmpObjS[76] + tmpFu[21]*tmpObjS[93] + tmpFu[25]*tmpObjS[110] + tmpFu[29]*tmpObjS[127] + tmpFu[33]*tmpObjS[144] + tmpFu[37]*tmpObjS[161] + tmpFu[41]*tmpObjS[178] + tmpFu[45]*tmpObjS[195] + tmpFu[49]*tmpObjS[212] + tmpFu[53]*tmpObjS[229] + tmpFu[57]*tmpObjS[246] + tmpFu[61]*tmpObjS[263] + tmpFu[65]*tmpObjS[280];
tmpR2[26] = + tmpFu[1]*tmpObjS[9] + tmpFu[5]*tmpObjS[26] + tmpFu[9]*tmpObjS[43] + tmpFu[13]*tmpObjS[60] + tmpFu[17]*tmpObjS[77] + tmpFu[21]*tmpObjS[94] + tmpFu[25]*tmpObjS[111] + tmpFu[29]*tmpObjS[128] + tmpFu[33]*tmpObjS[145] + tmpFu[37]*tmpObjS[162] + tmpFu[41]*tmpObjS[179] + tmpFu[45]*tmpObjS[196] + tmpFu[49]*tmpObjS[213] + tmpFu[53]*tmpObjS[230] + tmpFu[57]*tmpObjS[247] + tmpFu[61]*tmpObjS[264] + tmpFu[65]*tmpObjS[281];
tmpR2[27] = + tmpFu[1]*tmpObjS[10] + tmpFu[5]*tmpObjS[27] + tmpFu[9]*tmpObjS[44] + tmpFu[13]*tmpObjS[61] + tmpFu[17]*tmpObjS[78] + tmpFu[21]*tmpObjS[95] + tmpFu[25]*tmpObjS[112] + tmpFu[29]*tmpObjS[129] + tmpFu[33]*tmpObjS[146] + tmpFu[37]*tmpObjS[163] + tmpFu[41]*tmpObjS[180] + tmpFu[45]*tmpObjS[197] + tmpFu[49]*tmpObjS[214] + tmpFu[53]*tmpObjS[231] + tmpFu[57]*tmpObjS[248] + tmpFu[61]*tmpObjS[265] + tmpFu[65]*tmpObjS[282];
tmpR2[28] = + tmpFu[1]*tmpObjS[11] + tmpFu[5]*tmpObjS[28] + tmpFu[9]*tmpObjS[45] + tmpFu[13]*tmpObjS[62] + tmpFu[17]*tmpObjS[79] + tmpFu[21]*tmpObjS[96] + tmpFu[25]*tmpObjS[113] + tmpFu[29]*tmpObjS[130] + tmpFu[33]*tmpObjS[147] + tmpFu[37]*tmpObjS[164] + tmpFu[41]*tmpObjS[181] + tmpFu[45]*tmpObjS[198] + tmpFu[49]*tmpObjS[215] + tmpFu[53]*tmpObjS[232] + tmpFu[57]*tmpObjS[249] + tmpFu[61]*tmpObjS[266] + tmpFu[65]*tmpObjS[283];
tmpR2[29] = + tmpFu[1]*tmpObjS[12] + tmpFu[5]*tmpObjS[29] + tmpFu[9]*tmpObjS[46] + tmpFu[13]*tmpObjS[63] + tmpFu[17]*tmpObjS[80] + tmpFu[21]*tmpObjS[97] + tmpFu[25]*tmpObjS[114] + tmpFu[29]*tmpObjS[131] + tmpFu[33]*tmpObjS[148] + tmpFu[37]*tmpObjS[165] + tmpFu[41]*tmpObjS[182] + tmpFu[45]*tmpObjS[199] + tmpFu[49]*tmpObjS[216] + tmpFu[53]*tmpObjS[233] + tmpFu[57]*tmpObjS[250] + tmpFu[61]*tmpObjS[267] + tmpFu[65]*tmpObjS[284];
tmpR2[30] = + tmpFu[1]*tmpObjS[13] + tmpFu[5]*tmpObjS[30] + tmpFu[9]*tmpObjS[47] + tmpFu[13]*tmpObjS[64] + tmpFu[17]*tmpObjS[81] + tmpFu[21]*tmpObjS[98] + tmpFu[25]*tmpObjS[115] + tmpFu[29]*tmpObjS[132] + tmpFu[33]*tmpObjS[149] + tmpFu[37]*tmpObjS[166] + tmpFu[41]*tmpObjS[183] + tmpFu[45]*tmpObjS[200] + tmpFu[49]*tmpObjS[217] + tmpFu[53]*tmpObjS[234] + tmpFu[57]*tmpObjS[251] + tmpFu[61]*tmpObjS[268] + tmpFu[65]*tmpObjS[285];
tmpR2[31] = + tmpFu[1]*tmpObjS[14] + tmpFu[5]*tmpObjS[31] + tmpFu[9]*tmpObjS[48] + tmpFu[13]*tmpObjS[65] + tmpFu[17]*tmpObjS[82] + tmpFu[21]*tmpObjS[99] + tmpFu[25]*tmpObjS[116] + tmpFu[29]*tmpObjS[133] + tmpFu[33]*tmpObjS[150] + tmpFu[37]*tmpObjS[167] + tmpFu[41]*tmpObjS[184] + tmpFu[45]*tmpObjS[201] + tmpFu[49]*tmpObjS[218] + tmpFu[53]*tmpObjS[235] + tmpFu[57]*tmpObjS[252] + tmpFu[61]*tmpObjS[269] + tmpFu[65]*tmpObjS[286];
tmpR2[32] = + tmpFu[1]*tmpObjS[15] + tmpFu[5]*tmpObjS[32] + tmpFu[9]*tmpObjS[49] + tmpFu[13]*tmpObjS[66] + tmpFu[17]*tmpObjS[83] + tmpFu[21]*tmpObjS[100] + tmpFu[25]*tmpObjS[117] + tmpFu[29]*tmpObjS[134] + tmpFu[33]*tmpObjS[151] + tmpFu[37]*tmpObjS[168] + tmpFu[41]*tmpObjS[185] + tmpFu[45]*tmpObjS[202] + tmpFu[49]*tmpObjS[219] + tmpFu[53]*tmpObjS[236] + tmpFu[57]*tmpObjS[253] + tmpFu[61]*tmpObjS[270] + tmpFu[65]*tmpObjS[287];
tmpR2[33] = + tmpFu[1]*tmpObjS[16] + tmpFu[5]*tmpObjS[33] + tmpFu[9]*tmpObjS[50] + tmpFu[13]*tmpObjS[67] + tmpFu[17]*tmpObjS[84] + tmpFu[21]*tmpObjS[101] + tmpFu[25]*tmpObjS[118] + tmpFu[29]*tmpObjS[135] + tmpFu[33]*tmpObjS[152] + tmpFu[37]*tmpObjS[169] + tmpFu[41]*tmpObjS[186] + tmpFu[45]*tmpObjS[203] + tmpFu[49]*tmpObjS[220] + tmpFu[53]*tmpObjS[237] + tmpFu[57]*tmpObjS[254] + tmpFu[61]*tmpObjS[271] + tmpFu[65]*tmpObjS[288];
tmpR2[34] = + tmpFu[2]*tmpObjS[0] + tmpFu[6]*tmpObjS[17] + tmpFu[10]*tmpObjS[34] + tmpFu[14]*tmpObjS[51] + tmpFu[18]*tmpObjS[68] + tmpFu[22]*tmpObjS[85] + tmpFu[26]*tmpObjS[102] + tmpFu[30]*tmpObjS[119] + tmpFu[34]*tmpObjS[136] + tmpFu[38]*tmpObjS[153] + tmpFu[42]*tmpObjS[170] + tmpFu[46]*tmpObjS[187] + tmpFu[50]*tmpObjS[204] + tmpFu[54]*tmpObjS[221] + tmpFu[58]*tmpObjS[238] + tmpFu[62]*tmpObjS[255] + tmpFu[66]*tmpObjS[272];
tmpR2[35] = + tmpFu[2]*tmpObjS[1] + tmpFu[6]*tmpObjS[18] + tmpFu[10]*tmpObjS[35] + tmpFu[14]*tmpObjS[52] + tmpFu[18]*tmpObjS[69] + tmpFu[22]*tmpObjS[86] + tmpFu[26]*tmpObjS[103] + tmpFu[30]*tmpObjS[120] + tmpFu[34]*tmpObjS[137] + tmpFu[38]*tmpObjS[154] + tmpFu[42]*tmpObjS[171] + tmpFu[46]*tmpObjS[188] + tmpFu[50]*tmpObjS[205] + tmpFu[54]*tmpObjS[222] + tmpFu[58]*tmpObjS[239] + tmpFu[62]*tmpObjS[256] + tmpFu[66]*tmpObjS[273];
tmpR2[36] = + tmpFu[2]*tmpObjS[2] + tmpFu[6]*tmpObjS[19] + tmpFu[10]*tmpObjS[36] + tmpFu[14]*tmpObjS[53] + tmpFu[18]*tmpObjS[70] + tmpFu[22]*tmpObjS[87] + tmpFu[26]*tmpObjS[104] + tmpFu[30]*tmpObjS[121] + tmpFu[34]*tmpObjS[138] + tmpFu[38]*tmpObjS[155] + tmpFu[42]*tmpObjS[172] + tmpFu[46]*tmpObjS[189] + tmpFu[50]*tmpObjS[206] + tmpFu[54]*tmpObjS[223] + tmpFu[58]*tmpObjS[240] + tmpFu[62]*tmpObjS[257] + tmpFu[66]*tmpObjS[274];
tmpR2[37] = + tmpFu[2]*tmpObjS[3] + tmpFu[6]*tmpObjS[20] + tmpFu[10]*tmpObjS[37] + tmpFu[14]*tmpObjS[54] + tmpFu[18]*tmpObjS[71] + tmpFu[22]*tmpObjS[88] + tmpFu[26]*tmpObjS[105] + tmpFu[30]*tmpObjS[122] + tmpFu[34]*tmpObjS[139] + tmpFu[38]*tmpObjS[156] + tmpFu[42]*tmpObjS[173] + tmpFu[46]*tmpObjS[190] + tmpFu[50]*tmpObjS[207] + tmpFu[54]*tmpObjS[224] + tmpFu[58]*tmpObjS[241] + tmpFu[62]*tmpObjS[258] + tmpFu[66]*tmpObjS[275];
tmpR2[38] = + tmpFu[2]*tmpObjS[4] + tmpFu[6]*tmpObjS[21] + tmpFu[10]*tmpObjS[38] + tmpFu[14]*tmpObjS[55] + tmpFu[18]*tmpObjS[72] + tmpFu[22]*tmpObjS[89] + tmpFu[26]*tmpObjS[106] + tmpFu[30]*tmpObjS[123] + tmpFu[34]*tmpObjS[140] + tmpFu[38]*tmpObjS[157] + tmpFu[42]*tmpObjS[174] + tmpFu[46]*tmpObjS[191] + tmpFu[50]*tmpObjS[208] + tmpFu[54]*tmpObjS[225] + tmpFu[58]*tmpObjS[242] + tmpFu[62]*tmpObjS[259] + tmpFu[66]*tmpObjS[276];
tmpR2[39] = + tmpFu[2]*tmpObjS[5] + tmpFu[6]*tmpObjS[22] + tmpFu[10]*tmpObjS[39] + tmpFu[14]*tmpObjS[56] + tmpFu[18]*tmpObjS[73] + tmpFu[22]*tmpObjS[90] + tmpFu[26]*tmpObjS[107] + tmpFu[30]*tmpObjS[124] + tmpFu[34]*tmpObjS[141] + tmpFu[38]*tmpObjS[158] + tmpFu[42]*tmpObjS[175] + tmpFu[46]*tmpObjS[192] + tmpFu[50]*tmpObjS[209] + tmpFu[54]*tmpObjS[226] + tmpFu[58]*tmpObjS[243] + tmpFu[62]*tmpObjS[260] + tmpFu[66]*tmpObjS[277];
tmpR2[40] = + tmpFu[2]*tmpObjS[6] + tmpFu[6]*tmpObjS[23] + tmpFu[10]*tmpObjS[40] + tmpFu[14]*tmpObjS[57] + tmpFu[18]*tmpObjS[74] + tmpFu[22]*tmpObjS[91] + tmpFu[26]*tmpObjS[108] + tmpFu[30]*tmpObjS[125] + tmpFu[34]*tmpObjS[142] + tmpFu[38]*tmpObjS[159] + tmpFu[42]*tmpObjS[176] + tmpFu[46]*tmpObjS[193] + tmpFu[50]*tmpObjS[210] + tmpFu[54]*tmpObjS[227] + tmpFu[58]*tmpObjS[244] + tmpFu[62]*tmpObjS[261] + tmpFu[66]*tmpObjS[278];
tmpR2[41] = + tmpFu[2]*tmpObjS[7] + tmpFu[6]*tmpObjS[24] + tmpFu[10]*tmpObjS[41] + tmpFu[14]*tmpObjS[58] + tmpFu[18]*tmpObjS[75] + tmpFu[22]*tmpObjS[92] + tmpFu[26]*tmpObjS[109] + tmpFu[30]*tmpObjS[126] + tmpFu[34]*tmpObjS[143] + tmpFu[38]*tmpObjS[160] + tmpFu[42]*tmpObjS[177] + tmpFu[46]*tmpObjS[194] + tmpFu[50]*tmpObjS[211] + tmpFu[54]*tmpObjS[228] + tmpFu[58]*tmpObjS[245] + tmpFu[62]*tmpObjS[262] + tmpFu[66]*tmpObjS[279];
tmpR2[42] = + tmpFu[2]*tmpObjS[8] + tmpFu[6]*tmpObjS[25] + tmpFu[10]*tmpObjS[42] + tmpFu[14]*tmpObjS[59] + tmpFu[18]*tmpObjS[76] + tmpFu[22]*tmpObjS[93] + tmpFu[26]*tmpObjS[110] + tmpFu[30]*tmpObjS[127] + tmpFu[34]*tmpObjS[144] + tmpFu[38]*tmpObjS[161] + tmpFu[42]*tmpObjS[178] + tmpFu[46]*tmpObjS[195] + tmpFu[50]*tmpObjS[212] + tmpFu[54]*tmpObjS[229] + tmpFu[58]*tmpObjS[246] + tmpFu[62]*tmpObjS[263] + tmpFu[66]*tmpObjS[280];
tmpR2[43] = + tmpFu[2]*tmpObjS[9] + tmpFu[6]*tmpObjS[26] + tmpFu[10]*tmpObjS[43] + tmpFu[14]*tmpObjS[60] + tmpFu[18]*tmpObjS[77] + tmpFu[22]*tmpObjS[94] + tmpFu[26]*tmpObjS[111] + tmpFu[30]*tmpObjS[128] + tmpFu[34]*tmpObjS[145] + tmpFu[38]*tmpObjS[162] + tmpFu[42]*tmpObjS[179] + tmpFu[46]*tmpObjS[196] + tmpFu[50]*tmpObjS[213] + tmpFu[54]*tmpObjS[230] + tmpFu[58]*tmpObjS[247] + tmpFu[62]*tmpObjS[264] + tmpFu[66]*tmpObjS[281];
tmpR2[44] = + tmpFu[2]*tmpObjS[10] + tmpFu[6]*tmpObjS[27] + tmpFu[10]*tmpObjS[44] + tmpFu[14]*tmpObjS[61] + tmpFu[18]*tmpObjS[78] + tmpFu[22]*tmpObjS[95] + tmpFu[26]*tmpObjS[112] + tmpFu[30]*tmpObjS[129] + tmpFu[34]*tmpObjS[146] + tmpFu[38]*tmpObjS[163] + tmpFu[42]*tmpObjS[180] + tmpFu[46]*tmpObjS[197] + tmpFu[50]*tmpObjS[214] + tmpFu[54]*tmpObjS[231] + tmpFu[58]*tmpObjS[248] + tmpFu[62]*tmpObjS[265] + tmpFu[66]*tmpObjS[282];
tmpR2[45] = + tmpFu[2]*tmpObjS[11] + tmpFu[6]*tmpObjS[28] + tmpFu[10]*tmpObjS[45] + tmpFu[14]*tmpObjS[62] + tmpFu[18]*tmpObjS[79] + tmpFu[22]*tmpObjS[96] + tmpFu[26]*tmpObjS[113] + tmpFu[30]*tmpObjS[130] + tmpFu[34]*tmpObjS[147] + tmpFu[38]*tmpObjS[164] + tmpFu[42]*tmpObjS[181] + tmpFu[46]*tmpObjS[198] + tmpFu[50]*tmpObjS[215] + tmpFu[54]*tmpObjS[232] + tmpFu[58]*tmpObjS[249] + tmpFu[62]*tmpObjS[266] + tmpFu[66]*tmpObjS[283];
tmpR2[46] = + tmpFu[2]*tmpObjS[12] + tmpFu[6]*tmpObjS[29] + tmpFu[10]*tmpObjS[46] + tmpFu[14]*tmpObjS[63] + tmpFu[18]*tmpObjS[80] + tmpFu[22]*tmpObjS[97] + tmpFu[26]*tmpObjS[114] + tmpFu[30]*tmpObjS[131] + tmpFu[34]*tmpObjS[148] + tmpFu[38]*tmpObjS[165] + tmpFu[42]*tmpObjS[182] + tmpFu[46]*tmpObjS[199] + tmpFu[50]*tmpObjS[216] + tmpFu[54]*tmpObjS[233] + tmpFu[58]*tmpObjS[250] + tmpFu[62]*tmpObjS[267] + tmpFu[66]*tmpObjS[284];
tmpR2[47] = + tmpFu[2]*tmpObjS[13] + tmpFu[6]*tmpObjS[30] + tmpFu[10]*tmpObjS[47] + tmpFu[14]*tmpObjS[64] + tmpFu[18]*tmpObjS[81] + tmpFu[22]*tmpObjS[98] + tmpFu[26]*tmpObjS[115] + tmpFu[30]*tmpObjS[132] + tmpFu[34]*tmpObjS[149] + tmpFu[38]*tmpObjS[166] + tmpFu[42]*tmpObjS[183] + tmpFu[46]*tmpObjS[200] + tmpFu[50]*tmpObjS[217] + tmpFu[54]*tmpObjS[234] + tmpFu[58]*tmpObjS[251] + tmpFu[62]*tmpObjS[268] + tmpFu[66]*tmpObjS[285];
tmpR2[48] = + tmpFu[2]*tmpObjS[14] + tmpFu[6]*tmpObjS[31] + tmpFu[10]*tmpObjS[48] + tmpFu[14]*tmpObjS[65] + tmpFu[18]*tmpObjS[82] + tmpFu[22]*tmpObjS[99] + tmpFu[26]*tmpObjS[116] + tmpFu[30]*tmpObjS[133] + tmpFu[34]*tmpObjS[150] + tmpFu[38]*tmpObjS[167] + tmpFu[42]*tmpObjS[184] + tmpFu[46]*tmpObjS[201] + tmpFu[50]*tmpObjS[218] + tmpFu[54]*tmpObjS[235] + tmpFu[58]*tmpObjS[252] + tmpFu[62]*tmpObjS[269] + tmpFu[66]*tmpObjS[286];
tmpR2[49] = + tmpFu[2]*tmpObjS[15] + tmpFu[6]*tmpObjS[32] + tmpFu[10]*tmpObjS[49] + tmpFu[14]*tmpObjS[66] + tmpFu[18]*tmpObjS[83] + tmpFu[22]*tmpObjS[100] + tmpFu[26]*tmpObjS[117] + tmpFu[30]*tmpObjS[134] + tmpFu[34]*tmpObjS[151] + tmpFu[38]*tmpObjS[168] + tmpFu[42]*tmpObjS[185] + tmpFu[46]*tmpObjS[202] + tmpFu[50]*tmpObjS[219] + tmpFu[54]*tmpObjS[236] + tmpFu[58]*tmpObjS[253] + tmpFu[62]*tmpObjS[270] + tmpFu[66]*tmpObjS[287];
tmpR2[50] = + tmpFu[2]*tmpObjS[16] + tmpFu[6]*tmpObjS[33] + tmpFu[10]*tmpObjS[50] + tmpFu[14]*tmpObjS[67] + tmpFu[18]*tmpObjS[84] + tmpFu[22]*tmpObjS[101] + tmpFu[26]*tmpObjS[118] + tmpFu[30]*tmpObjS[135] + tmpFu[34]*tmpObjS[152] + tmpFu[38]*tmpObjS[169] + tmpFu[42]*tmpObjS[186] + tmpFu[46]*tmpObjS[203] + tmpFu[50]*tmpObjS[220] + tmpFu[54]*tmpObjS[237] + tmpFu[58]*tmpObjS[254] + tmpFu[62]*tmpObjS[271] + tmpFu[66]*tmpObjS[288];
tmpR2[51] = + tmpFu[3]*tmpObjS[0] + tmpFu[7]*tmpObjS[17] + tmpFu[11]*tmpObjS[34] + tmpFu[15]*tmpObjS[51] + tmpFu[19]*tmpObjS[68] + tmpFu[23]*tmpObjS[85] + tmpFu[27]*tmpObjS[102] + tmpFu[31]*tmpObjS[119] + tmpFu[35]*tmpObjS[136] + tmpFu[39]*tmpObjS[153] + tmpFu[43]*tmpObjS[170] + tmpFu[47]*tmpObjS[187] + tmpFu[51]*tmpObjS[204] + tmpFu[55]*tmpObjS[221] + tmpFu[59]*tmpObjS[238] + tmpFu[63]*tmpObjS[255] + tmpFu[67]*tmpObjS[272];
tmpR2[52] = + tmpFu[3]*tmpObjS[1] + tmpFu[7]*tmpObjS[18] + tmpFu[11]*tmpObjS[35] + tmpFu[15]*tmpObjS[52] + tmpFu[19]*tmpObjS[69] + tmpFu[23]*tmpObjS[86] + tmpFu[27]*tmpObjS[103] + tmpFu[31]*tmpObjS[120] + tmpFu[35]*tmpObjS[137] + tmpFu[39]*tmpObjS[154] + tmpFu[43]*tmpObjS[171] + tmpFu[47]*tmpObjS[188] + tmpFu[51]*tmpObjS[205] + tmpFu[55]*tmpObjS[222] + tmpFu[59]*tmpObjS[239] + tmpFu[63]*tmpObjS[256] + tmpFu[67]*tmpObjS[273];
tmpR2[53] = + tmpFu[3]*tmpObjS[2] + tmpFu[7]*tmpObjS[19] + tmpFu[11]*tmpObjS[36] + tmpFu[15]*tmpObjS[53] + tmpFu[19]*tmpObjS[70] + tmpFu[23]*tmpObjS[87] + tmpFu[27]*tmpObjS[104] + tmpFu[31]*tmpObjS[121] + tmpFu[35]*tmpObjS[138] + tmpFu[39]*tmpObjS[155] + tmpFu[43]*tmpObjS[172] + tmpFu[47]*tmpObjS[189] + tmpFu[51]*tmpObjS[206] + tmpFu[55]*tmpObjS[223] + tmpFu[59]*tmpObjS[240] + tmpFu[63]*tmpObjS[257] + tmpFu[67]*tmpObjS[274];
tmpR2[54] = + tmpFu[3]*tmpObjS[3] + tmpFu[7]*tmpObjS[20] + tmpFu[11]*tmpObjS[37] + tmpFu[15]*tmpObjS[54] + tmpFu[19]*tmpObjS[71] + tmpFu[23]*tmpObjS[88] + tmpFu[27]*tmpObjS[105] + tmpFu[31]*tmpObjS[122] + tmpFu[35]*tmpObjS[139] + tmpFu[39]*tmpObjS[156] + tmpFu[43]*tmpObjS[173] + tmpFu[47]*tmpObjS[190] + tmpFu[51]*tmpObjS[207] + tmpFu[55]*tmpObjS[224] + tmpFu[59]*tmpObjS[241] + tmpFu[63]*tmpObjS[258] + tmpFu[67]*tmpObjS[275];
tmpR2[55] = + tmpFu[3]*tmpObjS[4] + tmpFu[7]*tmpObjS[21] + tmpFu[11]*tmpObjS[38] + tmpFu[15]*tmpObjS[55] + tmpFu[19]*tmpObjS[72] + tmpFu[23]*tmpObjS[89] + tmpFu[27]*tmpObjS[106] + tmpFu[31]*tmpObjS[123] + tmpFu[35]*tmpObjS[140] + tmpFu[39]*tmpObjS[157] + tmpFu[43]*tmpObjS[174] + tmpFu[47]*tmpObjS[191] + tmpFu[51]*tmpObjS[208] + tmpFu[55]*tmpObjS[225] + tmpFu[59]*tmpObjS[242] + tmpFu[63]*tmpObjS[259] + tmpFu[67]*tmpObjS[276];
tmpR2[56] = + tmpFu[3]*tmpObjS[5] + tmpFu[7]*tmpObjS[22] + tmpFu[11]*tmpObjS[39] + tmpFu[15]*tmpObjS[56] + tmpFu[19]*tmpObjS[73] + tmpFu[23]*tmpObjS[90] + tmpFu[27]*tmpObjS[107] + tmpFu[31]*tmpObjS[124] + tmpFu[35]*tmpObjS[141] + tmpFu[39]*tmpObjS[158] + tmpFu[43]*tmpObjS[175] + tmpFu[47]*tmpObjS[192] + tmpFu[51]*tmpObjS[209] + tmpFu[55]*tmpObjS[226] + tmpFu[59]*tmpObjS[243] + tmpFu[63]*tmpObjS[260] + tmpFu[67]*tmpObjS[277];
tmpR2[57] = + tmpFu[3]*tmpObjS[6] + tmpFu[7]*tmpObjS[23] + tmpFu[11]*tmpObjS[40] + tmpFu[15]*tmpObjS[57] + tmpFu[19]*tmpObjS[74] + tmpFu[23]*tmpObjS[91] + tmpFu[27]*tmpObjS[108] + tmpFu[31]*tmpObjS[125] + tmpFu[35]*tmpObjS[142] + tmpFu[39]*tmpObjS[159] + tmpFu[43]*tmpObjS[176] + tmpFu[47]*tmpObjS[193] + tmpFu[51]*tmpObjS[210] + tmpFu[55]*tmpObjS[227] + tmpFu[59]*tmpObjS[244] + tmpFu[63]*tmpObjS[261] + tmpFu[67]*tmpObjS[278];
tmpR2[58] = + tmpFu[3]*tmpObjS[7] + tmpFu[7]*tmpObjS[24] + tmpFu[11]*tmpObjS[41] + tmpFu[15]*tmpObjS[58] + tmpFu[19]*tmpObjS[75] + tmpFu[23]*tmpObjS[92] + tmpFu[27]*tmpObjS[109] + tmpFu[31]*tmpObjS[126] + tmpFu[35]*tmpObjS[143] + tmpFu[39]*tmpObjS[160] + tmpFu[43]*tmpObjS[177] + tmpFu[47]*tmpObjS[194] + tmpFu[51]*tmpObjS[211] + tmpFu[55]*tmpObjS[228] + tmpFu[59]*tmpObjS[245] + tmpFu[63]*tmpObjS[262] + tmpFu[67]*tmpObjS[279];
tmpR2[59] = + tmpFu[3]*tmpObjS[8] + tmpFu[7]*tmpObjS[25] + tmpFu[11]*tmpObjS[42] + tmpFu[15]*tmpObjS[59] + tmpFu[19]*tmpObjS[76] + tmpFu[23]*tmpObjS[93] + tmpFu[27]*tmpObjS[110] + tmpFu[31]*tmpObjS[127] + tmpFu[35]*tmpObjS[144] + tmpFu[39]*tmpObjS[161] + tmpFu[43]*tmpObjS[178] + tmpFu[47]*tmpObjS[195] + tmpFu[51]*tmpObjS[212] + tmpFu[55]*tmpObjS[229] + tmpFu[59]*tmpObjS[246] + tmpFu[63]*tmpObjS[263] + tmpFu[67]*tmpObjS[280];
tmpR2[60] = + tmpFu[3]*tmpObjS[9] + tmpFu[7]*tmpObjS[26] + tmpFu[11]*tmpObjS[43] + tmpFu[15]*tmpObjS[60] + tmpFu[19]*tmpObjS[77] + tmpFu[23]*tmpObjS[94] + tmpFu[27]*tmpObjS[111] + tmpFu[31]*tmpObjS[128] + tmpFu[35]*tmpObjS[145] + tmpFu[39]*tmpObjS[162] + tmpFu[43]*tmpObjS[179] + tmpFu[47]*tmpObjS[196] + tmpFu[51]*tmpObjS[213] + tmpFu[55]*tmpObjS[230] + tmpFu[59]*tmpObjS[247] + tmpFu[63]*tmpObjS[264] + tmpFu[67]*tmpObjS[281];
tmpR2[61] = + tmpFu[3]*tmpObjS[10] + tmpFu[7]*tmpObjS[27] + tmpFu[11]*tmpObjS[44] + tmpFu[15]*tmpObjS[61] + tmpFu[19]*tmpObjS[78] + tmpFu[23]*tmpObjS[95] + tmpFu[27]*tmpObjS[112] + tmpFu[31]*tmpObjS[129] + tmpFu[35]*tmpObjS[146] + tmpFu[39]*tmpObjS[163] + tmpFu[43]*tmpObjS[180] + tmpFu[47]*tmpObjS[197] + tmpFu[51]*tmpObjS[214] + tmpFu[55]*tmpObjS[231] + tmpFu[59]*tmpObjS[248] + tmpFu[63]*tmpObjS[265] + tmpFu[67]*tmpObjS[282];
tmpR2[62] = + tmpFu[3]*tmpObjS[11] + tmpFu[7]*tmpObjS[28] + tmpFu[11]*tmpObjS[45] + tmpFu[15]*tmpObjS[62] + tmpFu[19]*tmpObjS[79] + tmpFu[23]*tmpObjS[96] + tmpFu[27]*tmpObjS[113] + tmpFu[31]*tmpObjS[130] + tmpFu[35]*tmpObjS[147] + tmpFu[39]*tmpObjS[164] + tmpFu[43]*tmpObjS[181] + tmpFu[47]*tmpObjS[198] + tmpFu[51]*tmpObjS[215] + tmpFu[55]*tmpObjS[232] + tmpFu[59]*tmpObjS[249] + tmpFu[63]*tmpObjS[266] + tmpFu[67]*tmpObjS[283];
tmpR2[63] = + tmpFu[3]*tmpObjS[12] + tmpFu[7]*tmpObjS[29] + tmpFu[11]*tmpObjS[46] + tmpFu[15]*tmpObjS[63] + tmpFu[19]*tmpObjS[80] + tmpFu[23]*tmpObjS[97] + tmpFu[27]*tmpObjS[114] + tmpFu[31]*tmpObjS[131] + tmpFu[35]*tmpObjS[148] + tmpFu[39]*tmpObjS[165] + tmpFu[43]*tmpObjS[182] + tmpFu[47]*tmpObjS[199] + tmpFu[51]*tmpObjS[216] + tmpFu[55]*tmpObjS[233] + tmpFu[59]*tmpObjS[250] + tmpFu[63]*tmpObjS[267] + tmpFu[67]*tmpObjS[284];
tmpR2[64] = + tmpFu[3]*tmpObjS[13] + tmpFu[7]*tmpObjS[30] + tmpFu[11]*tmpObjS[47] + tmpFu[15]*tmpObjS[64] + tmpFu[19]*tmpObjS[81] + tmpFu[23]*tmpObjS[98] + tmpFu[27]*tmpObjS[115] + tmpFu[31]*tmpObjS[132] + tmpFu[35]*tmpObjS[149] + tmpFu[39]*tmpObjS[166] + tmpFu[43]*tmpObjS[183] + tmpFu[47]*tmpObjS[200] + tmpFu[51]*tmpObjS[217] + tmpFu[55]*tmpObjS[234] + tmpFu[59]*tmpObjS[251] + tmpFu[63]*tmpObjS[268] + tmpFu[67]*tmpObjS[285];
tmpR2[65] = + tmpFu[3]*tmpObjS[14] + tmpFu[7]*tmpObjS[31] + tmpFu[11]*tmpObjS[48] + tmpFu[15]*tmpObjS[65] + tmpFu[19]*tmpObjS[82] + tmpFu[23]*tmpObjS[99] + tmpFu[27]*tmpObjS[116] + tmpFu[31]*tmpObjS[133] + tmpFu[35]*tmpObjS[150] + tmpFu[39]*tmpObjS[167] + tmpFu[43]*tmpObjS[184] + tmpFu[47]*tmpObjS[201] + tmpFu[51]*tmpObjS[218] + tmpFu[55]*tmpObjS[235] + tmpFu[59]*tmpObjS[252] + tmpFu[63]*tmpObjS[269] + tmpFu[67]*tmpObjS[286];
tmpR2[66] = + tmpFu[3]*tmpObjS[15] + tmpFu[7]*tmpObjS[32] + tmpFu[11]*tmpObjS[49] + tmpFu[15]*tmpObjS[66] + tmpFu[19]*tmpObjS[83] + tmpFu[23]*tmpObjS[100] + tmpFu[27]*tmpObjS[117] + tmpFu[31]*tmpObjS[134] + tmpFu[35]*tmpObjS[151] + tmpFu[39]*tmpObjS[168] + tmpFu[43]*tmpObjS[185] + tmpFu[47]*tmpObjS[202] + tmpFu[51]*tmpObjS[219] + tmpFu[55]*tmpObjS[236] + tmpFu[59]*tmpObjS[253] + tmpFu[63]*tmpObjS[270] + tmpFu[67]*tmpObjS[287];
tmpR2[67] = + tmpFu[3]*tmpObjS[16] + tmpFu[7]*tmpObjS[33] + tmpFu[11]*tmpObjS[50] + tmpFu[15]*tmpObjS[67] + tmpFu[19]*tmpObjS[84] + tmpFu[23]*tmpObjS[101] + tmpFu[27]*tmpObjS[118] + tmpFu[31]*tmpObjS[135] + tmpFu[35]*tmpObjS[152] + tmpFu[39]*tmpObjS[169] + tmpFu[43]*tmpObjS[186] + tmpFu[47]*tmpObjS[203] + tmpFu[51]*tmpObjS[220] + tmpFu[55]*tmpObjS[237] + tmpFu[59]*tmpObjS[254] + tmpFu[63]*tmpObjS[271] + tmpFu[67]*tmpObjS[288];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[12] + tmpR2[4]*tmpFu[16] + tmpR2[5]*tmpFu[20] + tmpR2[6]*tmpFu[24] + tmpR2[7]*tmpFu[28] + tmpR2[8]*tmpFu[32] + tmpR2[9]*tmpFu[36] + tmpR2[10]*tmpFu[40] + tmpR2[11]*tmpFu[44] + tmpR2[12]*tmpFu[48] + tmpR2[13]*tmpFu[52] + tmpR2[14]*tmpFu[56] + tmpR2[15]*tmpFu[60] + tmpR2[16]*tmpFu[64];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[9] + tmpR2[3]*tmpFu[13] + tmpR2[4]*tmpFu[17] + tmpR2[5]*tmpFu[21] + tmpR2[6]*tmpFu[25] + tmpR2[7]*tmpFu[29] + tmpR2[8]*tmpFu[33] + tmpR2[9]*tmpFu[37] + tmpR2[10]*tmpFu[41] + tmpR2[11]*tmpFu[45] + tmpR2[12]*tmpFu[49] + tmpR2[13]*tmpFu[53] + tmpR2[14]*tmpFu[57] + tmpR2[15]*tmpFu[61] + tmpR2[16]*tmpFu[65];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[6] + tmpR2[2]*tmpFu[10] + tmpR2[3]*tmpFu[14] + tmpR2[4]*tmpFu[18] + tmpR2[5]*tmpFu[22] + tmpR2[6]*tmpFu[26] + tmpR2[7]*tmpFu[30] + tmpR2[8]*tmpFu[34] + tmpR2[9]*tmpFu[38] + tmpR2[10]*tmpFu[42] + tmpR2[11]*tmpFu[46] + tmpR2[12]*tmpFu[50] + tmpR2[13]*tmpFu[54] + tmpR2[14]*tmpFu[58] + tmpR2[15]*tmpFu[62] + tmpR2[16]*tmpFu[66];
tmpR1[3] = + tmpR2[0]*tmpFu[3] + tmpR2[1]*tmpFu[7] + tmpR2[2]*tmpFu[11] + tmpR2[3]*tmpFu[15] + tmpR2[4]*tmpFu[19] + tmpR2[5]*tmpFu[23] + tmpR2[6]*tmpFu[27] + tmpR2[7]*tmpFu[31] + tmpR2[8]*tmpFu[35] + tmpR2[9]*tmpFu[39] + tmpR2[10]*tmpFu[43] + tmpR2[11]*tmpFu[47] + tmpR2[12]*tmpFu[51] + tmpR2[13]*tmpFu[55] + tmpR2[14]*tmpFu[59] + tmpR2[15]*tmpFu[63] + tmpR2[16]*tmpFu[67];
tmpR1[4] = + tmpR2[17]*tmpFu[0] + tmpR2[18]*tmpFu[4] + tmpR2[19]*tmpFu[8] + tmpR2[20]*tmpFu[12] + tmpR2[21]*tmpFu[16] + tmpR2[22]*tmpFu[20] + tmpR2[23]*tmpFu[24] + tmpR2[24]*tmpFu[28] + tmpR2[25]*tmpFu[32] + tmpR2[26]*tmpFu[36] + tmpR2[27]*tmpFu[40] + tmpR2[28]*tmpFu[44] + tmpR2[29]*tmpFu[48] + tmpR2[30]*tmpFu[52] + tmpR2[31]*tmpFu[56] + tmpR2[32]*tmpFu[60] + tmpR2[33]*tmpFu[64];
tmpR1[5] = + tmpR2[17]*tmpFu[1] + tmpR2[18]*tmpFu[5] + tmpR2[19]*tmpFu[9] + tmpR2[20]*tmpFu[13] + tmpR2[21]*tmpFu[17] + tmpR2[22]*tmpFu[21] + tmpR2[23]*tmpFu[25] + tmpR2[24]*tmpFu[29] + tmpR2[25]*tmpFu[33] + tmpR2[26]*tmpFu[37] + tmpR2[27]*tmpFu[41] + tmpR2[28]*tmpFu[45] + tmpR2[29]*tmpFu[49] + tmpR2[30]*tmpFu[53] + tmpR2[31]*tmpFu[57] + tmpR2[32]*tmpFu[61] + tmpR2[33]*tmpFu[65];
tmpR1[6] = + tmpR2[17]*tmpFu[2] + tmpR2[18]*tmpFu[6] + tmpR2[19]*tmpFu[10] + tmpR2[20]*tmpFu[14] + tmpR2[21]*tmpFu[18] + tmpR2[22]*tmpFu[22] + tmpR2[23]*tmpFu[26] + tmpR2[24]*tmpFu[30] + tmpR2[25]*tmpFu[34] + tmpR2[26]*tmpFu[38] + tmpR2[27]*tmpFu[42] + tmpR2[28]*tmpFu[46] + tmpR2[29]*tmpFu[50] + tmpR2[30]*tmpFu[54] + tmpR2[31]*tmpFu[58] + tmpR2[32]*tmpFu[62] + tmpR2[33]*tmpFu[66];
tmpR1[7] = + tmpR2[17]*tmpFu[3] + tmpR2[18]*tmpFu[7] + tmpR2[19]*tmpFu[11] + tmpR2[20]*tmpFu[15] + tmpR2[21]*tmpFu[19] + tmpR2[22]*tmpFu[23] + tmpR2[23]*tmpFu[27] + tmpR2[24]*tmpFu[31] + tmpR2[25]*tmpFu[35] + tmpR2[26]*tmpFu[39] + tmpR2[27]*tmpFu[43] + tmpR2[28]*tmpFu[47] + tmpR2[29]*tmpFu[51] + tmpR2[30]*tmpFu[55] + tmpR2[31]*tmpFu[59] + tmpR2[32]*tmpFu[63] + tmpR2[33]*tmpFu[67];
tmpR1[8] = + tmpR2[34]*tmpFu[0] + tmpR2[35]*tmpFu[4] + tmpR2[36]*tmpFu[8] + tmpR2[37]*tmpFu[12] + tmpR2[38]*tmpFu[16] + tmpR2[39]*tmpFu[20] + tmpR2[40]*tmpFu[24] + tmpR2[41]*tmpFu[28] + tmpR2[42]*tmpFu[32] + tmpR2[43]*tmpFu[36] + tmpR2[44]*tmpFu[40] + tmpR2[45]*tmpFu[44] + tmpR2[46]*tmpFu[48] + tmpR2[47]*tmpFu[52] + tmpR2[48]*tmpFu[56] + tmpR2[49]*tmpFu[60] + tmpR2[50]*tmpFu[64];
tmpR1[9] = + tmpR2[34]*tmpFu[1] + tmpR2[35]*tmpFu[5] + tmpR2[36]*tmpFu[9] + tmpR2[37]*tmpFu[13] + tmpR2[38]*tmpFu[17] + tmpR2[39]*tmpFu[21] + tmpR2[40]*tmpFu[25] + tmpR2[41]*tmpFu[29] + tmpR2[42]*tmpFu[33] + tmpR2[43]*tmpFu[37] + tmpR2[44]*tmpFu[41] + tmpR2[45]*tmpFu[45] + tmpR2[46]*tmpFu[49] + tmpR2[47]*tmpFu[53] + tmpR2[48]*tmpFu[57] + tmpR2[49]*tmpFu[61] + tmpR2[50]*tmpFu[65];
tmpR1[10] = + tmpR2[34]*tmpFu[2] + tmpR2[35]*tmpFu[6] + tmpR2[36]*tmpFu[10] + tmpR2[37]*tmpFu[14] + tmpR2[38]*tmpFu[18] + tmpR2[39]*tmpFu[22] + tmpR2[40]*tmpFu[26] + tmpR2[41]*tmpFu[30] + tmpR2[42]*tmpFu[34] + tmpR2[43]*tmpFu[38] + tmpR2[44]*tmpFu[42] + tmpR2[45]*tmpFu[46] + tmpR2[46]*tmpFu[50] + tmpR2[47]*tmpFu[54] + tmpR2[48]*tmpFu[58] + tmpR2[49]*tmpFu[62] + tmpR2[50]*tmpFu[66];
tmpR1[11] = + tmpR2[34]*tmpFu[3] + tmpR2[35]*tmpFu[7] + tmpR2[36]*tmpFu[11] + tmpR2[37]*tmpFu[15] + tmpR2[38]*tmpFu[19] + tmpR2[39]*tmpFu[23] + tmpR2[40]*tmpFu[27] + tmpR2[41]*tmpFu[31] + tmpR2[42]*tmpFu[35] + tmpR2[43]*tmpFu[39] + tmpR2[44]*tmpFu[43] + tmpR2[45]*tmpFu[47] + tmpR2[46]*tmpFu[51] + tmpR2[47]*tmpFu[55] + tmpR2[48]*tmpFu[59] + tmpR2[49]*tmpFu[63] + tmpR2[50]*tmpFu[67];
tmpR1[12] = + tmpR2[51]*tmpFu[0] + tmpR2[52]*tmpFu[4] + tmpR2[53]*tmpFu[8] + tmpR2[54]*tmpFu[12] + tmpR2[55]*tmpFu[16] + tmpR2[56]*tmpFu[20] + tmpR2[57]*tmpFu[24] + tmpR2[58]*tmpFu[28] + tmpR2[59]*tmpFu[32] + tmpR2[60]*tmpFu[36] + tmpR2[61]*tmpFu[40] + tmpR2[62]*tmpFu[44] + tmpR2[63]*tmpFu[48] + tmpR2[64]*tmpFu[52] + tmpR2[65]*tmpFu[56] + tmpR2[66]*tmpFu[60] + tmpR2[67]*tmpFu[64];
tmpR1[13] = + tmpR2[51]*tmpFu[1] + tmpR2[52]*tmpFu[5] + tmpR2[53]*tmpFu[9] + tmpR2[54]*tmpFu[13] + tmpR2[55]*tmpFu[17] + tmpR2[56]*tmpFu[21] + tmpR2[57]*tmpFu[25] + tmpR2[58]*tmpFu[29] + tmpR2[59]*tmpFu[33] + tmpR2[60]*tmpFu[37] + tmpR2[61]*tmpFu[41] + tmpR2[62]*tmpFu[45] + tmpR2[63]*tmpFu[49] + tmpR2[64]*tmpFu[53] + tmpR2[65]*tmpFu[57] + tmpR2[66]*tmpFu[61] + tmpR2[67]*tmpFu[65];
tmpR1[14] = + tmpR2[51]*tmpFu[2] + tmpR2[52]*tmpFu[6] + tmpR2[53]*tmpFu[10] + tmpR2[54]*tmpFu[14] + tmpR2[55]*tmpFu[18] + tmpR2[56]*tmpFu[22] + tmpR2[57]*tmpFu[26] + tmpR2[58]*tmpFu[30] + tmpR2[59]*tmpFu[34] + tmpR2[60]*tmpFu[38] + tmpR2[61]*tmpFu[42] + tmpR2[62]*tmpFu[46] + tmpR2[63]*tmpFu[50] + tmpR2[64]*tmpFu[54] + tmpR2[65]*tmpFu[58] + tmpR2[66]*tmpFu[62] + tmpR2[67]*tmpFu[66];
tmpR1[15] = + tmpR2[51]*tmpFu[3] + tmpR2[52]*tmpFu[7] + tmpR2[53]*tmpFu[11] + tmpR2[54]*tmpFu[15] + tmpR2[55]*tmpFu[19] + tmpR2[56]*tmpFu[23] + tmpR2[57]*tmpFu[27] + tmpR2[58]*tmpFu[31] + tmpR2[59]*tmpFu[35] + tmpR2[60]*tmpFu[39] + tmpR2[61]*tmpFu[43] + tmpR2[62]*tmpFu[47] + tmpR2[63]*tmpFu[51] + tmpR2[64]*tmpFu[55] + tmpR2[65]*tmpFu[59] + tmpR2[66]*tmpFu[63] + tmpR2[67]*tmpFu[67];
}

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
tmpQN2[81] = +tmpObjSEndTerm[81];
tmpQN2[82] = +tmpObjSEndTerm[82];
tmpQN2[83] = +tmpObjSEndTerm[83];
tmpQN2[84] = +tmpObjSEndTerm[84];
tmpQN2[85] = +tmpObjSEndTerm[85];
tmpQN2[86] = +tmpObjSEndTerm[86];
tmpQN2[87] = +tmpObjSEndTerm[87];
tmpQN2[88] = +tmpObjSEndTerm[88];
tmpQN2[89] = +tmpObjSEndTerm[89];
tmpQN2[90] = +tmpObjSEndTerm[90];
tmpQN2[91] = +tmpObjSEndTerm[91];
tmpQN2[92] = +tmpObjSEndTerm[92];
tmpQN2[93] = +tmpObjSEndTerm[93];
tmpQN2[94] = +tmpObjSEndTerm[94];
tmpQN2[95] = +tmpObjSEndTerm[95];
tmpQN2[96] = +tmpObjSEndTerm[96];
tmpQN2[97] = +tmpObjSEndTerm[97];
tmpQN2[98] = +tmpObjSEndTerm[98];
tmpQN2[99] = +tmpObjSEndTerm[99];
tmpQN2[100] = 0.0;
;
tmpQN2[101] = 0.0;
;
tmpQN2[102] = 0.0;
;
tmpQN2[103] = 0.0;
;
tmpQN2[104] = 0.0;
;
tmpQN2[105] = 0.0;
;
tmpQN2[106] = 0.0;
;
tmpQN2[107] = 0.0;
;
tmpQN2[108] = 0.0;
;
tmpQN2[109] = 0.0;
;
tmpQN2[110] = 0.0;
;
tmpQN2[111] = 0.0;
;
tmpQN2[112] = 0.0;
;
tmpQN2[113] = 0.0;
;
tmpQN2[114] = 0.0;
;
tmpQN2[115] = 0.0;
;
tmpQN2[116] = 0.0;
;
tmpQN2[117] = 0.0;
;
tmpQN2[118] = 0.0;
;
tmpQN2[119] = 0.0;
;
tmpQN2[120] = 0.0;
;
tmpQN2[121] = 0.0;
;
tmpQN2[122] = 0.0;
;
tmpQN2[123] = 0.0;
;
tmpQN2[124] = 0.0;
;
tmpQN2[125] = 0.0;
;
tmpQN2[126] = 0.0;
;
tmpQN2[127] = 0.0;
;
tmpQN2[128] = 0.0;
;
tmpQN2[129] = 0.0;
;
tmpQN2[130] = 0.0;
;
tmpQN2[131] = 0.0;
;
tmpQN2[132] = 0.0;
;
tmpQN2[133] = 0.0;
;
tmpQN2[134] = 0.0;
;
tmpQN2[135] = 0.0;
;
tmpQN2[136] = 0.0;
;
tmpQN2[137] = 0.0;
;
tmpQN2[138] = 0.0;
;
tmpQN2[139] = 0.0;
;
tmpQN2[140] = 0.0;
;
tmpQN2[141] = 0.0;
;
tmpQN2[142] = 0.0;
;
tmpQN2[143] = 0.0;
;
tmpQN2[144] = 0.0;
;
tmpQN2[145] = 0.0;
;
tmpQN2[146] = 0.0;
;
tmpQN2[147] = 0.0;
;
tmpQN2[148] = 0.0;
;
tmpQN2[149] = 0.0;
;
tmpQN2[150] = 0.0;
;
tmpQN2[151] = 0.0;
;
tmpQN2[152] = 0.0;
;
tmpQN2[153] = 0.0;
;
tmpQN2[154] = 0.0;
;
tmpQN2[155] = 0.0;
;
tmpQN2[156] = 0.0;
;
tmpQN2[157] = 0.0;
;
tmpQN2[158] = 0.0;
;
tmpQN2[159] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = 0.0;
;
tmpQN1[11] = 0.0;
;
tmpQN1[12] = 0.0;
;
tmpQN1[13] = 0.0;
;
tmpQN1[14] = 0.0;
;
tmpQN1[15] = 0.0;
;
tmpQN1[16] = + tmpQN2[10];
tmpQN1[17] = + tmpQN2[11];
tmpQN1[18] = + tmpQN2[12];
tmpQN1[19] = + tmpQN2[13];
tmpQN1[20] = + tmpQN2[14];
tmpQN1[21] = + tmpQN2[15];
tmpQN1[22] = + tmpQN2[16];
tmpQN1[23] = + tmpQN2[17];
tmpQN1[24] = + tmpQN2[18];
tmpQN1[25] = + tmpQN2[19];
tmpQN1[26] = 0.0;
;
tmpQN1[27] = 0.0;
;
tmpQN1[28] = 0.0;
;
tmpQN1[29] = 0.0;
;
tmpQN1[30] = 0.0;
;
tmpQN1[31] = 0.0;
;
tmpQN1[32] = + tmpQN2[20];
tmpQN1[33] = + tmpQN2[21];
tmpQN1[34] = + tmpQN2[22];
tmpQN1[35] = + tmpQN2[23];
tmpQN1[36] = + tmpQN2[24];
tmpQN1[37] = + tmpQN2[25];
tmpQN1[38] = + tmpQN2[26];
tmpQN1[39] = + tmpQN2[27];
tmpQN1[40] = + tmpQN2[28];
tmpQN1[41] = + tmpQN2[29];
tmpQN1[42] = 0.0;
;
tmpQN1[43] = 0.0;
;
tmpQN1[44] = 0.0;
;
tmpQN1[45] = 0.0;
;
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = + tmpQN2[30];
tmpQN1[49] = + tmpQN2[31];
tmpQN1[50] = + tmpQN2[32];
tmpQN1[51] = + tmpQN2[33];
tmpQN1[52] = + tmpQN2[34];
tmpQN1[53] = + tmpQN2[35];
tmpQN1[54] = + tmpQN2[36];
tmpQN1[55] = + tmpQN2[37];
tmpQN1[56] = + tmpQN2[38];
tmpQN1[57] = + tmpQN2[39];
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = 0.0;
;
tmpQN1[61] = 0.0;
;
tmpQN1[62] = 0.0;
;
tmpQN1[63] = 0.0;
;
tmpQN1[64] = + tmpQN2[40];
tmpQN1[65] = + tmpQN2[41];
tmpQN1[66] = + tmpQN2[42];
tmpQN1[67] = + tmpQN2[43];
tmpQN1[68] = + tmpQN2[44];
tmpQN1[69] = + tmpQN2[45];
tmpQN1[70] = + tmpQN2[46];
tmpQN1[71] = + tmpQN2[47];
tmpQN1[72] = + tmpQN2[48];
tmpQN1[73] = + tmpQN2[49];
tmpQN1[74] = 0.0;
;
tmpQN1[75] = 0.0;
;
tmpQN1[76] = 0.0;
;
tmpQN1[77] = 0.0;
;
tmpQN1[78] = 0.0;
;
tmpQN1[79] = 0.0;
;
tmpQN1[80] = + tmpQN2[50];
tmpQN1[81] = + tmpQN2[51];
tmpQN1[82] = + tmpQN2[52];
tmpQN1[83] = + tmpQN2[53];
tmpQN1[84] = + tmpQN2[54];
tmpQN1[85] = + tmpQN2[55];
tmpQN1[86] = + tmpQN2[56];
tmpQN1[87] = + tmpQN2[57];
tmpQN1[88] = + tmpQN2[58];
tmpQN1[89] = + tmpQN2[59];
tmpQN1[90] = 0.0;
;
tmpQN1[91] = 0.0;
;
tmpQN1[92] = 0.0;
;
tmpQN1[93] = 0.0;
;
tmpQN1[94] = 0.0;
;
tmpQN1[95] = 0.0;
;
tmpQN1[96] = + tmpQN2[60];
tmpQN1[97] = + tmpQN2[61];
tmpQN1[98] = + tmpQN2[62];
tmpQN1[99] = + tmpQN2[63];
tmpQN1[100] = + tmpQN2[64];
tmpQN1[101] = + tmpQN2[65];
tmpQN1[102] = + tmpQN2[66];
tmpQN1[103] = + tmpQN2[67];
tmpQN1[104] = + tmpQN2[68];
tmpQN1[105] = + tmpQN2[69];
tmpQN1[106] = 0.0;
;
tmpQN1[107] = 0.0;
;
tmpQN1[108] = 0.0;
;
tmpQN1[109] = 0.0;
;
tmpQN1[110] = 0.0;
;
tmpQN1[111] = 0.0;
;
tmpQN1[112] = + tmpQN2[70];
tmpQN1[113] = + tmpQN2[71];
tmpQN1[114] = + tmpQN2[72];
tmpQN1[115] = + tmpQN2[73];
tmpQN1[116] = + tmpQN2[74];
tmpQN1[117] = + tmpQN2[75];
tmpQN1[118] = + tmpQN2[76];
tmpQN1[119] = + tmpQN2[77];
tmpQN1[120] = + tmpQN2[78];
tmpQN1[121] = + tmpQN2[79];
tmpQN1[122] = 0.0;
;
tmpQN1[123] = 0.0;
;
tmpQN1[124] = 0.0;
;
tmpQN1[125] = 0.0;
;
tmpQN1[126] = 0.0;
;
tmpQN1[127] = 0.0;
;
tmpQN1[128] = + tmpQN2[80];
tmpQN1[129] = + tmpQN2[81];
tmpQN1[130] = + tmpQN2[82];
tmpQN1[131] = + tmpQN2[83];
tmpQN1[132] = + tmpQN2[84];
tmpQN1[133] = + tmpQN2[85];
tmpQN1[134] = + tmpQN2[86];
tmpQN1[135] = + tmpQN2[87];
tmpQN1[136] = + tmpQN2[88];
tmpQN1[137] = + tmpQN2[89];
tmpQN1[138] = 0.0;
;
tmpQN1[139] = 0.0;
;
tmpQN1[140] = 0.0;
;
tmpQN1[141] = 0.0;
;
tmpQN1[142] = 0.0;
;
tmpQN1[143] = 0.0;
;
tmpQN1[144] = + tmpQN2[90];
tmpQN1[145] = + tmpQN2[91];
tmpQN1[146] = + tmpQN2[92];
tmpQN1[147] = + tmpQN2[93];
tmpQN1[148] = + tmpQN2[94];
tmpQN1[149] = + tmpQN2[95];
tmpQN1[150] = + tmpQN2[96];
tmpQN1[151] = + tmpQN2[97];
tmpQN1[152] = + tmpQN2[98];
tmpQN1[153] = + tmpQN2[99];
tmpQN1[154] = 0.0;
;
tmpQN1[155] = 0.0;
;
tmpQN1[156] = 0.0;
;
tmpQN1[157] = 0.0;
;
tmpQN1[158] = 0.0;
;
tmpQN1[159] = 0.0;
;
tmpQN1[160] = + tmpQN2[100];
tmpQN1[161] = + tmpQN2[101];
tmpQN1[162] = + tmpQN2[102];
tmpQN1[163] = + tmpQN2[103];
tmpQN1[164] = + tmpQN2[104];
tmpQN1[165] = + tmpQN2[105];
tmpQN1[166] = + tmpQN2[106];
tmpQN1[167] = + tmpQN2[107];
tmpQN1[168] = + tmpQN2[108];
tmpQN1[169] = + tmpQN2[109];
tmpQN1[170] = 0.0;
;
tmpQN1[171] = 0.0;
;
tmpQN1[172] = 0.0;
;
tmpQN1[173] = 0.0;
;
tmpQN1[174] = 0.0;
;
tmpQN1[175] = 0.0;
;
tmpQN1[176] = + tmpQN2[110];
tmpQN1[177] = + tmpQN2[111];
tmpQN1[178] = + tmpQN2[112];
tmpQN1[179] = + tmpQN2[113];
tmpQN1[180] = + tmpQN2[114];
tmpQN1[181] = + tmpQN2[115];
tmpQN1[182] = + tmpQN2[116];
tmpQN1[183] = + tmpQN2[117];
tmpQN1[184] = + tmpQN2[118];
tmpQN1[185] = + tmpQN2[119];
tmpQN1[186] = 0.0;
;
tmpQN1[187] = 0.0;
;
tmpQN1[188] = 0.0;
;
tmpQN1[189] = 0.0;
;
tmpQN1[190] = 0.0;
;
tmpQN1[191] = 0.0;
;
tmpQN1[192] = + tmpQN2[120];
tmpQN1[193] = + tmpQN2[121];
tmpQN1[194] = + tmpQN2[122];
tmpQN1[195] = + tmpQN2[123];
tmpQN1[196] = + tmpQN2[124];
tmpQN1[197] = + tmpQN2[125];
tmpQN1[198] = + tmpQN2[126];
tmpQN1[199] = + tmpQN2[127];
tmpQN1[200] = + tmpQN2[128];
tmpQN1[201] = + tmpQN2[129];
tmpQN1[202] = 0.0;
;
tmpQN1[203] = 0.0;
;
tmpQN1[204] = 0.0;
;
tmpQN1[205] = 0.0;
;
tmpQN1[206] = 0.0;
;
tmpQN1[207] = 0.0;
;
tmpQN1[208] = + tmpQN2[130];
tmpQN1[209] = + tmpQN2[131];
tmpQN1[210] = + tmpQN2[132];
tmpQN1[211] = + tmpQN2[133];
tmpQN1[212] = + tmpQN2[134];
tmpQN1[213] = + tmpQN2[135];
tmpQN1[214] = + tmpQN2[136];
tmpQN1[215] = + tmpQN2[137];
tmpQN1[216] = + tmpQN2[138];
tmpQN1[217] = + tmpQN2[139];
tmpQN1[218] = 0.0;
;
tmpQN1[219] = 0.0;
;
tmpQN1[220] = 0.0;
;
tmpQN1[221] = 0.0;
;
tmpQN1[222] = 0.0;
;
tmpQN1[223] = 0.0;
;
tmpQN1[224] = + tmpQN2[140];
tmpQN1[225] = + tmpQN2[141];
tmpQN1[226] = + tmpQN2[142];
tmpQN1[227] = + tmpQN2[143];
tmpQN1[228] = + tmpQN2[144];
tmpQN1[229] = + tmpQN2[145];
tmpQN1[230] = + tmpQN2[146];
tmpQN1[231] = + tmpQN2[147];
tmpQN1[232] = + tmpQN2[148];
tmpQN1[233] = + tmpQN2[149];
tmpQN1[234] = 0.0;
;
tmpQN1[235] = 0.0;
;
tmpQN1[236] = 0.0;
;
tmpQN1[237] = 0.0;
;
tmpQN1[238] = 0.0;
;
tmpQN1[239] = 0.0;
;
tmpQN1[240] = + tmpQN2[150];
tmpQN1[241] = + tmpQN2[151];
tmpQN1[242] = + tmpQN2[152];
tmpQN1[243] = + tmpQN2[153];
tmpQN1[244] = + tmpQN2[154];
tmpQN1[245] = + tmpQN2[155];
tmpQN1[246] = + tmpQN2[156];
tmpQN1[247] = + tmpQN2[157];
tmpQN1[248] = + tmpQN2[158];
tmpQN1[249] = + tmpQN2[159];
tmpQN1[250] = 0.0;
;
tmpQN1[251] = 0.0;
;
tmpQN1[252] = 0.0;
;
tmpQN1[253] = 0.0;
;
tmpQN1[254] = 0.0;
;
tmpQN1[255] = 0.0;
;
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 16];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 16 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 16 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 16 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 16 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 16 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[runObj * 16 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[runObj * 16 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[runObj * 16 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[runObj * 16 + 9];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[runObj * 16 + 10];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[runObj * 16 + 11];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[runObj * 16 + 12];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[runObj * 16 + 13];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[runObj * 16 + 14];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[runObj * 16 + 15];
nmpcWorkspace.objValueIn[16] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[17] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[18] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[19] = nmpcVariables.u[runObj * 4 + 3];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[runObj * 9];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[runObj * 9 + 1];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[runObj * 9 + 2];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[runObj * 9 + 3];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[runObj * 9 + 4];
nmpcWorkspace.objValueIn[25] = nmpcVariables.od[runObj * 9 + 5];
nmpcWorkspace.objValueIn[26] = nmpcVariables.od[runObj * 9 + 6];
nmpcWorkspace.objValueIn[27] = nmpcVariables.od[runObj * 9 + 7];
nmpcWorkspace.objValueIn[28] = nmpcVariables.od[runObj * 9 + 8];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 17] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 17 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 17 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 17 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 17 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 17 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 17 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 17 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 17 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 17 + 9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.Dy[runObj * 17 + 10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.Dy[runObj * 17 + 11] = nmpcWorkspace.objValueOut[11];
nmpcWorkspace.Dy[runObj * 17 + 12] = nmpcWorkspace.objValueOut[12];
nmpcWorkspace.Dy[runObj * 17 + 13] = nmpcWorkspace.objValueOut[13];
nmpcWorkspace.Dy[runObj * 17 + 14] = nmpcWorkspace.objValueOut[14];
nmpcWorkspace.Dy[runObj * 17 + 15] = nmpcWorkspace.objValueOut[15];
nmpcWorkspace.Dy[runObj * 17 + 16] = nmpcWorkspace.objValueOut[16];

nmpc_setObjQ1Q2( &(nmpcWorkspace.objValueOut[ 17 ]), nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 256 ]), &(nmpcWorkspace.Q2[ runObj * 272 ]) );

nmpc_setObjR1R2( &(nmpcWorkspace.objValueOut[ 289 ]), nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 68 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[480];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[481];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[482];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[483];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[484];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[485];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[486];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[487];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[488];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[489];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[490];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[491];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[492];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[493];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[494];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[495];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[270];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[271];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[272];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[273];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[274];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[275];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[276];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[277];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[278];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.DyN[8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.DyN[9] = nmpcWorkspace.objValueOut[9];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11] + Gx1[12]*dOld[12] + Gx1[13]*dOld[13] + Gx1[14]*dOld[14] + Gx1[15]*dOld[15];
dNew[1] += + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7] + Gx1[24]*dOld[8] + Gx1[25]*dOld[9] + Gx1[26]*dOld[10] + Gx1[27]*dOld[11] + Gx1[28]*dOld[12] + Gx1[29]*dOld[13] + Gx1[30]*dOld[14] + Gx1[31]*dOld[15];
dNew[2] += + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7] + Gx1[40]*dOld[8] + Gx1[41]*dOld[9] + Gx1[42]*dOld[10] + Gx1[43]*dOld[11] + Gx1[44]*dOld[12] + Gx1[45]*dOld[13] + Gx1[46]*dOld[14] + Gx1[47]*dOld[15];
dNew[3] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11] + Gx1[60]*dOld[12] + Gx1[61]*dOld[13] + Gx1[62]*dOld[14] + Gx1[63]*dOld[15];
dNew[4] += + Gx1[64]*dOld[0] + Gx1[65]*dOld[1] + Gx1[66]*dOld[2] + Gx1[67]*dOld[3] + Gx1[68]*dOld[4] + Gx1[69]*dOld[5] + Gx1[70]*dOld[6] + Gx1[71]*dOld[7] + Gx1[72]*dOld[8] + Gx1[73]*dOld[9] + Gx1[74]*dOld[10] + Gx1[75]*dOld[11] + Gx1[76]*dOld[12] + Gx1[77]*dOld[13] + Gx1[78]*dOld[14] + Gx1[79]*dOld[15];
dNew[5] += + Gx1[80]*dOld[0] + Gx1[81]*dOld[1] + Gx1[82]*dOld[2] + Gx1[83]*dOld[3] + Gx1[84]*dOld[4] + Gx1[85]*dOld[5] + Gx1[86]*dOld[6] + Gx1[87]*dOld[7] + Gx1[88]*dOld[8] + Gx1[89]*dOld[9] + Gx1[90]*dOld[10] + Gx1[91]*dOld[11] + Gx1[92]*dOld[12] + Gx1[93]*dOld[13] + Gx1[94]*dOld[14] + Gx1[95]*dOld[15];
dNew[6] += + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11] + Gx1[108]*dOld[12] + Gx1[109]*dOld[13] + Gx1[110]*dOld[14] + Gx1[111]*dOld[15];
dNew[7] += + Gx1[112]*dOld[0] + Gx1[113]*dOld[1] + Gx1[114]*dOld[2] + Gx1[115]*dOld[3] + Gx1[116]*dOld[4] + Gx1[117]*dOld[5] + Gx1[118]*dOld[6] + Gx1[119]*dOld[7] + Gx1[120]*dOld[8] + Gx1[121]*dOld[9] + Gx1[122]*dOld[10] + Gx1[123]*dOld[11] + Gx1[124]*dOld[12] + Gx1[125]*dOld[13] + Gx1[126]*dOld[14] + Gx1[127]*dOld[15];
dNew[8] += + Gx1[128]*dOld[0] + Gx1[129]*dOld[1] + Gx1[130]*dOld[2] + Gx1[131]*dOld[3] + Gx1[132]*dOld[4] + Gx1[133]*dOld[5] + Gx1[134]*dOld[6] + Gx1[135]*dOld[7] + Gx1[136]*dOld[8] + Gx1[137]*dOld[9] + Gx1[138]*dOld[10] + Gx1[139]*dOld[11] + Gx1[140]*dOld[12] + Gx1[141]*dOld[13] + Gx1[142]*dOld[14] + Gx1[143]*dOld[15];
dNew[9] += + Gx1[144]*dOld[0] + Gx1[145]*dOld[1] + Gx1[146]*dOld[2] + Gx1[147]*dOld[3] + Gx1[148]*dOld[4] + Gx1[149]*dOld[5] + Gx1[150]*dOld[6] + Gx1[151]*dOld[7] + Gx1[152]*dOld[8] + Gx1[153]*dOld[9] + Gx1[154]*dOld[10] + Gx1[155]*dOld[11] + Gx1[156]*dOld[12] + Gx1[157]*dOld[13] + Gx1[158]*dOld[14] + Gx1[159]*dOld[15];
dNew[10] += + Gx1[160]*dOld[0] + Gx1[161]*dOld[1] + Gx1[162]*dOld[2] + Gx1[163]*dOld[3] + Gx1[164]*dOld[4] + Gx1[165]*dOld[5] + Gx1[166]*dOld[6] + Gx1[167]*dOld[7] + Gx1[168]*dOld[8] + Gx1[169]*dOld[9] + Gx1[170]*dOld[10] + Gx1[171]*dOld[11] + Gx1[172]*dOld[12] + Gx1[173]*dOld[13] + Gx1[174]*dOld[14] + Gx1[175]*dOld[15];
dNew[11] += + Gx1[176]*dOld[0] + Gx1[177]*dOld[1] + Gx1[178]*dOld[2] + Gx1[179]*dOld[3] + Gx1[180]*dOld[4] + Gx1[181]*dOld[5] + Gx1[182]*dOld[6] + Gx1[183]*dOld[7] + Gx1[184]*dOld[8] + Gx1[185]*dOld[9] + Gx1[186]*dOld[10] + Gx1[187]*dOld[11] + Gx1[188]*dOld[12] + Gx1[189]*dOld[13] + Gx1[190]*dOld[14] + Gx1[191]*dOld[15];
dNew[12] += + Gx1[192]*dOld[0] + Gx1[193]*dOld[1] + Gx1[194]*dOld[2] + Gx1[195]*dOld[3] + Gx1[196]*dOld[4] + Gx1[197]*dOld[5] + Gx1[198]*dOld[6] + Gx1[199]*dOld[7] + Gx1[200]*dOld[8] + Gx1[201]*dOld[9] + Gx1[202]*dOld[10] + Gx1[203]*dOld[11] + Gx1[204]*dOld[12] + Gx1[205]*dOld[13] + Gx1[206]*dOld[14] + Gx1[207]*dOld[15];
dNew[13] += + Gx1[208]*dOld[0] + Gx1[209]*dOld[1] + Gx1[210]*dOld[2] + Gx1[211]*dOld[3] + Gx1[212]*dOld[4] + Gx1[213]*dOld[5] + Gx1[214]*dOld[6] + Gx1[215]*dOld[7] + Gx1[216]*dOld[8] + Gx1[217]*dOld[9] + Gx1[218]*dOld[10] + Gx1[219]*dOld[11] + Gx1[220]*dOld[12] + Gx1[221]*dOld[13] + Gx1[222]*dOld[14] + Gx1[223]*dOld[15];
dNew[14] += + Gx1[224]*dOld[0] + Gx1[225]*dOld[1] + Gx1[226]*dOld[2] + Gx1[227]*dOld[3] + Gx1[228]*dOld[4] + Gx1[229]*dOld[5] + Gx1[230]*dOld[6] + Gx1[231]*dOld[7] + Gx1[232]*dOld[8] + Gx1[233]*dOld[9] + Gx1[234]*dOld[10] + Gx1[235]*dOld[11] + Gx1[236]*dOld[12] + Gx1[237]*dOld[13] + Gx1[238]*dOld[14] + Gx1[239]*dOld[15];
dNew[15] += + Gx1[240]*dOld[0] + Gx1[241]*dOld[1] + Gx1[242]*dOld[2] + Gx1[243]*dOld[3] + Gx1[244]*dOld[4] + Gx1[245]*dOld[5] + Gx1[246]*dOld[6] + Gx1[247]*dOld[7] + Gx1[248]*dOld[8] + Gx1[249]*dOld[9] + Gx1[250]*dOld[10] + Gx1[251]*dOld[11] + Gx1[252]*dOld[12] + Gx1[253]*dOld[13] + Gx1[254]*dOld[14] + Gx1[255]*dOld[15];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
for (lRun1 = 0;lRun1 < 16; ++lRun1)
for (lRun2 = 0;lRun2 < 16; ++lRun2)
Gx2[(lRun1 * 16) + (lRun2)] = Gx1[(lRun1 * 16) + (lRun2)];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 16; ++lRun3)
{
t += + Gx1[(lRun1 * 16) + (lRun3)]*Gx2[(lRun3 * 16) + (lRun2)];
}
Gx3[(lRun1 * 16) + (lRun2)] = t;
}
}
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44] + Gx1[12]*Gu1[48] + Gx1[13]*Gu1[52] + Gx1[14]*Gu1[56] + Gx1[15]*Gu1[60];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45] + Gx1[12]*Gu1[49] + Gx1[13]*Gu1[53] + Gx1[14]*Gu1[57] + Gx1[15]*Gu1[61];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46] + Gx1[12]*Gu1[50] + Gx1[13]*Gu1[54] + Gx1[14]*Gu1[58] + Gx1[15]*Gu1[62];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47] + Gx1[12]*Gu1[51] + Gx1[13]*Gu1[55] + Gx1[14]*Gu1[59] + Gx1[15]*Gu1[63];
Gu2[4] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[8] + Gx1[19]*Gu1[12] + Gx1[20]*Gu1[16] + Gx1[21]*Gu1[20] + Gx1[22]*Gu1[24] + Gx1[23]*Gu1[28] + Gx1[24]*Gu1[32] + Gx1[25]*Gu1[36] + Gx1[26]*Gu1[40] + Gx1[27]*Gu1[44] + Gx1[28]*Gu1[48] + Gx1[29]*Gu1[52] + Gx1[30]*Gu1[56] + Gx1[31]*Gu1[60];
Gu2[5] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[9] + Gx1[19]*Gu1[13] + Gx1[20]*Gu1[17] + Gx1[21]*Gu1[21] + Gx1[22]*Gu1[25] + Gx1[23]*Gu1[29] + Gx1[24]*Gu1[33] + Gx1[25]*Gu1[37] + Gx1[26]*Gu1[41] + Gx1[27]*Gu1[45] + Gx1[28]*Gu1[49] + Gx1[29]*Gu1[53] + Gx1[30]*Gu1[57] + Gx1[31]*Gu1[61];
Gu2[6] = + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[18]*Gu1[10] + Gx1[19]*Gu1[14] + Gx1[20]*Gu1[18] + Gx1[21]*Gu1[22] + Gx1[22]*Gu1[26] + Gx1[23]*Gu1[30] + Gx1[24]*Gu1[34] + Gx1[25]*Gu1[38] + Gx1[26]*Gu1[42] + Gx1[27]*Gu1[46] + Gx1[28]*Gu1[50] + Gx1[29]*Gu1[54] + Gx1[30]*Gu1[58] + Gx1[31]*Gu1[62];
Gu2[7] = + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[18]*Gu1[11] + Gx1[19]*Gu1[15] + Gx1[20]*Gu1[19] + Gx1[21]*Gu1[23] + Gx1[22]*Gu1[27] + Gx1[23]*Gu1[31] + Gx1[24]*Gu1[35] + Gx1[25]*Gu1[39] + Gx1[26]*Gu1[43] + Gx1[27]*Gu1[47] + Gx1[28]*Gu1[51] + Gx1[29]*Gu1[55] + Gx1[30]*Gu1[59] + Gx1[31]*Gu1[63];
Gu2[8] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[36]*Gu1[16] + Gx1[37]*Gu1[20] + Gx1[38]*Gu1[24] + Gx1[39]*Gu1[28] + Gx1[40]*Gu1[32] + Gx1[41]*Gu1[36] + Gx1[42]*Gu1[40] + Gx1[43]*Gu1[44] + Gx1[44]*Gu1[48] + Gx1[45]*Gu1[52] + Gx1[46]*Gu1[56] + Gx1[47]*Gu1[60];
Gu2[9] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[36]*Gu1[17] + Gx1[37]*Gu1[21] + Gx1[38]*Gu1[25] + Gx1[39]*Gu1[29] + Gx1[40]*Gu1[33] + Gx1[41]*Gu1[37] + Gx1[42]*Gu1[41] + Gx1[43]*Gu1[45] + Gx1[44]*Gu1[49] + Gx1[45]*Gu1[53] + Gx1[46]*Gu1[57] + Gx1[47]*Gu1[61];
Gu2[10] = + Gx1[32]*Gu1[2] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[36]*Gu1[18] + Gx1[37]*Gu1[22] + Gx1[38]*Gu1[26] + Gx1[39]*Gu1[30] + Gx1[40]*Gu1[34] + Gx1[41]*Gu1[38] + Gx1[42]*Gu1[42] + Gx1[43]*Gu1[46] + Gx1[44]*Gu1[50] + Gx1[45]*Gu1[54] + Gx1[46]*Gu1[58] + Gx1[47]*Gu1[62];
Gu2[11] = + Gx1[32]*Gu1[3] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[36]*Gu1[19] + Gx1[37]*Gu1[23] + Gx1[38]*Gu1[27] + Gx1[39]*Gu1[31] + Gx1[40]*Gu1[35] + Gx1[41]*Gu1[39] + Gx1[42]*Gu1[43] + Gx1[43]*Gu1[47] + Gx1[44]*Gu1[51] + Gx1[45]*Gu1[55] + Gx1[46]*Gu1[59] + Gx1[47]*Gu1[63];
Gu2[12] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44] + Gx1[60]*Gu1[48] + Gx1[61]*Gu1[52] + Gx1[62]*Gu1[56] + Gx1[63]*Gu1[60];
Gu2[13] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45] + Gx1[60]*Gu1[49] + Gx1[61]*Gu1[53] + Gx1[62]*Gu1[57] + Gx1[63]*Gu1[61];
Gu2[14] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46] + Gx1[60]*Gu1[50] + Gx1[61]*Gu1[54] + Gx1[62]*Gu1[58] + Gx1[63]*Gu1[62];
Gu2[15] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47] + Gx1[60]*Gu1[51] + Gx1[61]*Gu1[55] + Gx1[62]*Gu1[59] + Gx1[63]*Gu1[63];
Gu2[16] = + Gx1[64]*Gu1[0] + Gx1[65]*Gu1[4] + Gx1[66]*Gu1[8] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[24] + Gx1[71]*Gu1[28] + Gx1[72]*Gu1[32] + Gx1[73]*Gu1[36] + Gx1[74]*Gu1[40] + Gx1[75]*Gu1[44] + Gx1[76]*Gu1[48] + Gx1[77]*Gu1[52] + Gx1[78]*Gu1[56] + Gx1[79]*Gu1[60];
Gu2[17] = + Gx1[64]*Gu1[1] + Gx1[65]*Gu1[5] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[21] + Gx1[70]*Gu1[25] + Gx1[71]*Gu1[29] + Gx1[72]*Gu1[33] + Gx1[73]*Gu1[37] + Gx1[74]*Gu1[41] + Gx1[75]*Gu1[45] + Gx1[76]*Gu1[49] + Gx1[77]*Gu1[53] + Gx1[78]*Gu1[57] + Gx1[79]*Gu1[61];
Gu2[18] = + Gx1[64]*Gu1[2] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[18] + Gx1[69]*Gu1[22] + Gx1[70]*Gu1[26] + Gx1[71]*Gu1[30] + Gx1[72]*Gu1[34] + Gx1[73]*Gu1[38] + Gx1[74]*Gu1[42] + Gx1[75]*Gu1[46] + Gx1[76]*Gu1[50] + Gx1[77]*Gu1[54] + Gx1[78]*Gu1[58] + Gx1[79]*Gu1[62];
Gu2[19] = + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[15] + Gx1[68]*Gu1[19] + Gx1[69]*Gu1[23] + Gx1[70]*Gu1[27] + Gx1[71]*Gu1[31] + Gx1[72]*Gu1[35] + Gx1[73]*Gu1[39] + Gx1[74]*Gu1[43] + Gx1[75]*Gu1[47] + Gx1[76]*Gu1[51] + Gx1[77]*Gu1[55] + Gx1[78]*Gu1[59] + Gx1[79]*Gu1[63];
Gu2[20] = + Gx1[80]*Gu1[0] + Gx1[81]*Gu1[4] + Gx1[82]*Gu1[8] + Gx1[83]*Gu1[12] + Gx1[84]*Gu1[16] + Gx1[85]*Gu1[20] + Gx1[86]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[89]*Gu1[36] + Gx1[90]*Gu1[40] + Gx1[91]*Gu1[44] + Gx1[92]*Gu1[48] + Gx1[93]*Gu1[52] + Gx1[94]*Gu1[56] + Gx1[95]*Gu1[60];
Gu2[21] = + Gx1[80]*Gu1[1] + Gx1[81]*Gu1[5] + Gx1[82]*Gu1[9] + Gx1[83]*Gu1[13] + Gx1[84]*Gu1[17] + Gx1[85]*Gu1[21] + Gx1[86]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[89]*Gu1[37] + Gx1[90]*Gu1[41] + Gx1[91]*Gu1[45] + Gx1[92]*Gu1[49] + Gx1[93]*Gu1[53] + Gx1[94]*Gu1[57] + Gx1[95]*Gu1[61];
Gu2[22] = + Gx1[80]*Gu1[2] + Gx1[81]*Gu1[6] + Gx1[82]*Gu1[10] + Gx1[83]*Gu1[14] + Gx1[84]*Gu1[18] + Gx1[85]*Gu1[22] + Gx1[86]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[89]*Gu1[38] + Gx1[90]*Gu1[42] + Gx1[91]*Gu1[46] + Gx1[92]*Gu1[50] + Gx1[93]*Gu1[54] + Gx1[94]*Gu1[58] + Gx1[95]*Gu1[62];
Gu2[23] = + Gx1[80]*Gu1[3] + Gx1[81]*Gu1[7] + Gx1[82]*Gu1[11] + Gx1[83]*Gu1[15] + Gx1[84]*Gu1[19] + Gx1[85]*Gu1[23] + Gx1[86]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[89]*Gu1[39] + Gx1[90]*Gu1[43] + Gx1[91]*Gu1[47] + Gx1[92]*Gu1[51] + Gx1[93]*Gu1[55] + Gx1[94]*Gu1[59] + Gx1[95]*Gu1[63];
Gu2[24] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44] + Gx1[108]*Gu1[48] + Gx1[109]*Gu1[52] + Gx1[110]*Gu1[56] + Gx1[111]*Gu1[60];
Gu2[25] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45] + Gx1[108]*Gu1[49] + Gx1[109]*Gu1[53] + Gx1[110]*Gu1[57] + Gx1[111]*Gu1[61];
Gu2[26] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46] + Gx1[108]*Gu1[50] + Gx1[109]*Gu1[54] + Gx1[110]*Gu1[58] + Gx1[111]*Gu1[62];
Gu2[27] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47] + Gx1[108]*Gu1[51] + Gx1[109]*Gu1[55] + Gx1[110]*Gu1[59] + Gx1[111]*Gu1[63];
Gu2[28] = + Gx1[112]*Gu1[0] + Gx1[113]*Gu1[4] + Gx1[114]*Gu1[8] + Gx1[115]*Gu1[12] + Gx1[116]*Gu1[16] + Gx1[117]*Gu1[20] + Gx1[118]*Gu1[24] + Gx1[119]*Gu1[28] + Gx1[120]*Gu1[32] + Gx1[121]*Gu1[36] + Gx1[122]*Gu1[40] + Gx1[123]*Gu1[44] + Gx1[124]*Gu1[48] + Gx1[125]*Gu1[52] + Gx1[126]*Gu1[56] + Gx1[127]*Gu1[60];
Gu2[29] = + Gx1[112]*Gu1[1] + Gx1[113]*Gu1[5] + Gx1[114]*Gu1[9] + Gx1[115]*Gu1[13] + Gx1[116]*Gu1[17] + Gx1[117]*Gu1[21] + Gx1[118]*Gu1[25] + Gx1[119]*Gu1[29] + Gx1[120]*Gu1[33] + Gx1[121]*Gu1[37] + Gx1[122]*Gu1[41] + Gx1[123]*Gu1[45] + Gx1[124]*Gu1[49] + Gx1[125]*Gu1[53] + Gx1[126]*Gu1[57] + Gx1[127]*Gu1[61];
Gu2[30] = + Gx1[112]*Gu1[2] + Gx1[113]*Gu1[6] + Gx1[114]*Gu1[10] + Gx1[115]*Gu1[14] + Gx1[116]*Gu1[18] + Gx1[117]*Gu1[22] + Gx1[118]*Gu1[26] + Gx1[119]*Gu1[30] + Gx1[120]*Gu1[34] + Gx1[121]*Gu1[38] + Gx1[122]*Gu1[42] + Gx1[123]*Gu1[46] + Gx1[124]*Gu1[50] + Gx1[125]*Gu1[54] + Gx1[126]*Gu1[58] + Gx1[127]*Gu1[62];
Gu2[31] = + Gx1[112]*Gu1[3] + Gx1[113]*Gu1[7] + Gx1[114]*Gu1[11] + Gx1[115]*Gu1[15] + Gx1[116]*Gu1[19] + Gx1[117]*Gu1[23] + Gx1[118]*Gu1[27] + Gx1[119]*Gu1[31] + Gx1[120]*Gu1[35] + Gx1[121]*Gu1[39] + Gx1[122]*Gu1[43] + Gx1[123]*Gu1[47] + Gx1[124]*Gu1[51] + Gx1[125]*Gu1[55] + Gx1[126]*Gu1[59] + Gx1[127]*Gu1[63];
Gu2[32] = + Gx1[128]*Gu1[0] + Gx1[129]*Gu1[4] + Gx1[130]*Gu1[8] + Gx1[131]*Gu1[12] + Gx1[132]*Gu1[16] + Gx1[133]*Gu1[20] + Gx1[134]*Gu1[24] + Gx1[135]*Gu1[28] + Gx1[136]*Gu1[32] + Gx1[137]*Gu1[36] + Gx1[138]*Gu1[40] + Gx1[139]*Gu1[44] + Gx1[140]*Gu1[48] + Gx1[141]*Gu1[52] + Gx1[142]*Gu1[56] + Gx1[143]*Gu1[60];
Gu2[33] = + Gx1[128]*Gu1[1] + Gx1[129]*Gu1[5] + Gx1[130]*Gu1[9] + Gx1[131]*Gu1[13] + Gx1[132]*Gu1[17] + Gx1[133]*Gu1[21] + Gx1[134]*Gu1[25] + Gx1[135]*Gu1[29] + Gx1[136]*Gu1[33] + Gx1[137]*Gu1[37] + Gx1[138]*Gu1[41] + Gx1[139]*Gu1[45] + Gx1[140]*Gu1[49] + Gx1[141]*Gu1[53] + Gx1[142]*Gu1[57] + Gx1[143]*Gu1[61];
Gu2[34] = + Gx1[128]*Gu1[2] + Gx1[129]*Gu1[6] + Gx1[130]*Gu1[10] + Gx1[131]*Gu1[14] + Gx1[132]*Gu1[18] + Gx1[133]*Gu1[22] + Gx1[134]*Gu1[26] + Gx1[135]*Gu1[30] + Gx1[136]*Gu1[34] + Gx1[137]*Gu1[38] + Gx1[138]*Gu1[42] + Gx1[139]*Gu1[46] + Gx1[140]*Gu1[50] + Gx1[141]*Gu1[54] + Gx1[142]*Gu1[58] + Gx1[143]*Gu1[62];
Gu2[35] = + Gx1[128]*Gu1[3] + Gx1[129]*Gu1[7] + Gx1[130]*Gu1[11] + Gx1[131]*Gu1[15] + Gx1[132]*Gu1[19] + Gx1[133]*Gu1[23] + Gx1[134]*Gu1[27] + Gx1[135]*Gu1[31] + Gx1[136]*Gu1[35] + Gx1[137]*Gu1[39] + Gx1[138]*Gu1[43] + Gx1[139]*Gu1[47] + Gx1[140]*Gu1[51] + Gx1[141]*Gu1[55] + Gx1[142]*Gu1[59] + Gx1[143]*Gu1[63];
Gu2[36] = + Gx1[144]*Gu1[0] + Gx1[145]*Gu1[4] + Gx1[146]*Gu1[8] + Gx1[147]*Gu1[12] + Gx1[148]*Gu1[16] + Gx1[149]*Gu1[20] + Gx1[150]*Gu1[24] + Gx1[151]*Gu1[28] + Gx1[152]*Gu1[32] + Gx1[153]*Gu1[36] + Gx1[154]*Gu1[40] + Gx1[155]*Gu1[44] + Gx1[156]*Gu1[48] + Gx1[157]*Gu1[52] + Gx1[158]*Gu1[56] + Gx1[159]*Gu1[60];
Gu2[37] = + Gx1[144]*Gu1[1] + Gx1[145]*Gu1[5] + Gx1[146]*Gu1[9] + Gx1[147]*Gu1[13] + Gx1[148]*Gu1[17] + Gx1[149]*Gu1[21] + Gx1[150]*Gu1[25] + Gx1[151]*Gu1[29] + Gx1[152]*Gu1[33] + Gx1[153]*Gu1[37] + Gx1[154]*Gu1[41] + Gx1[155]*Gu1[45] + Gx1[156]*Gu1[49] + Gx1[157]*Gu1[53] + Gx1[158]*Gu1[57] + Gx1[159]*Gu1[61];
Gu2[38] = + Gx1[144]*Gu1[2] + Gx1[145]*Gu1[6] + Gx1[146]*Gu1[10] + Gx1[147]*Gu1[14] + Gx1[148]*Gu1[18] + Gx1[149]*Gu1[22] + Gx1[150]*Gu1[26] + Gx1[151]*Gu1[30] + Gx1[152]*Gu1[34] + Gx1[153]*Gu1[38] + Gx1[154]*Gu1[42] + Gx1[155]*Gu1[46] + Gx1[156]*Gu1[50] + Gx1[157]*Gu1[54] + Gx1[158]*Gu1[58] + Gx1[159]*Gu1[62];
Gu2[39] = + Gx1[144]*Gu1[3] + Gx1[145]*Gu1[7] + Gx1[146]*Gu1[11] + Gx1[147]*Gu1[15] + Gx1[148]*Gu1[19] + Gx1[149]*Gu1[23] + Gx1[150]*Gu1[27] + Gx1[151]*Gu1[31] + Gx1[152]*Gu1[35] + Gx1[153]*Gu1[39] + Gx1[154]*Gu1[43] + Gx1[155]*Gu1[47] + Gx1[156]*Gu1[51] + Gx1[157]*Gu1[55] + Gx1[158]*Gu1[59] + Gx1[159]*Gu1[63];
Gu2[40] = + Gx1[160]*Gu1[0] + Gx1[161]*Gu1[4] + Gx1[162]*Gu1[8] + Gx1[163]*Gu1[12] + Gx1[164]*Gu1[16] + Gx1[165]*Gu1[20] + Gx1[166]*Gu1[24] + Gx1[167]*Gu1[28] + Gx1[168]*Gu1[32] + Gx1[169]*Gu1[36] + Gx1[170]*Gu1[40] + Gx1[171]*Gu1[44] + Gx1[172]*Gu1[48] + Gx1[173]*Gu1[52] + Gx1[174]*Gu1[56] + Gx1[175]*Gu1[60];
Gu2[41] = + Gx1[160]*Gu1[1] + Gx1[161]*Gu1[5] + Gx1[162]*Gu1[9] + Gx1[163]*Gu1[13] + Gx1[164]*Gu1[17] + Gx1[165]*Gu1[21] + Gx1[166]*Gu1[25] + Gx1[167]*Gu1[29] + Gx1[168]*Gu1[33] + Gx1[169]*Gu1[37] + Gx1[170]*Gu1[41] + Gx1[171]*Gu1[45] + Gx1[172]*Gu1[49] + Gx1[173]*Gu1[53] + Gx1[174]*Gu1[57] + Gx1[175]*Gu1[61];
Gu2[42] = + Gx1[160]*Gu1[2] + Gx1[161]*Gu1[6] + Gx1[162]*Gu1[10] + Gx1[163]*Gu1[14] + Gx1[164]*Gu1[18] + Gx1[165]*Gu1[22] + Gx1[166]*Gu1[26] + Gx1[167]*Gu1[30] + Gx1[168]*Gu1[34] + Gx1[169]*Gu1[38] + Gx1[170]*Gu1[42] + Gx1[171]*Gu1[46] + Gx1[172]*Gu1[50] + Gx1[173]*Gu1[54] + Gx1[174]*Gu1[58] + Gx1[175]*Gu1[62];
Gu2[43] = + Gx1[160]*Gu1[3] + Gx1[161]*Gu1[7] + Gx1[162]*Gu1[11] + Gx1[163]*Gu1[15] + Gx1[164]*Gu1[19] + Gx1[165]*Gu1[23] + Gx1[166]*Gu1[27] + Gx1[167]*Gu1[31] + Gx1[168]*Gu1[35] + Gx1[169]*Gu1[39] + Gx1[170]*Gu1[43] + Gx1[171]*Gu1[47] + Gx1[172]*Gu1[51] + Gx1[173]*Gu1[55] + Gx1[174]*Gu1[59] + Gx1[175]*Gu1[63];
Gu2[44] = + Gx1[176]*Gu1[0] + Gx1[177]*Gu1[4] + Gx1[178]*Gu1[8] + Gx1[179]*Gu1[12] + Gx1[180]*Gu1[16] + Gx1[181]*Gu1[20] + Gx1[182]*Gu1[24] + Gx1[183]*Gu1[28] + Gx1[184]*Gu1[32] + Gx1[185]*Gu1[36] + Gx1[186]*Gu1[40] + Gx1[187]*Gu1[44] + Gx1[188]*Gu1[48] + Gx1[189]*Gu1[52] + Gx1[190]*Gu1[56] + Gx1[191]*Gu1[60];
Gu2[45] = + Gx1[176]*Gu1[1] + Gx1[177]*Gu1[5] + Gx1[178]*Gu1[9] + Gx1[179]*Gu1[13] + Gx1[180]*Gu1[17] + Gx1[181]*Gu1[21] + Gx1[182]*Gu1[25] + Gx1[183]*Gu1[29] + Gx1[184]*Gu1[33] + Gx1[185]*Gu1[37] + Gx1[186]*Gu1[41] + Gx1[187]*Gu1[45] + Gx1[188]*Gu1[49] + Gx1[189]*Gu1[53] + Gx1[190]*Gu1[57] + Gx1[191]*Gu1[61];
Gu2[46] = + Gx1[176]*Gu1[2] + Gx1[177]*Gu1[6] + Gx1[178]*Gu1[10] + Gx1[179]*Gu1[14] + Gx1[180]*Gu1[18] + Gx1[181]*Gu1[22] + Gx1[182]*Gu1[26] + Gx1[183]*Gu1[30] + Gx1[184]*Gu1[34] + Gx1[185]*Gu1[38] + Gx1[186]*Gu1[42] + Gx1[187]*Gu1[46] + Gx1[188]*Gu1[50] + Gx1[189]*Gu1[54] + Gx1[190]*Gu1[58] + Gx1[191]*Gu1[62];
Gu2[47] = + Gx1[176]*Gu1[3] + Gx1[177]*Gu1[7] + Gx1[178]*Gu1[11] + Gx1[179]*Gu1[15] + Gx1[180]*Gu1[19] + Gx1[181]*Gu1[23] + Gx1[182]*Gu1[27] + Gx1[183]*Gu1[31] + Gx1[184]*Gu1[35] + Gx1[185]*Gu1[39] + Gx1[186]*Gu1[43] + Gx1[187]*Gu1[47] + Gx1[188]*Gu1[51] + Gx1[189]*Gu1[55] + Gx1[190]*Gu1[59] + Gx1[191]*Gu1[63];
Gu2[48] = + Gx1[192]*Gu1[0] + Gx1[193]*Gu1[4] + Gx1[194]*Gu1[8] + Gx1[195]*Gu1[12] + Gx1[196]*Gu1[16] + Gx1[197]*Gu1[20] + Gx1[198]*Gu1[24] + Gx1[199]*Gu1[28] + Gx1[200]*Gu1[32] + Gx1[201]*Gu1[36] + Gx1[202]*Gu1[40] + Gx1[203]*Gu1[44] + Gx1[204]*Gu1[48] + Gx1[205]*Gu1[52] + Gx1[206]*Gu1[56] + Gx1[207]*Gu1[60];
Gu2[49] = + Gx1[192]*Gu1[1] + Gx1[193]*Gu1[5] + Gx1[194]*Gu1[9] + Gx1[195]*Gu1[13] + Gx1[196]*Gu1[17] + Gx1[197]*Gu1[21] + Gx1[198]*Gu1[25] + Gx1[199]*Gu1[29] + Gx1[200]*Gu1[33] + Gx1[201]*Gu1[37] + Gx1[202]*Gu1[41] + Gx1[203]*Gu1[45] + Gx1[204]*Gu1[49] + Gx1[205]*Gu1[53] + Gx1[206]*Gu1[57] + Gx1[207]*Gu1[61];
Gu2[50] = + Gx1[192]*Gu1[2] + Gx1[193]*Gu1[6] + Gx1[194]*Gu1[10] + Gx1[195]*Gu1[14] + Gx1[196]*Gu1[18] + Gx1[197]*Gu1[22] + Gx1[198]*Gu1[26] + Gx1[199]*Gu1[30] + Gx1[200]*Gu1[34] + Gx1[201]*Gu1[38] + Gx1[202]*Gu1[42] + Gx1[203]*Gu1[46] + Gx1[204]*Gu1[50] + Gx1[205]*Gu1[54] + Gx1[206]*Gu1[58] + Gx1[207]*Gu1[62];
Gu2[51] = + Gx1[192]*Gu1[3] + Gx1[193]*Gu1[7] + Gx1[194]*Gu1[11] + Gx1[195]*Gu1[15] + Gx1[196]*Gu1[19] + Gx1[197]*Gu1[23] + Gx1[198]*Gu1[27] + Gx1[199]*Gu1[31] + Gx1[200]*Gu1[35] + Gx1[201]*Gu1[39] + Gx1[202]*Gu1[43] + Gx1[203]*Gu1[47] + Gx1[204]*Gu1[51] + Gx1[205]*Gu1[55] + Gx1[206]*Gu1[59] + Gx1[207]*Gu1[63];
Gu2[52] = + Gx1[208]*Gu1[0] + Gx1[209]*Gu1[4] + Gx1[210]*Gu1[8] + Gx1[211]*Gu1[12] + Gx1[212]*Gu1[16] + Gx1[213]*Gu1[20] + Gx1[214]*Gu1[24] + Gx1[215]*Gu1[28] + Gx1[216]*Gu1[32] + Gx1[217]*Gu1[36] + Gx1[218]*Gu1[40] + Gx1[219]*Gu1[44] + Gx1[220]*Gu1[48] + Gx1[221]*Gu1[52] + Gx1[222]*Gu1[56] + Gx1[223]*Gu1[60];
Gu2[53] = + Gx1[208]*Gu1[1] + Gx1[209]*Gu1[5] + Gx1[210]*Gu1[9] + Gx1[211]*Gu1[13] + Gx1[212]*Gu1[17] + Gx1[213]*Gu1[21] + Gx1[214]*Gu1[25] + Gx1[215]*Gu1[29] + Gx1[216]*Gu1[33] + Gx1[217]*Gu1[37] + Gx1[218]*Gu1[41] + Gx1[219]*Gu1[45] + Gx1[220]*Gu1[49] + Gx1[221]*Gu1[53] + Gx1[222]*Gu1[57] + Gx1[223]*Gu1[61];
Gu2[54] = + Gx1[208]*Gu1[2] + Gx1[209]*Gu1[6] + Gx1[210]*Gu1[10] + Gx1[211]*Gu1[14] + Gx1[212]*Gu1[18] + Gx1[213]*Gu1[22] + Gx1[214]*Gu1[26] + Gx1[215]*Gu1[30] + Gx1[216]*Gu1[34] + Gx1[217]*Gu1[38] + Gx1[218]*Gu1[42] + Gx1[219]*Gu1[46] + Gx1[220]*Gu1[50] + Gx1[221]*Gu1[54] + Gx1[222]*Gu1[58] + Gx1[223]*Gu1[62];
Gu2[55] = + Gx1[208]*Gu1[3] + Gx1[209]*Gu1[7] + Gx1[210]*Gu1[11] + Gx1[211]*Gu1[15] + Gx1[212]*Gu1[19] + Gx1[213]*Gu1[23] + Gx1[214]*Gu1[27] + Gx1[215]*Gu1[31] + Gx1[216]*Gu1[35] + Gx1[217]*Gu1[39] + Gx1[218]*Gu1[43] + Gx1[219]*Gu1[47] + Gx1[220]*Gu1[51] + Gx1[221]*Gu1[55] + Gx1[222]*Gu1[59] + Gx1[223]*Gu1[63];
Gu2[56] = + Gx1[224]*Gu1[0] + Gx1[225]*Gu1[4] + Gx1[226]*Gu1[8] + Gx1[227]*Gu1[12] + Gx1[228]*Gu1[16] + Gx1[229]*Gu1[20] + Gx1[230]*Gu1[24] + Gx1[231]*Gu1[28] + Gx1[232]*Gu1[32] + Gx1[233]*Gu1[36] + Gx1[234]*Gu1[40] + Gx1[235]*Gu1[44] + Gx1[236]*Gu1[48] + Gx1[237]*Gu1[52] + Gx1[238]*Gu1[56] + Gx1[239]*Gu1[60];
Gu2[57] = + Gx1[224]*Gu1[1] + Gx1[225]*Gu1[5] + Gx1[226]*Gu1[9] + Gx1[227]*Gu1[13] + Gx1[228]*Gu1[17] + Gx1[229]*Gu1[21] + Gx1[230]*Gu1[25] + Gx1[231]*Gu1[29] + Gx1[232]*Gu1[33] + Gx1[233]*Gu1[37] + Gx1[234]*Gu1[41] + Gx1[235]*Gu1[45] + Gx1[236]*Gu1[49] + Gx1[237]*Gu1[53] + Gx1[238]*Gu1[57] + Gx1[239]*Gu1[61];
Gu2[58] = + Gx1[224]*Gu1[2] + Gx1[225]*Gu1[6] + Gx1[226]*Gu1[10] + Gx1[227]*Gu1[14] + Gx1[228]*Gu1[18] + Gx1[229]*Gu1[22] + Gx1[230]*Gu1[26] + Gx1[231]*Gu1[30] + Gx1[232]*Gu1[34] + Gx1[233]*Gu1[38] + Gx1[234]*Gu1[42] + Gx1[235]*Gu1[46] + Gx1[236]*Gu1[50] + Gx1[237]*Gu1[54] + Gx1[238]*Gu1[58] + Gx1[239]*Gu1[62];
Gu2[59] = + Gx1[224]*Gu1[3] + Gx1[225]*Gu1[7] + Gx1[226]*Gu1[11] + Gx1[227]*Gu1[15] + Gx1[228]*Gu1[19] + Gx1[229]*Gu1[23] + Gx1[230]*Gu1[27] + Gx1[231]*Gu1[31] + Gx1[232]*Gu1[35] + Gx1[233]*Gu1[39] + Gx1[234]*Gu1[43] + Gx1[235]*Gu1[47] + Gx1[236]*Gu1[51] + Gx1[237]*Gu1[55] + Gx1[238]*Gu1[59] + Gx1[239]*Gu1[63];
Gu2[60] = + Gx1[240]*Gu1[0] + Gx1[241]*Gu1[4] + Gx1[242]*Gu1[8] + Gx1[243]*Gu1[12] + Gx1[244]*Gu1[16] + Gx1[245]*Gu1[20] + Gx1[246]*Gu1[24] + Gx1[247]*Gu1[28] + Gx1[248]*Gu1[32] + Gx1[249]*Gu1[36] + Gx1[250]*Gu1[40] + Gx1[251]*Gu1[44] + Gx1[252]*Gu1[48] + Gx1[253]*Gu1[52] + Gx1[254]*Gu1[56] + Gx1[255]*Gu1[60];
Gu2[61] = + Gx1[240]*Gu1[1] + Gx1[241]*Gu1[5] + Gx1[242]*Gu1[9] + Gx1[243]*Gu1[13] + Gx1[244]*Gu1[17] + Gx1[245]*Gu1[21] + Gx1[246]*Gu1[25] + Gx1[247]*Gu1[29] + Gx1[248]*Gu1[33] + Gx1[249]*Gu1[37] + Gx1[250]*Gu1[41] + Gx1[251]*Gu1[45] + Gx1[252]*Gu1[49] + Gx1[253]*Gu1[53] + Gx1[254]*Gu1[57] + Gx1[255]*Gu1[61];
Gu2[62] = + Gx1[240]*Gu1[2] + Gx1[241]*Gu1[6] + Gx1[242]*Gu1[10] + Gx1[243]*Gu1[14] + Gx1[244]*Gu1[18] + Gx1[245]*Gu1[22] + Gx1[246]*Gu1[26] + Gx1[247]*Gu1[30] + Gx1[248]*Gu1[34] + Gx1[249]*Gu1[38] + Gx1[250]*Gu1[42] + Gx1[251]*Gu1[46] + Gx1[252]*Gu1[50] + Gx1[253]*Gu1[54] + Gx1[254]*Gu1[58] + Gx1[255]*Gu1[62];
Gu2[63] = + Gx1[240]*Gu1[3] + Gx1[241]*Gu1[7] + Gx1[242]*Gu1[11] + Gx1[243]*Gu1[15] + Gx1[244]*Gu1[19] + Gx1[245]*Gu1[23] + Gx1[246]*Gu1[27] + Gx1[247]*Gu1[31] + Gx1[248]*Gu1[35] + Gx1[249]*Gu1[39] + Gx1[250]*Gu1[43] + Gx1[251]*Gu1[47] + Gx1[252]*Gu1[51] + Gx1[253]*Gu1[55] + Gx1[254]*Gu1[59] + Gx1[255]*Gu1[63];
}

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
Gu2[48] = Gu1[48];
Gu2[49] = Gu1[49];
Gu2[50] = Gu1[50];
Gu2[51] = Gu1[51];
Gu2[52] = Gu1[52];
Gu2[53] = Gu1[53];
Gu2[54] = Gu1[54];
Gu2[55] = Gu1[55];
Gu2[56] = Gu1[56];
Gu2[57] = Gu1[57];
Gu2[58] = Gu1[58];
Gu2[59] = Gu1[59];
Gu2[60] = Gu1[60];
Gu2[61] = Gu1[61];
Gu2[62] = Gu1[62];
Gu2[63] = Gu1[63];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44] + Gu1[48]*Gu2[48] + Gu1[52]*Gu2[52] + Gu1[56]*Gu2[56] + Gu1[60]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45] + Gu1[48]*Gu2[49] + Gu1[52]*Gu2[53] + Gu1[56]*Gu2[57] + Gu1[60]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46] + Gu1[48]*Gu2[50] + Gu1[52]*Gu2[54] + Gu1[56]*Gu2[58] + Gu1[60]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47] + Gu1[48]*Gu2[51] + Gu1[52]*Gu2[55] + Gu1[56]*Gu2[59] + Gu1[60]*Gu2[63];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44] + Gu1[49]*Gu2[48] + Gu1[53]*Gu2[52] + Gu1[57]*Gu2[56] + Gu1[61]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45] + Gu1[49]*Gu2[49] + Gu1[53]*Gu2[53] + Gu1[57]*Gu2[57] + Gu1[61]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46] + Gu1[49]*Gu2[50] + Gu1[53]*Gu2[54] + Gu1[57]*Gu2[58] + Gu1[61]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47] + Gu1[49]*Gu2[51] + Gu1[53]*Gu2[55] + Gu1[57]*Gu2[59] + Gu1[61]*Gu2[63];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44] + Gu1[50]*Gu2[48] + Gu1[54]*Gu2[52] + Gu1[58]*Gu2[56] + Gu1[62]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45] + Gu1[50]*Gu2[49] + Gu1[54]*Gu2[53] + Gu1[58]*Gu2[57] + Gu1[62]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46] + Gu1[50]*Gu2[50] + Gu1[54]*Gu2[54] + Gu1[58]*Gu2[58] + Gu1[62]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47] + Gu1[50]*Gu2[51] + Gu1[54]*Gu2[55] + Gu1[58]*Gu2[59] + Gu1[62]*Gu2[63];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44] + Gu1[51]*Gu2[48] + Gu1[55]*Gu2[52] + Gu1[59]*Gu2[56] + Gu1[63]*Gu2[60];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45] + Gu1[51]*Gu2[49] + Gu1[55]*Gu2[53] + Gu1[59]*Gu2[57] + Gu1[63]*Gu2[61];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46] + Gu1[51]*Gu2[50] + Gu1[55]*Gu2[54] + Gu1[59]*Gu2[58] + Gu1[63]*Gu2[62];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47] + Gu1[51]*Gu2[51] + Gu1[55]*Gu2[55] + Gu1[59]*Gu2[59] + Gu1[63]*Gu2[63];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] = R11[0];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] = R11[1];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] = R11[2];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] = R11[3];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] = R11[4];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] = R11[5];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] = R11[6];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] = R11[7];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] = R11[8];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] = R11[9];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] = R11[10];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] = R11[11];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] = R11[12];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] = R11[13];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] = R11[14];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2176) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 16)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2312) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 17)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2448) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 18)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 16)] = nmpcWorkspace.H[(iCol * 544 + 2176) + (iRow * 4 + 19)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 17)] = nmpcWorkspace.H[(iCol * 544 + 2312) + (iRow * 4 + 19)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 18)] = nmpcWorkspace.H[(iCol * 544 + 2448) + (iRow * 4 + 19)];
nmpcWorkspace.H[(iRow * 544 + 2584) + (iCol * 4 + 19)] = nmpcWorkspace.H[(iCol * 544 + 2584) + (iRow * 4 + 19)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11] + Gx1[12]*dOld[12] + Gx1[13]*dOld[13] + Gx1[14]*dOld[14] + Gx1[15]*dOld[15];
dNew[1] = + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7] + Gx1[24]*dOld[8] + Gx1[25]*dOld[9] + Gx1[26]*dOld[10] + Gx1[27]*dOld[11] + Gx1[28]*dOld[12] + Gx1[29]*dOld[13] + Gx1[30]*dOld[14] + Gx1[31]*dOld[15];
dNew[2] = + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7] + Gx1[40]*dOld[8] + Gx1[41]*dOld[9] + Gx1[42]*dOld[10] + Gx1[43]*dOld[11] + Gx1[44]*dOld[12] + Gx1[45]*dOld[13] + Gx1[46]*dOld[14] + Gx1[47]*dOld[15];
dNew[3] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11] + Gx1[60]*dOld[12] + Gx1[61]*dOld[13] + Gx1[62]*dOld[14] + Gx1[63]*dOld[15];
dNew[4] = + Gx1[64]*dOld[0] + Gx1[65]*dOld[1] + Gx1[66]*dOld[2] + Gx1[67]*dOld[3] + Gx1[68]*dOld[4] + Gx1[69]*dOld[5] + Gx1[70]*dOld[6] + Gx1[71]*dOld[7] + Gx1[72]*dOld[8] + Gx1[73]*dOld[9] + Gx1[74]*dOld[10] + Gx1[75]*dOld[11] + Gx1[76]*dOld[12] + Gx1[77]*dOld[13] + Gx1[78]*dOld[14] + Gx1[79]*dOld[15];
dNew[5] = + Gx1[80]*dOld[0] + Gx1[81]*dOld[1] + Gx1[82]*dOld[2] + Gx1[83]*dOld[3] + Gx1[84]*dOld[4] + Gx1[85]*dOld[5] + Gx1[86]*dOld[6] + Gx1[87]*dOld[7] + Gx1[88]*dOld[8] + Gx1[89]*dOld[9] + Gx1[90]*dOld[10] + Gx1[91]*dOld[11] + Gx1[92]*dOld[12] + Gx1[93]*dOld[13] + Gx1[94]*dOld[14] + Gx1[95]*dOld[15];
dNew[6] = + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11] + Gx1[108]*dOld[12] + Gx1[109]*dOld[13] + Gx1[110]*dOld[14] + Gx1[111]*dOld[15];
dNew[7] = + Gx1[112]*dOld[0] + Gx1[113]*dOld[1] + Gx1[114]*dOld[2] + Gx1[115]*dOld[3] + Gx1[116]*dOld[4] + Gx1[117]*dOld[5] + Gx1[118]*dOld[6] + Gx1[119]*dOld[7] + Gx1[120]*dOld[8] + Gx1[121]*dOld[9] + Gx1[122]*dOld[10] + Gx1[123]*dOld[11] + Gx1[124]*dOld[12] + Gx1[125]*dOld[13] + Gx1[126]*dOld[14] + Gx1[127]*dOld[15];
dNew[8] = + Gx1[128]*dOld[0] + Gx1[129]*dOld[1] + Gx1[130]*dOld[2] + Gx1[131]*dOld[3] + Gx1[132]*dOld[4] + Gx1[133]*dOld[5] + Gx1[134]*dOld[6] + Gx1[135]*dOld[7] + Gx1[136]*dOld[8] + Gx1[137]*dOld[9] + Gx1[138]*dOld[10] + Gx1[139]*dOld[11] + Gx1[140]*dOld[12] + Gx1[141]*dOld[13] + Gx1[142]*dOld[14] + Gx1[143]*dOld[15];
dNew[9] = + Gx1[144]*dOld[0] + Gx1[145]*dOld[1] + Gx1[146]*dOld[2] + Gx1[147]*dOld[3] + Gx1[148]*dOld[4] + Gx1[149]*dOld[5] + Gx1[150]*dOld[6] + Gx1[151]*dOld[7] + Gx1[152]*dOld[8] + Gx1[153]*dOld[9] + Gx1[154]*dOld[10] + Gx1[155]*dOld[11] + Gx1[156]*dOld[12] + Gx1[157]*dOld[13] + Gx1[158]*dOld[14] + Gx1[159]*dOld[15];
dNew[10] = + Gx1[160]*dOld[0] + Gx1[161]*dOld[1] + Gx1[162]*dOld[2] + Gx1[163]*dOld[3] + Gx1[164]*dOld[4] + Gx1[165]*dOld[5] + Gx1[166]*dOld[6] + Gx1[167]*dOld[7] + Gx1[168]*dOld[8] + Gx1[169]*dOld[9] + Gx1[170]*dOld[10] + Gx1[171]*dOld[11] + Gx1[172]*dOld[12] + Gx1[173]*dOld[13] + Gx1[174]*dOld[14] + Gx1[175]*dOld[15];
dNew[11] = + Gx1[176]*dOld[0] + Gx1[177]*dOld[1] + Gx1[178]*dOld[2] + Gx1[179]*dOld[3] + Gx1[180]*dOld[4] + Gx1[181]*dOld[5] + Gx1[182]*dOld[6] + Gx1[183]*dOld[7] + Gx1[184]*dOld[8] + Gx1[185]*dOld[9] + Gx1[186]*dOld[10] + Gx1[187]*dOld[11] + Gx1[188]*dOld[12] + Gx1[189]*dOld[13] + Gx1[190]*dOld[14] + Gx1[191]*dOld[15];
dNew[12] = + Gx1[192]*dOld[0] + Gx1[193]*dOld[1] + Gx1[194]*dOld[2] + Gx1[195]*dOld[3] + Gx1[196]*dOld[4] + Gx1[197]*dOld[5] + Gx1[198]*dOld[6] + Gx1[199]*dOld[7] + Gx1[200]*dOld[8] + Gx1[201]*dOld[9] + Gx1[202]*dOld[10] + Gx1[203]*dOld[11] + Gx1[204]*dOld[12] + Gx1[205]*dOld[13] + Gx1[206]*dOld[14] + Gx1[207]*dOld[15];
dNew[13] = + Gx1[208]*dOld[0] + Gx1[209]*dOld[1] + Gx1[210]*dOld[2] + Gx1[211]*dOld[3] + Gx1[212]*dOld[4] + Gx1[213]*dOld[5] + Gx1[214]*dOld[6] + Gx1[215]*dOld[7] + Gx1[216]*dOld[8] + Gx1[217]*dOld[9] + Gx1[218]*dOld[10] + Gx1[219]*dOld[11] + Gx1[220]*dOld[12] + Gx1[221]*dOld[13] + Gx1[222]*dOld[14] + Gx1[223]*dOld[15];
dNew[14] = + Gx1[224]*dOld[0] + Gx1[225]*dOld[1] + Gx1[226]*dOld[2] + Gx1[227]*dOld[3] + Gx1[228]*dOld[4] + Gx1[229]*dOld[5] + Gx1[230]*dOld[6] + Gx1[231]*dOld[7] + Gx1[232]*dOld[8] + Gx1[233]*dOld[9] + Gx1[234]*dOld[10] + Gx1[235]*dOld[11] + Gx1[236]*dOld[12] + Gx1[237]*dOld[13] + Gx1[238]*dOld[14] + Gx1[239]*dOld[15];
dNew[15] = + Gx1[240]*dOld[0] + Gx1[241]*dOld[1] + Gx1[242]*dOld[2] + Gx1[243]*dOld[3] + Gx1[244]*dOld[4] + Gx1[245]*dOld[5] + Gx1[246]*dOld[6] + Gx1[247]*dOld[7] + Gx1[248]*dOld[8] + Gx1[249]*dOld[9] + Gx1[250]*dOld[10] + Gx1[251]*dOld[11] + Gx1[252]*dOld[12] + Gx1[253]*dOld[13] + Gx1[254]*dOld[14] + Gx1[255]*dOld[15];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5] + nmpcWorkspace.QN1[6]*dOld[6] + nmpcWorkspace.QN1[7]*dOld[7] + nmpcWorkspace.QN1[8]*dOld[8] + nmpcWorkspace.QN1[9]*dOld[9] + nmpcWorkspace.QN1[10]*dOld[10] + nmpcWorkspace.QN1[11]*dOld[11] + nmpcWorkspace.QN1[12]*dOld[12] + nmpcWorkspace.QN1[13]*dOld[13] + nmpcWorkspace.QN1[14]*dOld[14] + nmpcWorkspace.QN1[15]*dOld[15];
dNew[1] = + nmpcWorkspace.QN1[16]*dOld[0] + nmpcWorkspace.QN1[17]*dOld[1] + nmpcWorkspace.QN1[18]*dOld[2] + nmpcWorkspace.QN1[19]*dOld[3] + nmpcWorkspace.QN1[20]*dOld[4] + nmpcWorkspace.QN1[21]*dOld[5] + nmpcWorkspace.QN1[22]*dOld[6] + nmpcWorkspace.QN1[23]*dOld[7] + nmpcWorkspace.QN1[24]*dOld[8] + nmpcWorkspace.QN1[25]*dOld[9] + nmpcWorkspace.QN1[26]*dOld[10] + nmpcWorkspace.QN1[27]*dOld[11] + nmpcWorkspace.QN1[28]*dOld[12] + nmpcWorkspace.QN1[29]*dOld[13] + nmpcWorkspace.QN1[30]*dOld[14] + nmpcWorkspace.QN1[31]*dOld[15];
dNew[2] = + nmpcWorkspace.QN1[32]*dOld[0] + nmpcWorkspace.QN1[33]*dOld[1] + nmpcWorkspace.QN1[34]*dOld[2] + nmpcWorkspace.QN1[35]*dOld[3] + nmpcWorkspace.QN1[36]*dOld[4] + nmpcWorkspace.QN1[37]*dOld[5] + nmpcWorkspace.QN1[38]*dOld[6] + nmpcWorkspace.QN1[39]*dOld[7] + nmpcWorkspace.QN1[40]*dOld[8] + nmpcWorkspace.QN1[41]*dOld[9] + nmpcWorkspace.QN1[42]*dOld[10] + nmpcWorkspace.QN1[43]*dOld[11] + nmpcWorkspace.QN1[44]*dOld[12] + nmpcWorkspace.QN1[45]*dOld[13] + nmpcWorkspace.QN1[46]*dOld[14] + nmpcWorkspace.QN1[47]*dOld[15];
dNew[3] = + nmpcWorkspace.QN1[48]*dOld[0] + nmpcWorkspace.QN1[49]*dOld[1] + nmpcWorkspace.QN1[50]*dOld[2] + nmpcWorkspace.QN1[51]*dOld[3] + nmpcWorkspace.QN1[52]*dOld[4] + nmpcWorkspace.QN1[53]*dOld[5] + nmpcWorkspace.QN1[54]*dOld[6] + nmpcWorkspace.QN1[55]*dOld[7] + nmpcWorkspace.QN1[56]*dOld[8] + nmpcWorkspace.QN1[57]*dOld[9] + nmpcWorkspace.QN1[58]*dOld[10] + nmpcWorkspace.QN1[59]*dOld[11] + nmpcWorkspace.QN1[60]*dOld[12] + nmpcWorkspace.QN1[61]*dOld[13] + nmpcWorkspace.QN1[62]*dOld[14] + nmpcWorkspace.QN1[63]*dOld[15];
dNew[4] = + nmpcWorkspace.QN1[64]*dOld[0] + nmpcWorkspace.QN1[65]*dOld[1] + nmpcWorkspace.QN1[66]*dOld[2] + nmpcWorkspace.QN1[67]*dOld[3] + nmpcWorkspace.QN1[68]*dOld[4] + nmpcWorkspace.QN1[69]*dOld[5] + nmpcWorkspace.QN1[70]*dOld[6] + nmpcWorkspace.QN1[71]*dOld[7] + nmpcWorkspace.QN1[72]*dOld[8] + nmpcWorkspace.QN1[73]*dOld[9] + nmpcWorkspace.QN1[74]*dOld[10] + nmpcWorkspace.QN1[75]*dOld[11] + nmpcWorkspace.QN1[76]*dOld[12] + nmpcWorkspace.QN1[77]*dOld[13] + nmpcWorkspace.QN1[78]*dOld[14] + nmpcWorkspace.QN1[79]*dOld[15];
dNew[5] = + nmpcWorkspace.QN1[80]*dOld[0] + nmpcWorkspace.QN1[81]*dOld[1] + nmpcWorkspace.QN1[82]*dOld[2] + nmpcWorkspace.QN1[83]*dOld[3] + nmpcWorkspace.QN1[84]*dOld[4] + nmpcWorkspace.QN1[85]*dOld[5] + nmpcWorkspace.QN1[86]*dOld[6] + nmpcWorkspace.QN1[87]*dOld[7] + nmpcWorkspace.QN1[88]*dOld[8] + nmpcWorkspace.QN1[89]*dOld[9] + nmpcWorkspace.QN1[90]*dOld[10] + nmpcWorkspace.QN1[91]*dOld[11] + nmpcWorkspace.QN1[92]*dOld[12] + nmpcWorkspace.QN1[93]*dOld[13] + nmpcWorkspace.QN1[94]*dOld[14] + nmpcWorkspace.QN1[95]*dOld[15];
dNew[6] = + nmpcWorkspace.QN1[96]*dOld[0] + nmpcWorkspace.QN1[97]*dOld[1] + nmpcWorkspace.QN1[98]*dOld[2] + nmpcWorkspace.QN1[99]*dOld[3] + nmpcWorkspace.QN1[100]*dOld[4] + nmpcWorkspace.QN1[101]*dOld[5] + nmpcWorkspace.QN1[102]*dOld[6] + nmpcWorkspace.QN1[103]*dOld[7] + nmpcWorkspace.QN1[104]*dOld[8] + nmpcWorkspace.QN1[105]*dOld[9] + nmpcWorkspace.QN1[106]*dOld[10] + nmpcWorkspace.QN1[107]*dOld[11] + nmpcWorkspace.QN1[108]*dOld[12] + nmpcWorkspace.QN1[109]*dOld[13] + nmpcWorkspace.QN1[110]*dOld[14] + nmpcWorkspace.QN1[111]*dOld[15];
dNew[7] = + nmpcWorkspace.QN1[112]*dOld[0] + nmpcWorkspace.QN1[113]*dOld[1] + nmpcWorkspace.QN1[114]*dOld[2] + nmpcWorkspace.QN1[115]*dOld[3] + nmpcWorkspace.QN1[116]*dOld[4] + nmpcWorkspace.QN1[117]*dOld[5] + nmpcWorkspace.QN1[118]*dOld[6] + nmpcWorkspace.QN1[119]*dOld[7] + nmpcWorkspace.QN1[120]*dOld[8] + nmpcWorkspace.QN1[121]*dOld[9] + nmpcWorkspace.QN1[122]*dOld[10] + nmpcWorkspace.QN1[123]*dOld[11] + nmpcWorkspace.QN1[124]*dOld[12] + nmpcWorkspace.QN1[125]*dOld[13] + nmpcWorkspace.QN1[126]*dOld[14] + nmpcWorkspace.QN1[127]*dOld[15];
dNew[8] = + nmpcWorkspace.QN1[128]*dOld[0] + nmpcWorkspace.QN1[129]*dOld[1] + nmpcWorkspace.QN1[130]*dOld[2] + nmpcWorkspace.QN1[131]*dOld[3] + nmpcWorkspace.QN1[132]*dOld[4] + nmpcWorkspace.QN1[133]*dOld[5] + nmpcWorkspace.QN1[134]*dOld[6] + nmpcWorkspace.QN1[135]*dOld[7] + nmpcWorkspace.QN1[136]*dOld[8] + nmpcWorkspace.QN1[137]*dOld[9] + nmpcWorkspace.QN1[138]*dOld[10] + nmpcWorkspace.QN1[139]*dOld[11] + nmpcWorkspace.QN1[140]*dOld[12] + nmpcWorkspace.QN1[141]*dOld[13] + nmpcWorkspace.QN1[142]*dOld[14] + nmpcWorkspace.QN1[143]*dOld[15];
dNew[9] = + nmpcWorkspace.QN1[144]*dOld[0] + nmpcWorkspace.QN1[145]*dOld[1] + nmpcWorkspace.QN1[146]*dOld[2] + nmpcWorkspace.QN1[147]*dOld[3] + nmpcWorkspace.QN1[148]*dOld[4] + nmpcWorkspace.QN1[149]*dOld[5] + nmpcWorkspace.QN1[150]*dOld[6] + nmpcWorkspace.QN1[151]*dOld[7] + nmpcWorkspace.QN1[152]*dOld[8] + nmpcWorkspace.QN1[153]*dOld[9] + nmpcWorkspace.QN1[154]*dOld[10] + nmpcWorkspace.QN1[155]*dOld[11] + nmpcWorkspace.QN1[156]*dOld[12] + nmpcWorkspace.QN1[157]*dOld[13] + nmpcWorkspace.QN1[158]*dOld[14] + nmpcWorkspace.QN1[159]*dOld[15];
dNew[10] = + nmpcWorkspace.QN1[160]*dOld[0] + nmpcWorkspace.QN1[161]*dOld[1] + nmpcWorkspace.QN1[162]*dOld[2] + nmpcWorkspace.QN1[163]*dOld[3] + nmpcWorkspace.QN1[164]*dOld[4] + nmpcWorkspace.QN1[165]*dOld[5] + nmpcWorkspace.QN1[166]*dOld[6] + nmpcWorkspace.QN1[167]*dOld[7] + nmpcWorkspace.QN1[168]*dOld[8] + nmpcWorkspace.QN1[169]*dOld[9] + nmpcWorkspace.QN1[170]*dOld[10] + nmpcWorkspace.QN1[171]*dOld[11] + nmpcWorkspace.QN1[172]*dOld[12] + nmpcWorkspace.QN1[173]*dOld[13] + nmpcWorkspace.QN1[174]*dOld[14] + nmpcWorkspace.QN1[175]*dOld[15];
dNew[11] = + nmpcWorkspace.QN1[176]*dOld[0] + nmpcWorkspace.QN1[177]*dOld[1] + nmpcWorkspace.QN1[178]*dOld[2] + nmpcWorkspace.QN1[179]*dOld[3] + nmpcWorkspace.QN1[180]*dOld[4] + nmpcWorkspace.QN1[181]*dOld[5] + nmpcWorkspace.QN1[182]*dOld[6] + nmpcWorkspace.QN1[183]*dOld[7] + nmpcWorkspace.QN1[184]*dOld[8] + nmpcWorkspace.QN1[185]*dOld[9] + nmpcWorkspace.QN1[186]*dOld[10] + nmpcWorkspace.QN1[187]*dOld[11] + nmpcWorkspace.QN1[188]*dOld[12] + nmpcWorkspace.QN1[189]*dOld[13] + nmpcWorkspace.QN1[190]*dOld[14] + nmpcWorkspace.QN1[191]*dOld[15];
dNew[12] = + nmpcWorkspace.QN1[192]*dOld[0] + nmpcWorkspace.QN1[193]*dOld[1] + nmpcWorkspace.QN1[194]*dOld[2] + nmpcWorkspace.QN1[195]*dOld[3] + nmpcWorkspace.QN1[196]*dOld[4] + nmpcWorkspace.QN1[197]*dOld[5] + nmpcWorkspace.QN1[198]*dOld[6] + nmpcWorkspace.QN1[199]*dOld[7] + nmpcWorkspace.QN1[200]*dOld[8] + nmpcWorkspace.QN1[201]*dOld[9] + nmpcWorkspace.QN1[202]*dOld[10] + nmpcWorkspace.QN1[203]*dOld[11] + nmpcWorkspace.QN1[204]*dOld[12] + nmpcWorkspace.QN1[205]*dOld[13] + nmpcWorkspace.QN1[206]*dOld[14] + nmpcWorkspace.QN1[207]*dOld[15];
dNew[13] = + nmpcWorkspace.QN1[208]*dOld[0] + nmpcWorkspace.QN1[209]*dOld[1] + nmpcWorkspace.QN1[210]*dOld[2] + nmpcWorkspace.QN1[211]*dOld[3] + nmpcWorkspace.QN1[212]*dOld[4] + nmpcWorkspace.QN1[213]*dOld[5] + nmpcWorkspace.QN1[214]*dOld[6] + nmpcWorkspace.QN1[215]*dOld[7] + nmpcWorkspace.QN1[216]*dOld[8] + nmpcWorkspace.QN1[217]*dOld[9] + nmpcWorkspace.QN1[218]*dOld[10] + nmpcWorkspace.QN1[219]*dOld[11] + nmpcWorkspace.QN1[220]*dOld[12] + nmpcWorkspace.QN1[221]*dOld[13] + nmpcWorkspace.QN1[222]*dOld[14] + nmpcWorkspace.QN1[223]*dOld[15];
dNew[14] = + nmpcWorkspace.QN1[224]*dOld[0] + nmpcWorkspace.QN1[225]*dOld[1] + nmpcWorkspace.QN1[226]*dOld[2] + nmpcWorkspace.QN1[227]*dOld[3] + nmpcWorkspace.QN1[228]*dOld[4] + nmpcWorkspace.QN1[229]*dOld[5] + nmpcWorkspace.QN1[230]*dOld[6] + nmpcWorkspace.QN1[231]*dOld[7] + nmpcWorkspace.QN1[232]*dOld[8] + nmpcWorkspace.QN1[233]*dOld[9] + nmpcWorkspace.QN1[234]*dOld[10] + nmpcWorkspace.QN1[235]*dOld[11] + nmpcWorkspace.QN1[236]*dOld[12] + nmpcWorkspace.QN1[237]*dOld[13] + nmpcWorkspace.QN1[238]*dOld[14] + nmpcWorkspace.QN1[239]*dOld[15];
dNew[15] = + nmpcWorkspace.QN1[240]*dOld[0] + nmpcWorkspace.QN1[241]*dOld[1] + nmpcWorkspace.QN1[242]*dOld[2] + nmpcWorkspace.QN1[243]*dOld[3] + nmpcWorkspace.QN1[244]*dOld[4] + nmpcWorkspace.QN1[245]*dOld[5] + nmpcWorkspace.QN1[246]*dOld[6] + nmpcWorkspace.QN1[247]*dOld[7] + nmpcWorkspace.QN1[248]*dOld[8] + nmpcWorkspace.QN1[249]*dOld[9] + nmpcWorkspace.QN1[250]*dOld[10] + nmpcWorkspace.QN1[251]*dOld[11] + nmpcWorkspace.QN1[252]*dOld[12] + nmpcWorkspace.QN1[253]*dOld[13] + nmpcWorkspace.QN1[254]*dOld[14] + nmpcWorkspace.QN1[255]*dOld[15];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13] + R2[14]*Dy1[14] + R2[15]*Dy1[15] + R2[16]*Dy1[16];
RDy1[1] = + R2[17]*Dy1[0] + R2[18]*Dy1[1] + R2[19]*Dy1[2] + R2[20]*Dy1[3] + R2[21]*Dy1[4] + R2[22]*Dy1[5] + R2[23]*Dy1[6] + R2[24]*Dy1[7] + R2[25]*Dy1[8] + R2[26]*Dy1[9] + R2[27]*Dy1[10] + R2[28]*Dy1[11] + R2[29]*Dy1[12] + R2[30]*Dy1[13] + R2[31]*Dy1[14] + R2[32]*Dy1[15] + R2[33]*Dy1[16];
RDy1[2] = + R2[34]*Dy1[0] + R2[35]*Dy1[1] + R2[36]*Dy1[2] + R2[37]*Dy1[3] + R2[38]*Dy1[4] + R2[39]*Dy1[5] + R2[40]*Dy1[6] + R2[41]*Dy1[7] + R2[42]*Dy1[8] + R2[43]*Dy1[9] + R2[44]*Dy1[10] + R2[45]*Dy1[11] + R2[46]*Dy1[12] + R2[47]*Dy1[13] + R2[48]*Dy1[14] + R2[49]*Dy1[15] + R2[50]*Dy1[16];
RDy1[3] = + R2[51]*Dy1[0] + R2[52]*Dy1[1] + R2[53]*Dy1[2] + R2[54]*Dy1[3] + R2[55]*Dy1[4] + R2[56]*Dy1[5] + R2[57]*Dy1[6] + R2[58]*Dy1[7] + R2[59]*Dy1[8] + R2[60]*Dy1[9] + R2[61]*Dy1[10] + R2[62]*Dy1[11] + R2[63]*Dy1[12] + R2[64]*Dy1[13] + R2[65]*Dy1[14] + R2[66]*Dy1[15] + R2[67]*Dy1[16];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13] + Q2[14]*Dy1[14] + Q2[15]*Dy1[15] + Q2[16]*Dy1[16];
QDy1[1] = + Q2[17]*Dy1[0] + Q2[18]*Dy1[1] + Q2[19]*Dy1[2] + Q2[20]*Dy1[3] + Q2[21]*Dy1[4] + Q2[22]*Dy1[5] + Q2[23]*Dy1[6] + Q2[24]*Dy1[7] + Q2[25]*Dy1[8] + Q2[26]*Dy1[9] + Q2[27]*Dy1[10] + Q2[28]*Dy1[11] + Q2[29]*Dy1[12] + Q2[30]*Dy1[13] + Q2[31]*Dy1[14] + Q2[32]*Dy1[15] + Q2[33]*Dy1[16];
QDy1[2] = + Q2[34]*Dy1[0] + Q2[35]*Dy1[1] + Q2[36]*Dy1[2] + Q2[37]*Dy1[3] + Q2[38]*Dy1[4] + Q2[39]*Dy1[5] + Q2[40]*Dy1[6] + Q2[41]*Dy1[7] + Q2[42]*Dy1[8] + Q2[43]*Dy1[9] + Q2[44]*Dy1[10] + Q2[45]*Dy1[11] + Q2[46]*Dy1[12] + Q2[47]*Dy1[13] + Q2[48]*Dy1[14] + Q2[49]*Dy1[15] + Q2[50]*Dy1[16];
QDy1[3] = + Q2[51]*Dy1[0] + Q2[52]*Dy1[1] + Q2[53]*Dy1[2] + Q2[54]*Dy1[3] + Q2[55]*Dy1[4] + Q2[56]*Dy1[5] + Q2[57]*Dy1[6] + Q2[58]*Dy1[7] + Q2[59]*Dy1[8] + Q2[60]*Dy1[9] + Q2[61]*Dy1[10] + Q2[62]*Dy1[11] + Q2[63]*Dy1[12] + Q2[64]*Dy1[13] + Q2[65]*Dy1[14] + Q2[66]*Dy1[15] + Q2[67]*Dy1[16];
QDy1[4] = + Q2[68]*Dy1[0] + Q2[69]*Dy1[1] + Q2[70]*Dy1[2] + Q2[71]*Dy1[3] + Q2[72]*Dy1[4] + Q2[73]*Dy1[5] + Q2[74]*Dy1[6] + Q2[75]*Dy1[7] + Q2[76]*Dy1[8] + Q2[77]*Dy1[9] + Q2[78]*Dy1[10] + Q2[79]*Dy1[11] + Q2[80]*Dy1[12] + Q2[81]*Dy1[13] + Q2[82]*Dy1[14] + Q2[83]*Dy1[15] + Q2[84]*Dy1[16];
QDy1[5] = + Q2[85]*Dy1[0] + Q2[86]*Dy1[1] + Q2[87]*Dy1[2] + Q2[88]*Dy1[3] + Q2[89]*Dy1[4] + Q2[90]*Dy1[5] + Q2[91]*Dy1[6] + Q2[92]*Dy1[7] + Q2[93]*Dy1[8] + Q2[94]*Dy1[9] + Q2[95]*Dy1[10] + Q2[96]*Dy1[11] + Q2[97]*Dy1[12] + Q2[98]*Dy1[13] + Q2[99]*Dy1[14] + Q2[100]*Dy1[15] + Q2[101]*Dy1[16];
QDy1[6] = + Q2[102]*Dy1[0] + Q2[103]*Dy1[1] + Q2[104]*Dy1[2] + Q2[105]*Dy1[3] + Q2[106]*Dy1[4] + Q2[107]*Dy1[5] + Q2[108]*Dy1[6] + Q2[109]*Dy1[7] + Q2[110]*Dy1[8] + Q2[111]*Dy1[9] + Q2[112]*Dy1[10] + Q2[113]*Dy1[11] + Q2[114]*Dy1[12] + Q2[115]*Dy1[13] + Q2[116]*Dy1[14] + Q2[117]*Dy1[15] + Q2[118]*Dy1[16];
QDy1[7] = + Q2[119]*Dy1[0] + Q2[120]*Dy1[1] + Q2[121]*Dy1[2] + Q2[122]*Dy1[3] + Q2[123]*Dy1[4] + Q2[124]*Dy1[5] + Q2[125]*Dy1[6] + Q2[126]*Dy1[7] + Q2[127]*Dy1[8] + Q2[128]*Dy1[9] + Q2[129]*Dy1[10] + Q2[130]*Dy1[11] + Q2[131]*Dy1[12] + Q2[132]*Dy1[13] + Q2[133]*Dy1[14] + Q2[134]*Dy1[15] + Q2[135]*Dy1[16];
QDy1[8] = + Q2[136]*Dy1[0] + Q2[137]*Dy1[1] + Q2[138]*Dy1[2] + Q2[139]*Dy1[3] + Q2[140]*Dy1[4] + Q2[141]*Dy1[5] + Q2[142]*Dy1[6] + Q2[143]*Dy1[7] + Q2[144]*Dy1[8] + Q2[145]*Dy1[9] + Q2[146]*Dy1[10] + Q2[147]*Dy1[11] + Q2[148]*Dy1[12] + Q2[149]*Dy1[13] + Q2[150]*Dy1[14] + Q2[151]*Dy1[15] + Q2[152]*Dy1[16];
QDy1[9] = + Q2[153]*Dy1[0] + Q2[154]*Dy1[1] + Q2[155]*Dy1[2] + Q2[156]*Dy1[3] + Q2[157]*Dy1[4] + Q2[158]*Dy1[5] + Q2[159]*Dy1[6] + Q2[160]*Dy1[7] + Q2[161]*Dy1[8] + Q2[162]*Dy1[9] + Q2[163]*Dy1[10] + Q2[164]*Dy1[11] + Q2[165]*Dy1[12] + Q2[166]*Dy1[13] + Q2[167]*Dy1[14] + Q2[168]*Dy1[15] + Q2[169]*Dy1[16];
QDy1[10] = + Q2[170]*Dy1[0] + Q2[171]*Dy1[1] + Q2[172]*Dy1[2] + Q2[173]*Dy1[3] + Q2[174]*Dy1[4] + Q2[175]*Dy1[5] + Q2[176]*Dy1[6] + Q2[177]*Dy1[7] + Q2[178]*Dy1[8] + Q2[179]*Dy1[9] + Q2[180]*Dy1[10] + Q2[181]*Dy1[11] + Q2[182]*Dy1[12] + Q2[183]*Dy1[13] + Q2[184]*Dy1[14] + Q2[185]*Dy1[15] + Q2[186]*Dy1[16];
QDy1[11] = + Q2[187]*Dy1[0] + Q2[188]*Dy1[1] + Q2[189]*Dy1[2] + Q2[190]*Dy1[3] + Q2[191]*Dy1[4] + Q2[192]*Dy1[5] + Q2[193]*Dy1[6] + Q2[194]*Dy1[7] + Q2[195]*Dy1[8] + Q2[196]*Dy1[9] + Q2[197]*Dy1[10] + Q2[198]*Dy1[11] + Q2[199]*Dy1[12] + Q2[200]*Dy1[13] + Q2[201]*Dy1[14] + Q2[202]*Dy1[15] + Q2[203]*Dy1[16];
QDy1[12] = + Q2[204]*Dy1[0] + Q2[205]*Dy1[1] + Q2[206]*Dy1[2] + Q2[207]*Dy1[3] + Q2[208]*Dy1[4] + Q2[209]*Dy1[5] + Q2[210]*Dy1[6] + Q2[211]*Dy1[7] + Q2[212]*Dy1[8] + Q2[213]*Dy1[9] + Q2[214]*Dy1[10] + Q2[215]*Dy1[11] + Q2[216]*Dy1[12] + Q2[217]*Dy1[13] + Q2[218]*Dy1[14] + Q2[219]*Dy1[15] + Q2[220]*Dy1[16];
QDy1[13] = + Q2[221]*Dy1[0] + Q2[222]*Dy1[1] + Q2[223]*Dy1[2] + Q2[224]*Dy1[3] + Q2[225]*Dy1[4] + Q2[226]*Dy1[5] + Q2[227]*Dy1[6] + Q2[228]*Dy1[7] + Q2[229]*Dy1[8] + Q2[230]*Dy1[9] + Q2[231]*Dy1[10] + Q2[232]*Dy1[11] + Q2[233]*Dy1[12] + Q2[234]*Dy1[13] + Q2[235]*Dy1[14] + Q2[236]*Dy1[15] + Q2[237]*Dy1[16];
QDy1[14] = + Q2[238]*Dy1[0] + Q2[239]*Dy1[1] + Q2[240]*Dy1[2] + Q2[241]*Dy1[3] + Q2[242]*Dy1[4] + Q2[243]*Dy1[5] + Q2[244]*Dy1[6] + Q2[245]*Dy1[7] + Q2[246]*Dy1[8] + Q2[247]*Dy1[9] + Q2[248]*Dy1[10] + Q2[249]*Dy1[11] + Q2[250]*Dy1[12] + Q2[251]*Dy1[13] + Q2[252]*Dy1[14] + Q2[253]*Dy1[15] + Q2[254]*Dy1[16];
QDy1[15] = + Q2[255]*Dy1[0] + Q2[256]*Dy1[1] + Q2[257]*Dy1[2] + Q2[258]*Dy1[3] + Q2[259]*Dy1[4] + Q2[260]*Dy1[5] + Q2[261]*Dy1[6] + Q2[262]*Dy1[7] + Q2[263]*Dy1[8] + Q2[264]*Dy1[9] + Q2[265]*Dy1[10] + Q2[266]*Dy1[11] + Q2[267]*Dy1[12] + Q2[268]*Dy1[13] + Q2[269]*Dy1[14] + Q2[270]*Dy1[15] + Q2[271]*Dy1[16];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7] + E1[32]*QDy1[8] + E1[36]*QDy1[9] + E1[40]*QDy1[10] + E1[44]*QDy1[11] + E1[48]*QDy1[12] + E1[52]*QDy1[13] + E1[56]*QDy1[14] + E1[60]*QDy1[15];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7] + E1[33]*QDy1[8] + E1[37]*QDy1[9] + E1[41]*QDy1[10] + E1[45]*QDy1[11] + E1[49]*QDy1[12] + E1[53]*QDy1[13] + E1[57]*QDy1[14] + E1[61]*QDy1[15];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7] + E1[34]*QDy1[8] + E1[38]*QDy1[9] + E1[42]*QDy1[10] + E1[46]*QDy1[11] + E1[50]*QDy1[12] + E1[54]*QDy1[13] + E1[58]*QDy1[14] + E1[62]*QDy1[15];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7] + E1[35]*QDy1[8] + E1[39]*QDy1[9] + E1[43]*QDy1[10] + E1[47]*QDy1[11] + E1[51]*QDy1[12] + E1[55]*QDy1[13] + E1[59]*QDy1[14] + E1[63]*QDy1[15];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[16] + E1[8]*Gx1[32] + E1[12]*Gx1[48] + E1[16]*Gx1[64] + E1[20]*Gx1[80] + E1[24]*Gx1[96] + E1[28]*Gx1[112] + E1[32]*Gx1[128] + E1[36]*Gx1[144] + E1[40]*Gx1[160] + E1[44]*Gx1[176] + E1[48]*Gx1[192] + E1[52]*Gx1[208] + E1[56]*Gx1[224] + E1[60]*Gx1[240];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[17] + E1[8]*Gx1[33] + E1[12]*Gx1[49] + E1[16]*Gx1[65] + E1[20]*Gx1[81] + E1[24]*Gx1[97] + E1[28]*Gx1[113] + E1[32]*Gx1[129] + E1[36]*Gx1[145] + E1[40]*Gx1[161] + E1[44]*Gx1[177] + E1[48]*Gx1[193] + E1[52]*Gx1[209] + E1[56]*Gx1[225] + E1[60]*Gx1[241];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[18] + E1[8]*Gx1[34] + E1[12]*Gx1[50] + E1[16]*Gx1[66] + E1[20]*Gx1[82] + E1[24]*Gx1[98] + E1[28]*Gx1[114] + E1[32]*Gx1[130] + E1[36]*Gx1[146] + E1[40]*Gx1[162] + E1[44]*Gx1[178] + E1[48]*Gx1[194] + E1[52]*Gx1[210] + E1[56]*Gx1[226] + E1[60]*Gx1[242];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[19] + E1[8]*Gx1[35] + E1[12]*Gx1[51] + E1[16]*Gx1[67] + E1[20]*Gx1[83] + E1[24]*Gx1[99] + E1[28]*Gx1[115] + E1[32]*Gx1[131] + E1[36]*Gx1[147] + E1[40]*Gx1[163] + E1[44]*Gx1[179] + E1[48]*Gx1[195] + E1[52]*Gx1[211] + E1[56]*Gx1[227] + E1[60]*Gx1[243];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[20] + E1[8]*Gx1[36] + E1[12]*Gx1[52] + E1[16]*Gx1[68] + E1[20]*Gx1[84] + E1[24]*Gx1[100] + E1[28]*Gx1[116] + E1[32]*Gx1[132] + E1[36]*Gx1[148] + E1[40]*Gx1[164] + E1[44]*Gx1[180] + E1[48]*Gx1[196] + E1[52]*Gx1[212] + E1[56]*Gx1[228] + E1[60]*Gx1[244];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[21] + E1[8]*Gx1[37] + E1[12]*Gx1[53] + E1[16]*Gx1[69] + E1[20]*Gx1[85] + E1[24]*Gx1[101] + E1[28]*Gx1[117] + E1[32]*Gx1[133] + E1[36]*Gx1[149] + E1[40]*Gx1[165] + E1[44]*Gx1[181] + E1[48]*Gx1[197] + E1[52]*Gx1[213] + E1[56]*Gx1[229] + E1[60]*Gx1[245];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[22] + E1[8]*Gx1[38] + E1[12]*Gx1[54] + E1[16]*Gx1[70] + E1[20]*Gx1[86] + E1[24]*Gx1[102] + E1[28]*Gx1[118] + E1[32]*Gx1[134] + E1[36]*Gx1[150] + E1[40]*Gx1[166] + E1[44]*Gx1[182] + E1[48]*Gx1[198] + E1[52]*Gx1[214] + E1[56]*Gx1[230] + E1[60]*Gx1[246];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[23] + E1[8]*Gx1[39] + E1[12]*Gx1[55] + E1[16]*Gx1[71] + E1[20]*Gx1[87] + E1[24]*Gx1[103] + E1[28]*Gx1[119] + E1[32]*Gx1[135] + E1[36]*Gx1[151] + E1[40]*Gx1[167] + E1[44]*Gx1[183] + E1[48]*Gx1[199] + E1[52]*Gx1[215] + E1[56]*Gx1[231] + E1[60]*Gx1[247];
H101[8] += + E1[0]*Gx1[8] + E1[4]*Gx1[24] + E1[8]*Gx1[40] + E1[12]*Gx1[56] + E1[16]*Gx1[72] + E1[20]*Gx1[88] + E1[24]*Gx1[104] + E1[28]*Gx1[120] + E1[32]*Gx1[136] + E1[36]*Gx1[152] + E1[40]*Gx1[168] + E1[44]*Gx1[184] + E1[48]*Gx1[200] + E1[52]*Gx1[216] + E1[56]*Gx1[232] + E1[60]*Gx1[248];
H101[9] += + E1[0]*Gx1[9] + E1[4]*Gx1[25] + E1[8]*Gx1[41] + E1[12]*Gx1[57] + E1[16]*Gx1[73] + E1[20]*Gx1[89] + E1[24]*Gx1[105] + E1[28]*Gx1[121] + E1[32]*Gx1[137] + E1[36]*Gx1[153] + E1[40]*Gx1[169] + E1[44]*Gx1[185] + E1[48]*Gx1[201] + E1[52]*Gx1[217] + E1[56]*Gx1[233] + E1[60]*Gx1[249];
H101[10] += + E1[0]*Gx1[10] + E1[4]*Gx1[26] + E1[8]*Gx1[42] + E1[12]*Gx1[58] + E1[16]*Gx1[74] + E1[20]*Gx1[90] + E1[24]*Gx1[106] + E1[28]*Gx1[122] + E1[32]*Gx1[138] + E1[36]*Gx1[154] + E1[40]*Gx1[170] + E1[44]*Gx1[186] + E1[48]*Gx1[202] + E1[52]*Gx1[218] + E1[56]*Gx1[234] + E1[60]*Gx1[250];
H101[11] += + E1[0]*Gx1[11] + E1[4]*Gx1[27] + E1[8]*Gx1[43] + E1[12]*Gx1[59] + E1[16]*Gx1[75] + E1[20]*Gx1[91] + E1[24]*Gx1[107] + E1[28]*Gx1[123] + E1[32]*Gx1[139] + E1[36]*Gx1[155] + E1[40]*Gx1[171] + E1[44]*Gx1[187] + E1[48]*Gx1[203] + E1[52]*Gx1[219] + E1[56]*Gx1[235] + E1[60]*Gx1[251];
H101[12] += + E1[0]*Gx1[12] + E1[4]*Gx1[28] + E1[8]*Gx1[44] + E1[12]*Gx1[60] + E1[16]*Gx1[76] + E1[20]*Gx1[92] + E1[24]*Gx1[108] + E1[28]*Gx1[124] + E1[32]*Gx1[140] + E1[36]*Gx1[156] + E1[40]*Gx1[172] + E1[44]*Gx1[188] + E1[48]*Gx1[204] + E1[52]*Gx1[220] + E1[56]*Gx1[236] + E1[60]*Gx1[252];
H101[13] += + E1[0]*Gx1[13] + E1[4]*Gx1[29] + E1[8]*Gx1[45] + E1[12]*Gx1[61] + E1[16]*Gx1[77] + E1[20]*Gx1[93] + E1[24]*Gx1[109] + E1[28]*Gx1[125] + E1[32]*Gx1[141] + E1[36]*Gx1[157] + E1[40]*Gx1[173] + E1[44]*Gx1[189] + E1[48]*Gx1[205] + E1[52]*Gx1[221] + E1[56]*Gx1[237] + E1[60]*Gx1[253];
H101[14] += + E1[0]*Gx1[14] + E1[4]*Gx1[30] + E1[8]*Gx1[46] + E1[12]*Gx1[62] + E1[16]*Gx1[78] + E1[20]*Gx1[94] + E1[24]*Gx1[110] + E1[28]*Gx1[126] + E1[32]*Gx1[142] + E1[36]*Gx1[158] + E1[40]*Gx1[174] + E1[44]*Gx1[190] + E1[48]*Gx1[206] + E1[52]*Gx1[222] + E1[56]*Gx1[238] + E1[60]*Gx1[254];
H101[15] += + E1[0]*Gx1[15] + E1[4]*Gx1[31] + E1[8]*Gx1[47] + E1[12]*Gx1[63] + E1[16]*Gx1[79] + E1[20]*Gx1[95] + E1[24]*Gx1[111] + E1[28]*Gx1[127] + E1[32]*Gx1[143] + E1[36]*Gx1[159] + E1[40]*Gx1[175] + E1[44]*Gx1[191] + E1[48]*Gx1[207] + E1[52]*Gx1[223] + E1[56]*Gx1[239] + E1[60]*Gx1[255];
H101[16] += + E1[1]*Gx1[0] + E1[5]*Gx1[16] + E1[9]*Gx1[32] + E1[13]*Gx1[48] + E1[17]*Gx1[64] + E1[21]*Gx1[80] + E1[25]*Gx1[96] + E1[29]*Gx1[112] + E1[33]*Gx1[128] + E1[37]*Gx1[144] + E1[41]*Gx1[160] + E1[45]*Gx1[176] + E1[49]*Gx1[192] + E1[53]*Gx1[208] + E1[57]*Gx1[224] + E1[61]*Gx1[240];
H101[17] += + E1[1]*Gx1[1] + E1[5]*Gx1[17] + E1[9]*Gx1[33] + E1[13]*Gx1[49] + E1[17]*Gx1[65] + E1[21]*Gx1[81] + E1[25]*Gx1[97] + E1[29]*Gx1[113] + E1[33]*Gx1[129] + E1[37]*Gx1[145] + E1[41]*Gx1[161] + E1[45]*Gx1[177] + E1[49]*Gx1[193] + E1[53]*Gx1[209] + E1[57]*Gx1[225] + E1[61]*Gx1[241];
H101[18] += + E1[1]*Gx1[2] + E1[5]*Gx1[18] + E1[9]*Gx1[34] + E1[13]*Gx1[50] + E1[17]*Gx1[66] + E1[21]*Gx1[82] + E1[25]*Gx1[98] + E1[29]*Gx1[114] + E1[33]*Gx1[130] + E1[37]*Gx1[146] + E1[41]*Gx1[162] + E1[45]*Gx1[178] + E1[49]*Gx1[194] + E1[53]*Gx1[210] + E1[57]*Gx1[226] + E1[61]*Gx1[242];
H101[19] += + E1[1]*Gx1[3] + E1[5]*Gx1[19] + E1[9]*Gx1[35] + E1[13]*Gx1[51] + E1[17]*Gx1[67] + E1[21]*Gx1[83] + E1[25]*Gx1[99] + E1[29]*Gx1[115] + E1[33]*Gx1[131] + E1[37]*Gx1[147] + E1[41]*Gx1[163] + E1[45]*Gx1[179] + E1[49]*Gx1[195] + E1[53]*Gx1[211] + E1[57]*Gx1[227] + E1[61]*Gx1[243];
H101[20] += + E1[1]*Gx1[4] + E1[5]*Gx1[20] + E1[9]*Gx1[36] + E1[13]*Gx1[52] + E1[17]*Gx1[68] + E1[21]*Gx1[84] + E1[25]*Gx1[100] + E1[29]*Gx1[116] + E1[33]*Gx1[132] + E1[37]*Gx1[148] + E1[41]*Gx1[164] + E1[45]*Gx1[180] + E1[49]*Gx1[196] + E1[53]*Gx1[212] + E1[57]*Gx1[228] + E1[61]*Gx1[244];
H101[21] += + E1[1]*Gx1[5] + E1[5]*Gx1[21] + E1[9]*Gx1[37] + E1[13]*Gx1[53] + E1[17]*Gx1[69] + E1[21]*Gx1[85] + E1[25]*Gx1[101] + E1[29]*Gx1[117] + E1[33]*Gx1[133] + E1[37]*Gx1[149] + E1[41]*Gx1[165] + E1[45]*Gx1[181] + E1[49]*Gx1[197] + E1[53]*Gx1[213] + E1[57]*Gx1[229] + E1[61]*Gx1[245];
H101[22] += + E1[1]*Gx1[6] + E1[5]*Gx1[22] + E1[9]*Gx1[38] + E1[13]*Gx1[54] + E1[17]*Gx1[70] + E1[21]*Gx1[86] + E1[25]*Gx1[102] + E1[29]*Gx1[118] + E1[33]*Gx1[134] + E1[37]*Gx1[150] + E1[41]*Gx1[166] + E1[45]*Gx1[182] + E1[49]*Gx1[198] + E1[53]*Gx1[214] + E1[57]*Gx1[230] + E1[61]*Gx1[246];
H101[23] += + E1[1]*Gx1[7] + E1[5]*Gx1[23] + E1[9]*Gx1[39] + E1[13]*Gx1[55] + E1[17]*Gx1[71] + E1[21]*Gx1[87] + E1[25]*Gx1[103] + E1[29]*Gx1[119] + E1[33]*Gx1[135] + E1[37]*Gx1[151] + E1[41]*Gx1[167] + E1[45]*Gx1[183] + E1[49]*Gx1[199] + E1[53]*Gx1[215] + E1[57]*Gx1[231] + E1[61]*Gx1[247];
H101[24] += + E1[1]*Gx1[8] + E1[5]*Gx1[24] + E1[9]*Gx1[40] + E1[13]*Gx1[56] + E1[17]*Gx1[72] + E1[21]*Gx1[88] + E1[25]*Gx1[104] + E1[29]*Gx1[120] + E1[33]*Gx1[136] + E1[37]*Gx1[152] + E1[41]*Gx1[168] + E1[45]*Gx1[184] + E1[49]*Gx1[200] + E1[53]*Gx1[216] + E1[57]*Gx1[232] + E1[61]*Gx1[248];
H101[25] += + E1[1]*Gx1[9] + E1[5]*Gx1[25] + E1[9]*Gx1[41] + E1[13]*Gx1[57] + E1[17]*Gx1[73] + E1[21]*Gx1[89] + E1[25]*Gx1[105] + E1[29]*Gx1[121] + E1[33]*Gx1[137] + E1[37]*Gx1[153] + E1[41]*Gx1[169] + E1[45]*Gx1[185] + E1[49]*Gx1[201] + E1[53]*Gx1[217] + E1[57]*Gx1[233] + E1[61]*Gx1[249];
H101[26] += + E1[1]*Gx1[10] + E1[5]*Gx1[26] + E1[9]*Gx1[42] + E1[13]*Gx1[58] + E1[17]*Gx1[74] + E1[21]*Gx1[90] + E1[25]*Gx1[106] + E1[29]*Gx1[122] + E1[33]*Gx1[138] + E1[37]*Gx1[154] + E1[41]*Gx1[170] + E1[45]*Gx1[186] + E1[49]*Gx1[202] + E1[53]*Gx1[218] + E1[57]*Gx1[234] + E1[61]*Gx1[250];
H101[27] += + E1[1]*Gx1[11] + E1[5]*Gx1[27] + E1[9]*Gx1[43] + E1[13]*Gx1[59] + E1[17]*Gx1[75] + E1[21]*Gx1[91] + E1[25]*Gx1[107] + E1[29]*Gx1[123] + E1[33]*Gx1[139] + E1[37]*Gx1[155] + E1[41]*Gx1[171] + E1[45]*Gx1[187] + E1[49]*Gx1[203] + E1[53]*Gx1[219] + E1[57]*Gx1[235] + E1[61]*Gx1[251];
H101[28] += + E1[1]*Gx1[12] + E1[5]*Gx1[28] + E1[9]*Gx1[44] + E1[13]*Gx1[60] + E1[17]*Gx1[76] + E1[21]*Gx1[92] + E1[25]*Gx1[108] + E1[29]*Gx1[124] + E1[33]*Gx1[140] + E1[37]*Gx1[156] + E1[41]*Gx1[172] + E1[45]*Gx1[188] + E1[49]*Gx1[204] + E1[53]*Gx1[220] + E1[57]*Gx1[236] + E1[61]*Gx1[252];
H101[29] += + E1[1]*Gx1[13] + E1[5]*Gx1[29] + E1[9]*Gx1[45] + E1[13]*Gx1[61] + E1[17]*Gx1[77] + E1[21]*Gx1[93] + E1[25]*Gx1[109] + E1[29]*Gx1[125] + E1[33]*Gx1[141] + E1[37]*Gx1[157] + E1[41]*Gx1[173] + E1[45]*Gx1[189] + E1[49]*Gx1[205] + E1[53]*Gx1[221] + E1[57]*Gx1[237] + E1[61]*Gx1[253];
H101[30] += + E1[1]*Gx1[14] + E1[5]*Gx1[30] + E1[9]*Gx1[46] + E1[13]*Gx1[62] + E1[17]*Gx1[78] + E1[21]*Gx1[94] + E1[25]*Gx1[110] + E1[29]*Gx1[126] + E1[33]*Gx1[142] + E1[37]*Gx1[158] + E1[41]*Gx1[174] + E1[45]*Gx1[190] + E1[49]*Gx1[206] + E1[53]*Gx1[222] + E1[57]*Gx1[238] + E1[61]*Gx1[254];
H101[31] += + E1[1]*Gx1[15] + E1[5]*Gx1[31] + E1[9]*Gx1[47] + E1[13]*Gx1[63] + E1[17]*Gx1[79] + E1[21]*Gx1[95] + E1[25]*Gx1[111] + E1[29]*Gx1[127] + E1[33]*Gx1[143] + E1[37]*Gx1[159] + E1[41]*Gx1[175] + E1[45]*Gx1[191] + E1[49]*Gx1[207] + E1[53]*Gx1[223] + E1[57]*Gx1[239] + E1[61]*Gx1[255];
H101[32] += + E1[2]*Gx1[0] + E1[6]*Gx1[16] + E1[10]*Gx1[32] + E1[14]*Gx1[48] + E1[18]*Gx1[64] + E1[22]*Gx1[80] + E1[26]*Gx1[96] + E1[30]*Gx1[112] + E1[34]*Gx1[128] + E1[38]*Gx1[144] + E1[42]*Gx1[160] + E1[46]*Gx1[176] + E1[50]*Gx1[192] + E1[54]*Gx1[208] + E1[58]*Gx1[224] + E1[62]*Gx1[240];
H101[33] += + E1[2]*Gx1[1] + E1[6]*Gx1[17] + E1[10]*Gx1[33] + E1[14]*Gx1[49] + E1[18]*Gx1[65] + E1[22]*Gx1[81] + E1[26]*Gx1[97] + E1[30]*Gx1[113] + E1[34]*Gx1[129] + E1[38]*Gx1[145] + E1[42]*Gx1[161] + E1[46]*Gx1[177] + E1[50]*Gx1[193] + E1[54]*Gx1[209] + E1[58]*Gx1[225] + E1[62]*Gx1[241];
H101[34] += + E1[2]*Gx1[2] + E1[6]*Gx1[18] + E1[10]*Gx1[34] + E1[14]*Gx1[50] + E1[18]*Gx1[66] + E1[22]*Gx1[82] + E1[26]*Gx1[98] + E1[30]*Gx1[114] + E1[34]*Gx1[130] + E1[38]*Gx1[146] + E1[42]*Gx1[162] + E1[46]*Gx1[178] + E1[50]*Gx1[194] + E1[54]*Gx1[210] + E1[58]*Gx1[226] + E1[62]*Gx1[242];
H101[35] += + E1[2]*Gx1[3] + E1[6]*Gx1[19] + E1[10]*Gx1[35] + E1[14]*Gx1[51] + E1[18]*Gx1[67] + E1[22]*Gx1[83] + E1[26]*Gx1[99] + E1[30]*Gx1[115] + E1[34]*Gx1[131] + E1[38]*Gx1[147] + E1[42]*Gx1[163] + E1[46]*Gx1[179] + E1[50]*Gx1[195] + E1[54]*Gx1[211] + E1[58]*Gx1[227] + E1[62]*Gx1[243];
H101[36] += + E1[2]*Gx1[4] + E1[6]*Gx1[20] + E1[10]*Gx1[36] + E1[14]*Gx1[52] + E1[18]*Gx1[68] + E1[22]*Gx1[84] + E1[26]*Gx1[100] + E1[30]*Gx1[116] + E1[34]*Gx1[132] + E1[38]*Gx1[148] + E1[42]*Gx1[164] + E1[46]*Gx1[180] + E1[50]*Gx1[196] + E1[54]*Gx1[212] + E1[58]*Gx1[228] + E1[62]*Gx1[244];
H101[37] += + E1[2]*Gx1[5] + E1[6]*Gx1[21] + E1[10]*Gx1[37] + E1[14]*Gx1[53] + E1[18]*Gx1[69] + E1[22]*Gx1[85] + E1[26]*Gx1[101] + E1[30]*Gx1[117] + E1[34]*Gx1[133] + E1[38]*Gx1[149] + E1[42]*Gx1[165] + E1[46]*Gx1[181] + E1[50]*Gx1[197] + E1[54]*Gx1[213] + E1[58]*Gx1[229] + E1[62]*Gx1[245];
H101[38] += + E1[2]*Gx1[6] + E1[6]*Gx1[22] + E1[10]*Gx1[38] + E1[14]*Gx1[54] + E1[18]*Gx1[70] + E1[22]*Gx1[86] + E1[26]*Gx1[102] + E1[30]*Gx1[118] + E1[34]*Gx1[134] + E1[38]*Gx1[150] + E1[42]*Gx1[166] + E1[46]*Gx1[182] + E1[50]*Gx1[198] + E1[54]*Gx1[214] + E1[58]*Gx1[230] + E1[62]*Gx1[246];
H101[39] += + E1[2]*Gx1[7] + E1[6]*Gx1[23] + E1[10]*Gx1[39] + E1[14]*Gx1[55] + E1[18]*Gx1[71] + E1[22]*Gx1[87] + E1[26]*Gx1[103] + E1[30]*Gx1[119] + E1[34]*Gx1[135] + E1[38]*Gx1[151] + E1[42]*Gx1[167] + E1[46]*Gx1[183] + E1[50]*Gx1[199] + E1[54]*Gx1[215] + E1[58]*Gx1[231] + E1[62]*Gx1[247];
H101[40] += + E1[2]*Gx1[8] + E1[6]*Gx1[24] + E1[10]*Gx1[40] + E1[14]*Gx1[56] + E1[18]*Gx1[72] + E1[22]*Gx1[88] + E1[26]*Gx1[104] + E1[30]*Gx1[120] + E1[34]*Gx1[136] + E1[38]*Gx1[152] + E1[42]*Gx1[168] + E1[46]*Gx1[184] + E1[50]*Gx1[200] + E1[54]*Gx1[216] + E1[58]*Gx1[232] + E1[62]*Gx1[248];
H101[41] += + E1[2]*Gx1[9] + E1[6]*Gx1[25] + E1[10]*Gx1[41] + E1[14]*Gx1[57] + E1[18]*Gx1[73] + E1[22]*Gx1[89] + E1[26]*Gx1[105] + E1[30]*Gx1[121] + E1[34]*Gx1[137] + E1[38]*Gx1[153] + E1[42]*Gx1[169] + E1[46]*Gx1[185] + E1[50]*Gx1[201] + E1[54]*Gx1[217] + E1[58]*Gx1[233] + E1[62]*Gx1[249];
H101[42] += + E1[2]*Gx1[10] + E1[6]*Gx1[26] + E1[10]*Gx1[42] + E1[14]*Gx1[58] + E1[18]*Gx1[74] + E1[22]*Gx1[90] + E1[26]*Gx1[106] + E1[30]*Gx1[122] + E1[34]*Gx1[138] + E1[38]*Gx1[154] + E1[42]*Gx1[170] + E1[46]*Gx1[186] + E1[50]*Gx1[202] + E1[54]*Gx1[218] + E1[58]*Gx1[234] + E1[62]*Gx1[250];
H101[43] += + E1[2]*Gx1[11] + E1[6]*Gx1[27] + E1[10]*Gx1[43] + E1[14]*Gx1[59] + E1[18]*Gx1[75] + E1[22]*Gx1[91] + E1[26]*Gx1[107] + E1[30]*Gx1[123] + E1[34]*Gx1[139] + E1[38]*Gx1[155] + E1[42]*Gx1[171] + E1[46]*Gx1[187] + E1[50]*Gx1[203] + E1[54]*Gx1[219] + E1[58]*Gx1[235] + E1[62]*Gx1[251];
H101[44] += + E1[2]*Gx1[12] + E1[6]*Gx1[28] + E1[10]*Gx1[44] + E1[14]*Gx1[60] + E1[18]*Gx1[76] + E1[22]*Gx1[92] + E1[26]*Gx1[108] + E1[30]*Gx1[124] + E1[34]*Gx1[140] + E1[38]*Gx1[156] + E1[42]*Gx1[172] + E1[46]*Gx1[188] + E1[50]*Gx1[204] + E1[54]*Gx1[220] + E1[58]*Gx1[236] + E1[62]*Gx1[252];
H101[45] += + E1[2]*Gx1[13] + E1[6]*Gx1[29] + E1[10]*Gx1[45] + E1[14]*Gx1[61] + E1[18]*Gx1[77] + E1[22]*Gx1[93] + E1[26]*Gx1[109] + E1[30]*Gx1[125] + E1[34]*Gx1[141] + E1[38]*Gx1[157] + E1[42]*Gx1[173] + E1[46]*Gx1[189] + E1[50]*Gx1[205] + E1[54]*Gx1[221] + E1[58]*Gx1[237] + E1[62]*Gx1[253];
H101[46] += + E1[2]*Gx1[14] + E1[6]*Gx1[30] + E1[10]*Gx1[46] + E1[14]*Gx1[62] + E1[18]*Gx1[78] + E1[22]*Gx1[94] + E1[26]*Gx1[110] + E1[30]*Gx1[126] + E1[34]*Gx1[142] + E1[38]*Gx1[158] + E1[42]*Gx1[174] + E1[46]*Gx1[190] + E1[50]*Gx1[206] + E1[54]*Gx1[222] + E1[58]*Gx1[238] + E1[62]*Gx1[254];
H101[47] += + E1[2]*Gx1[15] + E1[6]*Gx1[31] + E1[10]*Gx1[47] + E1[14]*Gx1[63] + E1[18]*Gx1[79] + E1[22]*Gx1[95] + E1[26]*Gx1[111] + E1[30]*Gx1[127] + E1[34]*Gx1[143] + E1[38]*Gx1[159] + E1[42]*Gx1[175] + E1[46]*Gx1[191] + E1[50]*Gx1[207] + E1[54]*Gx1[223] + E1[58]*Gx1[239] + E1[62]*Gx1[255];
H101[48] += + E1[3]*Gx1[0] + E1[7]*Gx1[16] + E1[11]*Gx1[32] + E1[15]*Gx1[48] + E1[19]*Gx1[64] + E1[23]*Gx1[80] + E1[27]*Gx1[96] + E1[31]*Gx1[112] + E1[35]*Gx1[128] + E1[39]*Gx1[144] + E1[43]*Gx1[160] + E1[47]*Gx1[176] + E1[51]*Gx1[192] + E1[55]*Gx1[208] + E1[59]*Gx1[224] + E1[63]*Gx1[240];
H101[49] += + E1[3]*Gx1[1] + E1[7]*Gx1[17] + E1[11]*Gx1[33] + E1[15]*Gx1[49] + E1[19]*Gx1[65] + E1[23]*Gx1[81] + E1[27]*Gx1[97] + E1[31]*Gx1[113] + E1[35]*Gx1[129] + E1[39]*Gx1[145] + E1[43]*Gx1[161] + E1[47]*Gx1[177] + E1[51]*Gx1[193] + E1[55]*Gx1[209] + E1[59]*Gx1[225] + E1[63]*Gx1[241];
H101[50] += + E1[3]*Gx1[2] + E1[7]*Gx1[18] + E1[11]*Gx1[34] + E1[15]*Gx1[50] + E1[19]*Gx1[66] + E1[23]*Gx1[82] + E1[27]*Gx1[98] + E1[31]*Gx1[114] + E1[35]*Gx1[130] + E1[39]*Gx1[146] + E1[43]*Gx1[162] + E1[47]*Gx1[178] + E1[51]*Gx1[194] + E1[55]*Gx1[210] + E1[59]*Gx1[226] + E1[63]*Gx1[242];
H101[51] += + E1[3]*Gx1[3] + E1[7]*Gx1[19] + E1[11]*Gx1[35] + E1[15]*Gx1[51] + E1[19]*Gx1[67] + E1[23]*Gx1[83] + E1[27]*Gx1[99] + E1[31]*Gx1[115] + E1[35]*Gx1[131] + E1[39]*Gx1[147] + E1[43]*Gx1[163] + E1[47]*Gx1[179] + E1[51]*Gx1[195] + E1[55]*Gx1[211] + E1[59]*Gx1[227] + E1[63]*Gx1[243];
H101[52] += + E1[3]*Gx1[4] + E1[7]*Gx1[20] + E1[11]*Gx1[36] + E1[15]*Gx1[52] + E1[19]*Gx1[68] + E1[23]*Gx1[84] + E1[27]*Gx1[100] + E1[31]*Gx1[116] + E1[35]*Gx1[132] + E1[39]*Gx1[148] + E1[43]*Gx1[164] + E1[47]*Gx1[180] + E1[51]*Gx1[196] + E1[55]*Gx1[212] + E1[59]*Gx1[228] + E1[63]*Gx1[244];
H101[53] += + E1[3]*Gx1[5] + E1[7]*Gx1[21] + E1[11]*Gx1[37] + E1[15]*Gx1[53] + E1[19]*Gx1[69] + E1[23]*Gx1[85] + E1[27]*Gx1[101] + E1[31]*Gx1[117] + E1[35]*Gx1[133] + E1[39]*Gx1[149] + E1[43]*Gx1[165] + E1[47]*Gx1[181] + E1[51]*Gx1[197] + E1[55]*Gx1[213] + E1[59]*Gx1[229] + E1[63]*Gx1[245];
H101[54] += + E1[3]*Gx1[6] + E1[7]*Gx1[22] + E1[11]*Gx1[38] + E1[15]*Gx1[54] + E1[19]*Gx1[70] + E1[23]*Gx1[86] + E1[27]*Gx1[102] + E1[31]*Gx1[118] + E1[35]*Gx1[134] + E1[39]*Gx1[150] + E1[43]*Gx1[166] + E1[47]*Gx1[182] + E1[51]*Gx1[198] + E1[55]*Gx1[214] + E1[59]*Gx1[230] + E1[63]*Gx1[246];
H101[55] += + E1[3]*Gx1[7] + E1[7]*Gx1[23] + E1[11]*Gx1[39] + E1[15]*Gx1[55] + E1[19]*Gx1[71] + E1[23]*Gx1[87] + E1[27]*Gx1[103] + E1[31]*Gx1[119] + E1[35]*Gx1[135] + E1[39]*Gx1[151] + E1[43]*Gx1[167] + E1[47]*Gx1[183] + E1[51]*Gx1[199] + E1[55]*Gx1[215] + E1[59]*Gx1[231] + E1[63]*Gx1[247];
H101[56] += + E1[3]*Gx1[8] + E1[7]*Gx1[24] + E1[11]*Gx1[40] + E1[15]*Gx1[56] + E1[19]*Gx1[72] + E1[23]*Gx1[88] + E1[27]*Gx1[104] + E1[31]*Gx1[120] + E1[35]*Gx1[136] + E1[39]*Gx1[152] + E1[43]*Gx1[168] + E1[47]*Gx1[184] + E1[51]*Gx1[200] + E1[55]*Gx1[216] + E1[59]*Gx1[232] + E1[63]*Gx1[248];
H101[57] += + E1[3]*Gx1[9] + E1[7]*Gx1[25] + E1[11]*Gx1[41] + E1[15]*Gx1[57] + E1[19]*Gx1[73] + E1[23]*Gx1[89] + E1[27]*Gx1[105] + E1[31]*Gx1[121] + E1[35]*Gx1[137] + E1[39]*Gx1[153] + E1[43]*Gx1[169] + E1[47]*Gx1[185] + E1[51]*Gx1[201] + E1[55]*Gx1[217] + E1[59]*Gx1[233] + E1[63]*Gx1[249];
H101[58] += + E1[3]*Gx1[10] + E1[7]*Gx1[26] + E1[11]*Gx1[42] + E1[15]*Gx1[58] + E1[19]*Gx1[74] + E1[23]*Gx1[90] + E1[27]*Gx1[106] + E1[31]*Gx1[122] + E1[35]*Gx1[138] + E1[39]*Gx1[154] + E1[43]*Gx1[170] + E1[47]*Gx1[186] + E1[51]*Gx1[202] + E1[55]*Gx1[218] + E1[59]*Gx1[234] + E1[63]*Gx1[250];
H101[59] += + E1[3]*Gx1[11] + E1[7]*Gx1[27] + E1[11]*Gx1[43] + E1[15]*Gx1[59] + E1[19]*Gx1[75] + E1[23]*Gx1[91] + E1[27]*Gx1[107] + E1[31]*Gx1[123] + E1[35]*Gx1[139] + E1[39]*Gx1[155] + E1[43]*Gx1[171] + E1[47]*Gx1[187] + E1[51]*Gx1[203] + E1[55]*Gx1[219] + E1[59]*Gx1[235] + E1[63]*Gx1[251];
H101[60] += + E1[3]*Gx1[12] + E1[7]*Gx1[28] + E1[11]*Gx1[44] + E1[15]*Gx1[60] + E1[19]*Gx1[76] + E1[23]*Gx1[92] + E1[27]*Gx1[108] + E1[31]*Gx1[124] + E1[35]*Gx1[140] + E1[39]*Gx1[156] + E1[43]*Gx1[172] + E1[47]*Gx1[188] + E1[51]*Gx1[204] + E1[55]*Gx1[220] + E1[59]*Gx1[236] + E1[63]*Gx1[252];
H101[61] += + E1[3]*Gx1[13] + E1[7]*Gx1[29] + E1[11]*Gx1[45] + E1[15]*Gx1[61] + E1[19]*Gx1[77] + E1[23]*Gx1[93] + E1[27]*Gx1[109] + E1[31]*Gx1[125] + E1[35]*Gx1[141] + E1[39]*Gx1[157] + E1[43]*Gx1[173] + E1[47]*Gx1[189] + E1[51]*Gx1[205] + E1[55]*Gx1[221] + E1[59]*Gx1[237] + E1[63]*Gx1[253];
H101[62] += + E1[3]*Gx1[14] + E1[7]*Gx1[30] + E1[11]*Gx1[46] + E1[15]*Gx1[62] + E1[19]*Gx1[78] + E1[23]*Gx1[94] + E1[27]*Gx1[110] + E1[31]*Gx1[126] + E1[35]*Gx1[142] + E1[39]*Gx1[158] + E1[43]*Gx1[174] + E1[47]*Gx1[190] + E1[51]*Gx1[206] + E1[55]*Gx1[222] + E1[59]*Gx1[238] + E1[63]*Gx1[254];
H101[63] += + E1[3]*Gx1[15] + E1[7]*Gx1[31] + E1[11]*Gx1[47] + E1[15]*Gx1[63] + E1[19]*Gx1[79] + E1[23]*Gx1[95] + E1[27]*Gx1[111] + E1[31]*Gx1[127] + E1[35]*Gx1[143] + E1[39]*Gx1[159] + E1[43]*Gx1[175] + E1[47]*Gx1[191] + E1[51]*Gx1[207] + E1[55]*Gx1[223] + E1[59]*Gx1[239] + E1[63]*Gx1[255];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 64; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
dNew[8] += + E1[32]*U1[0] + E1[33]*U1[1] + E1[34]*U1[2] + E1[35]*U1[3];
dNew[9] += + E1[36]*U1[0] + E1[37]*U1[1] + E1[38]*U1[2] + E1[39]*U1[3];
dNew[10] += + E1[40]*U1[0] + E1[41]*U1[1] + E1[42]*U1[2] + E1[43]*U1[3];
dNew[11] += + E1[44]*U1[0] + E1[45]*U1[1] + E1[46]*U1[2] + E1[47]*U1[3];
dNew[12] += + E1[48]*U1[0] + E1[49]*U1[1] + E1[50]*U1[2] + E1[51]*U1[3];
dNew[13] += + E1[52]*U1[0] + E1[53]*U1[1] + E1[54]*U1[2] + E1[55]*U1[3];
dNew[14] += + E1[56]*U1[0] + E1[57]*U1[1] + E1[58]*U1[2] + E1[59]*U1[3];
dNew[15] += + E1[60]*U1[0] + E1[61]*U1[1] + E1[62]*U1[2] + E1[63]*U1[3];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[5] = 0.0000000000000000e+00;
nmpcWorkspace.H[6] = 0.0000000000000000e+00;
nmpcWorkspace.H[7] = 0.0000000000000000e+00;
nmpcWorkspace.H[8] = 0.0000000000000000e+00;
nmpcWorkspace.H[9] = 0.0000000000000000e+00;
nmpcWorkspace.H[10] = 0.0000000000000000e+00;
nmpcWorkspace.H[11] = 0.0000000000000000e+00;
nmpcWorkspace.H[12] = 0.0000000000000000e+00;
nmpcWorkspace.H[13] = 0.0000000000000000e+00;
nmpcWorkspace.H[14] = 0.0000000000000000e+00;
nmpcWorkspace.H[15] = 0.0000000000000000e+00;
nmpcWorkspace.H[136] = 0.0000000000000000e+00;
nmpcWorkspace.H[137] = 0.0000000000000000e+00;
nmpcWorkspace.H[138] = 0.0000000000000000e+00;
nmpcWorkspace.H[139] = 0.0000000000000000e+00;
nmpcWorkspace.H[140] = 0.0000000000000000e+00;
nmpcWorkspace.H[141] = 0.0000000000000000e+00;
nmpcWorkspace.H[142] = 0.0000000000000000e+00;
nmpcWorkspace.H[143] = 0.0000000000000000e+00;
nmpcWorkspace.H[144] = 0.0000000000000000e+00;
nmpcWorkspace.H[145] = 0.0000000000000000e+00;
nmpcWorkspace.H[146] = 0.0000000000000000e+00;
nmpcWorkspace.H[147] = 0.0000000000000000e+00;
nmpcWorkspace.H[148] = 0.0000000000000000e+00;
nmpcWorkspace.H[149] = 0.0000000000000000e+00;
nmpcWorkspace.H[150] = 0.0000000000000000e+00;
nmpcWorkspace.H[151] = 0.0000000000000000e+00;
nmpcWorkspace.H[272] = 0.0000000000000000e+00;
nmpcWorkspace.H[273] = 0.0000000000000000e+00;
nmpcWorkspace.H[274] = 0.0000000000000000e+00;
nmpcWorkspace.H[275] = 0.0000000000000000e+00;
nmpcWorkspace.H[276] = 0.0000000000000000e+00;
nmpcWorkspace.H[277] = 0.0000000000000000e+00;
nmpcWorkspace.H[278] = 0.0000000000000000e+00;
nmpcWorkspace.H[279] = 0.0000000000000000e+00;
nmpcWorkspace.H[280] = 0.0000000000000000e+00;
nmpcWorkspace.H[281] = 0.0000000000000000e+00;
nmpcWorkspace.H[282] = 0.0000000000000000e+00;
nmpcWorkspace.H[283] = 0.0000000000000000e+00;
nmpcWorkspace.H[284] = 0.0000000000000000e+00;
nmpcWorkspace.H[285] = 0.0000000000000000e+00;
nmpcWorkspace.H[286] = 0.0000000000000000e+00;
nmpcWorkspace.H[287] = 0.0000000000000000e+00;
nmpcWorkspace.H[408] = 0.0000000000000000e+00;
nmpcWorkspace.H[409] = 0.0000000000000000e+00;
nmpcWorkspace.H[410] = 0.0000000000000000e+00;
nmpcWorkspace.H[411] = 0.0000000000000000e+00;
nmpcWorkspace.H[412] = 0.0000000000000000e+00;
nmpcWorkspace.H[413] = 0.0000000000000000e+00;
nmpcWorkspace.H[414] = 0.0000000000000000e+00;
nmpcWorkspace.H[415] = 0.0000000000000000e+00;
nmpcWorkspace.H[416] = 0.0000000000000000e+00;
nmpcWorkspace.H[417] = 0.0000000000000000e+00;
nmpcWorkspace.H[418] = 0.0000000000000000e+00;
nmpcWorkspace.H[419] = 0.0000000000000000e+00;
nmpcWorkspace.H[420] = 0.0000000000000000e+00;
nmpcWorkspace.H[421] = 0.0000000000000000e+00;
nmpcWorkspace.H[422] = 0.0000000000000000e+00;
nmpcWorkspace.H[423] = 0.0000000000000000e+00;
nmpcWorkspace.H[544] = 0.0000000000000000e+00;
nmpcWorkspace.H[545] = 0.0000000000000000e+00;
nmpcWorkspace.H[546] = 0.0000000000000000e+00;
nmpcWorkspace.H[547] = 0.0000000000000000e+00;
nmpcWorkspace.H[548] = 0.0000000000000000e+00;
nmpcWorkspace.H[549] = 0.0000000000000000e+00;
nmpcWorkspace.H[550] = 0.0000000000000000e+00;
nmpcWorkspace.H[551] = 0.0000000000000000e+00;
nmpcWorkspace.H[552] = 0.0000000000000000e+00;
nmpcWorkspace.H[553] = 0.0000000000000000e+00;
nmpcWorkspace.H[554] = 0.0000000000000000e+00;
nmpcWorkspace.H[555] = 0.0000000000000000e+00;
nmpcWorkspace.H[556] = 0.0000000000000000e+00;
nmpcWorkspace.H[557] = 0.0000000000000000e+00;
nmpcWorkspace.H[558] = 0.0000000000000000e+00;
nmpcWorkspace.H[559] = 0.0000000000000000e+00;
nmpcWorkspace.H[680] = 0.0000000000000000e+00;
nmpcWorkspace.H[681] = 0.0000000000000000e+00;
nmpcWorkspace.H[682] = 0.0000000000000000e+00;
nmpcWorkspace.H[683] = 0.0000000000000000e+00;
nmpcWorkspace.H[684] = 0.0000000000000000e+00;
nmpcWorkspace.H[685] = 0.0000000000000000e+00;
nmpcWorkspace.H[686] = 0.0000000000000000e+00;
nmpcWorkspace.H[687] = 0.0000000000000000e+00;
nmpcWorkspace.H[688] = 0.0000000000000000e+00;
nmpcWorkspace.H[689] = 0.0000000000000000e+00;
nmpcWorkspace.H[690] = 0.0000000000000000e+00;
nmpcWorkspace.H[691] = 0.0000000000000000e+00;
nmpcWorkspace.H[692] = 0.0000000000000000e+00;
nmpcWorkspace.H[693] = 0.0000000000000000e+00;
nmpcWorkspace.H[694] = 0.0000000000000000e+00;
nmpcWorkspace.H[695] = 0.0000000000000000e+00;
nmpcWorkspace.H[816] = 0.0000000000000000e+00;
nmpcWorkspace.H[817] = 0.0000000000000000e+00;
nmpcWorkspace.H[818] = 0.0000000000000000e+00;
nmpcWorkspace.H[819] = 0.0000000000000000e+00;
nmpcWorkspace.H[820] = 0.0000000000000000e+00;
nmpcWorkspace.H[821] = 0.0000000000000000e+00;
nmpcWorkspace.H[822] = 0.0000000000000000e+00;
nmpcWorkspace.H[823] = 0.0000000000000000e+00;
nmpcWorkspace.H[824] = 0.0000000000000000e+00;
nmpcWorkspace.H[825] = 0.0000000000000000e+00;
nmpcWorkspace.H[826] = 0.0000000000000000e+00;
nmpcWorkspace.H[827] = 0.0000000000000000e+00;
nmpcWorkspace.H[828] = 0.0000000000000000e+00;
nmpcWorkspace.H[829] = 0.0000000000000000e+00;
nmpcWorkspace.H[830] = 0.0000000000000000e+00;
nmpcWorkspace.H[831] = 0.0000000000000000e+00;
nmpcWorkspace.H[952] = 0.0000000000000000e+00;
nmpcWorkspace.H[953] = 0.0000000000000000e+00;
nmpcWorkspace.H[954] = 0.0000000000000000e+00;
nmpcWorkspace.H[955] = 0.0000000000000000e+00;
nmpcWorkspace.H[956] = 0.0000000000000000e+00;
nmpcWorkspace.H[957] = 0.0000000000000000e+00;
nmpcWorkspace.H[958] = 0.0000000000000000e+00;
nmpcWorkspace.H[959] = 0.0000000000000000e+00;
nmpcWorkspace.H[960] = 0.0000000000000000e+00;
nmpcWorkspace.H[961] = 0.0000000000000000e+00;
nmpcWorkspace.H[962] = 0.0000000000000000e+00;
nmpcWorkspace.H[963] = 0.0000000000000000e+00;
nmpcWorkspace.H[964] = 0.0000000000000000e+00;
nmpcWorkspace.H[965] = 0.0000000000000000e+00;
nmpcWorkspace.H[966] = 0.0000000000000000e+00;
nmpcWorkspace.H[967] = 0.0000000000000000e+00;
nmpcWorkspace.H[1088] = 0.0000000000000000e+00;
nmpcWorkspace.H[1089] = 0.0000000000000000e+00;
nmpcWorkspace.H[1090] = 0.0000000000000000e+00;
nmpcWorkspace.H[1091] = 0.0000000000000000e+00;
nmpcWorkspace.H[1092] = 0.0000000000000000e+00;
nmpcWorkspace.H[1093] = 0.0000000000000000e+00;
nmpcWorkspace.H[1094] = 0.0000000000000000e+00;
nmpcWorkspace.H[1095] = 0.0000000000000000e+00;
nmpcWorkspace.H[1096] = 0.0000000000000000e+00;
nmpcWorkspace.H[1097] = 0.0000000000000000e+00;
nmpcWorkspace.H[1098] = 0.0000000000000000e+00;
nmpcWorkspace.H[1099] = 0.0000000000000000e+00;
nmpcWorkspace.H[1100] = 0.0000000000000000e+00;
nmpcWorkspace.H[1101] = 0.0000000000000000e+00;
nmpcWorkspace.H[1102] = 0.0000000000000000e+00;
nmpcWorkspace.H[1103] = 0.0000000000000000e+00;
nmpcWorkspace.H[1224] = 0.0000000000000000e+00;
nmpcWorkspace.H[1225] = 0.0000000000000000e+00;
nmpcWorkspace.H[1226] = 0.0000000000000000e+00;
nmpcWorkspace.H[1227] = 0.0000000000000000e+00;
nmpcWorkspace.H[1228] = 0.0000000000000000e+00;
nmpcWorkspace.H[1229] = 0.0000000000000000e+00;
nmpcWorkspace.H[1230] = 0.0000000000000000e+00;
nmpcWorkspace.H[1231] = 0.0000000000000000e+00;
nmpcWorkspace.H[1232] = 0.0000000000000000e+00;
nmpcWorkspace.H[1233] = 0.0000000000000000e+00;
nmpcWorkspace.H[1234] = 0.0000000000000000e+00;
nmpcWorkspace.H[1235] = 0.0000000000000000e+00;
nmpcWorkspace.H[1236] = 0.0000000000000000e+00;
nmpcWorkspace.H[1237] = 0.0000000000000000e+00;
nmpcWorkspace.H[1238] = 0.0000000000000000e+00;
nmpcWorkspace.H[1239] = 0.0000000000000000e+00;
nmpcWorkspace.H[1360] = 0.0000000000000000e+00;
nmpcWorkspace.H[1361] = 0.0000000000000000e+00;
nmpcWorkspace.H[1362] = 0.0000000000000000e+00;
nmpcWorkspace.H[1363] = 0.0000000000000000e+00;
nmpcWorkspace.H[1364] = 0.0000000000000000e+00;
nmpcWorkspace.H[1365] = 0.0000000000000000e+00;
nmpcWorkspace.H[1366] = 0.0000000000000000e+00;
nmpcWorkspace.H[1367] = 0.0000000000000000e+00;
nmpcWorkspace.H[1368] = 0.0000000000000000e+00;
nmpcWorkspace.H[1369] = 0.0000000000000000e+00;
nmpcWorkspace.H[1370] = 0.0000000000000000e+00;
nmpcWorkspace.H[1371] = 0.0000000000000000e+00;
nmpcWorkspace.H[1372] = 0.0000000000000000e+00;
nmpcWorkspace.H[1373] = 0.0000000000000000e+00;
nmpcWorkspace.H[1374] = 0.0000000000000000e+00;
nmpcWorkspace.H[1375] = 0.0000000000000000e+00;
nmpcWorkspace.H[1496] = 0.0000000000000000e+00;
nmpcWorkspace.H[1497] = 0.0000000000000000e+00;
nmpcWorkspace.H[1498] = 0.0000000000000000e+00;
nmpcWorkspace.H[1499] = 0.0000000000000000e+00;
nmpcWorkspace.H[1500] = 0.0000000000000000e+00;
nmpcWorkspace.H[1501] = 0.0000000000000000e+00;
nmpcWorkspace.H[1502] = 0.0000000000000000e+00;
nmpcWorkspace.H[1503] = 0.0000000000000000e+00;
nmpcWorkspace.H[1504] = 0.0000000000000000e+00;
nmpcWorkspace.H[1505] = 0.0000000000000000e+00;
nmpcWorkspace.H[1506] = 0.0000000000000000e+00;
nmpcWorkspace.H[1507] = 0.0000000000000000e+00;
nmpcWorkspace.H[1508] = 0.0000000000000000e+00;
nmpcWorkspace.H[1509] = 0.0000000000000000e+00;
nmpcWorkspace.H[1510] = 0.0000000000000000e+00;
nmpcWorkspace.H[1511] = 0.0000000000000000e+00;
nmpcWorkspace.H[1632] = 0.0000000000000000e+00;
nmpcWorkspace.H[1633] = 0.0000000000000000e+00;
nmpcWorkspace.H[1634] = 0.0000000000000000e+00;
nmpcWorkspace.H[1635] = 0.0000000000000000e+00;
nmpcWorkspace.H[1636] = 0.0000000000000000e+00;
nmpcWorkspace.H[1637] = 0.0000000000000000e+00;
nmpcWorkspace.H[1638] = 0.0000000000000000e+00;
nmpcWorkspace.H[1639] = 0.0000000000000000e+00;
nmpcWorkspace.H[1640] = 0.0000000000000000e+00;
nmpcWorkspace.H[1641] = 0.0000000000000000e+00;
nmpcWorkspace.H[1642] = 0.0000000000000000e+00;
nmpcWorkspace.H[1643] = 0.0000000000000000e+00;
nmpcWorkspace.H[1644] = 0.0000000000000000e+00;
nmpcWorkspace.H[1645] = 0.0000000000000000e+00;
nmpcWorkspace.H[1646] = 0.0000000000000000e+00;
nmpcWorkspace.H[1647] = 0.0000000000000000e+00;
nmpcWorkspace.H[1768] = 0.0000000000000000e+00;
nmpcWorkspace.H[1769] = 0.0000000000000000e+00;
nmpcWorkspace.H[1770] = 0.0000000000000000e+00;
nmpcWorkspace.H[1771] = 0.0000000000000000e+00;
nmpcWorkspace.H[1772] = 0.0000000000000000e+00;
nmpcWorkspace.H[1773] = 0.0000000000000000e+00;
nmpcWorkspace.H[1774] = 0.0000000000000000e+00;
nmpcWorkspace.H[1775] = 0.0000000000000000e+00;
nmpcWorkspace.H[1776] = 0.0000000000000000e+00;
nmpcWorkspace.H[1777] = 0.0000000000000000e+00;
nmpcWorkspace.H[1778] = 0.0000000000000000e+00;
nmpcWorkspace.H[1779] = 0.0000000000000000e+00;
nmpcWorkspace.H[1780] = 0.0000000000000000e+00;
nmpcWorkspace.H[1781] = 0.0000000000000000e+00;
nmpcWorkspace.H[1782] = 0.0000000000000000e+00;
nmpcWorkspace.H[1783] = 0.0000000000000000e+00;
nmpcWorkspace.H[1904] = 0.0000000000000000e+00;
nmpcWorkspace.H[1905] = 0.0000000000000000e+00;
nmpcWorkspace.H[1906] = 0.0000000000000000e+00;
nmpcWorkspace.H[1907] = 0.0000000000000000e+00;
nmpcWorkspace.H[1908] = 0.0000000000000000e+00;
nmpcWorkspace.H[1909] = 0.0000000000000000e+00;
nmpcWorkspace.H[1910] = 0.0000000000000000e+00;
nmpcWorkspace.H[1911] = 0.0000000000000000e+00;
nmpcWorkspace.H[1912] = 0.0000000000000000e+00;
nmpcWorkspace.H[1913] = 0.0000000000000000e+00;
nmpcWorkspace.H[1914] = 0.0000000000000000e+00;
nmpcWorkspace.H[1915] = 0.0000000000000000e+00;
nmpcWorkspace.H[1916] = 0.0000000000000000e+00;
nmpcWorkspace.H[1917] = 0.0000000000000000e+00;
nmpcWorkspace.H[1918] = 0.0000000000000000e+00;
nmpcWorkspace.H[1919] = 0.0000000000000000e+00;
nmpcWorkspace.H[2040] = 0.0000000000000000e+00;
nmpcWorkspace.H[2041] = 0.0000000000000000e+00;
nmpcWorkspace.H[2042] = 0.0000000000000000e+00;
nmpcWorkspace.H[2043] = 0.0000000000000000e+00;
nmpcWorkspace.H[2044] = 0.0000000000000000e+00;
nmpcWorkspace.H[2045] = 0.0000000000000000e+00;
nmpcWorkspace.H[2046] = 0.0000000000000000e+00;
nmpcWorkspace.H[2047] = 0.0000000000000000e+00;
nmpcWorkspace.H[2048] = 0.0000000000000000e+00;
nmpcWorkspace.H[2049] = 0.0000000000000000e+00;
nmpcWorkspace.H[2050] = 0.0000000000000000e+00;
nmpcWorkspace.H[2051] = 0.0000000000000000e+00;
nmpcWorkspace.H[2052] = 0.0000000000000000e+00;
nmpcWorkspace.H[2053] = 0.0000000000000000e+00;
nmpcWorkspace.H[2054] = 0.0000000000000000e+00;
nmpcWorkspace.H[2055] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 16; ++lRun1)
{
for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 16; ++lRun3)
{
t += + Gx1[(lRun3 * 16) + (lRun1)]*Gx2[(lRun3 * 16) + (lRun2)];
}
nmpcWorkspace.H[(lRun1 * 136) + (lRun2)] += t;
}
}
}

void nmpc_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
g0[5] += 0.0;
;
g0[6] += 0.0;
;
g0[7] += 0.0;
;
g0[8] += 0.0;
;
g0[9] += 0.0;
;
g0[10] += 0.0;
;
g0[11] += 0.0;
;
g0[12] += 0.0;
;
g0[13] += 0.0;
;
g0[14] += 0.0;
;
g0[15] += 0.0;
;
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 256 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 16-16 ]), &(nmpcWorkspace.evGx[ lRun1 * 256 ]), &(nmpcWorkspace.d[ lRun1 * 16 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 256-256 ]), &(nmpcWorkspace.evGx[ lRun1 * 256 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 64 ]), &(nmpcWorkspace.E[ lRun3 * 64 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 64 ]), &(nmpcWorkspace.E[ lRun3 * 64 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 256 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2048 ]), &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.evGx[ 2048 ]), &(nmpcWorkspace.QGx[ 2048 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2560 ]), &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2816 ]), &(nmpcWorkspace.evGx[ 2560 ]), &(nmpcWorkspace.QGx[ 2560 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3072 ]), &(nmpcWorkspace.evGx[ 2816 ]), &(nmpcWorkspace.QGx[ 2816 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3328 ]), &(nmpcWorkspace.evGx[ 3072 ]), &(nmpcWorkspace.QGx[ 3072 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3584 ]), &(nmpcWorkspace.evGx[ 3328 ]), &(nmpcWorkspace.QGx[ 3328 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3840 ]), &(nmpcWorkspace.evGx[ 3584 ]), &(nmpcWorkspace.QGx[ 3584 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4096 ]), &(nmpcWorkspace.evGx[ 3840 ]), &(nmpcWorkspace.QGx[ 3840 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4352 ]), &(nmpcWorkspace.evGx[ 4096 ]), &(nmpcWorkspace.QGx[ 4096 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4608 ]), &(nmpcWorkspace.evGx[ 4352 ]), &(nmpcWorkspace.QGx[ 4352 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4864 ]), &(nmpcWorkspace.evGx[ 4608 ]), &(nmpcWorkspace.QGx[ 4608 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5120 ]), &(nmpcWorkspace.evGx[ 4864 ]), &(nmpcWorkspace.QGx[ 4864 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5376 ]), &(nmpcWorkspace.evGx[ 5120 ]), &(nmpcWorkspace.QGx[ 5120 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5632 ]), &(nmpcWorkspace.evGx[ 5376 ]), &(nmpcWorkspace.QGx[ 5376 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5888 ]), &(nmpcWorkspace.evGx[ 5632 ]), &(nmpcWorkspace.QGx[ 5632 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6144 ]), &(nmpcWorkspace.evGx[ 5888 ]), &(nmpcWorkspace.QGx[ 5888 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6400 ]), &(nmpcWorkspace.evGx[ 6144 ]), &(nmpcWorkspace.QGx[ 6144 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6656 ]), &(nmpcWorkspace.evGx[ 6400 ]), &(nmpcWorkspace.QGx[ 6400 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6912 ]), &(nmpcWorkspace.evGx[ 6656 ]), &(nmpcWorkspace.QGx[ 6656 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 7168 ]), &(nmpcWorkspace.evGx[ 6912 ]), &(nmpcWorkspace.QGx[ 6912 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 7424 ]), &(nmpcWorkspace.evGx[ 7168 ]), &(nmpcWorkspace.QGx[ 7168 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 7424 ]), &(nmpcWorkspace.QGx[ 7424 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 256 + 256 ]), &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.QE[ lRun3 * 64 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.QE[ lRun3 * 64 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2048 ]), &(nmpcWorkspace.QGx[ 2048 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2560 ]), &(nmpcWorkspace.QGx[ 2560 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2816 ]), &(nmpcWorkspace.QGx[ 2816 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3072 ]), &(nmpcWorkspace.QGx[ 3072 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3328 ]), &(nmpcWorkspace.QGx[ 3328 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3584 ]), &(nmpcWorkspace.QGx[ 3584 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3840 ]), &(nmpcWorkspace.QGx[ 3840 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4096 ]), &(nmpcWorkspace.QGx[ 4096 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4352 ]), &(nmpcWorkspace.QGx[ 4352 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4608 ]), &(nmpcWorkspace.QGx[ 4608 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4864 ]), &(nmpcWorkspace.QGx[ 4864 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5120 ]), &(nmpcWorkspace.QGx[ 5120 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5376 ]), &(nmpcWorkspace.QGx[ 5376 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5632 ]), &(nmpcWorkspace.QGx[ 5632 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5888 ]), &(nmpcWorkspace.QGx[ 5888 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6144 ]), &(nmpcWorkspace.QGx[ 6144 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6400 ]), &(nmpcWorkspace.QGx[ 6400 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6656 ]), &(nmpcWorkspace.QGx[ 6656 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6912 ]), &(nmpcWorkspace.QGx[ 6912 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 7168 ]), &(nmpcWorkspace.QGx[ 7168 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 7424 ]), &(nmpcWorkspace.QGx[ 7424 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 64 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 64 ]), &(nmpcWorkspace.evGx[ lRun2 * 256 ]), &(nmpcWorkspace.H10[ lRun1 * 64 ]) );
}
}

for (lRun1 = 0;lRun1 < 16; ++lRun1)
for (lRun2 = 0;lRun2 < 120; ++lRun2)
nmpcWorkspace.H[(lRun1 * 136) + (lRun2 + 16)] = nmpcWorkspace.H10[(lRun2 * 16) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 64 ]), &(nmpcWorkspace.QE[ lRun5 * 64 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 64 ]), &(nmpcWorkspace.QE[ lRun5 * 64 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 120; ++lRun1)
for (lRun2 = 0;lRun2 < 16; ++lRun2)
nmpcWorkspace.H[(lRun1 * 136 + 2176) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 16) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 256 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.Qd[ 16 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.Qd[ 32 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.Qd[ 64 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.d[ 80 ]), &(nmpcWorkspace.Qd[ 80 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2048 ]), &(nmpcWorkspace.d[ 112 ]), &(nmpcWorkspace.Qd[ 112 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.d[ 128 ]), &(nmpcWorkspace.Qd[ 128 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2560 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2816 ]), &(nmpcWorkspace.d[ 160 ]), &(nmpcWorkspace.Qd[ 160 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3072 ]), &(nmpcWorkspace.d[ 176 ]), &(nmpcWorkspace.Qd[ 176 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3328 ]), &(nmpcWorkspace.d[ 192 ]), &(nmpcWorkspace.Qd[ 192 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3584 ]), &(nmpcWorkspace.d[ 208 ]), &(nmpcWorkspace.Qd[ 208 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3840 ]), &(nmpcWorkspace.d[ 224 ]), &(nmpcWorkspace.Qd[ 224 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4096 ]), &(nmpcWorkspace.d[ 240 ]), &(nmpcWorkspace.Qd[ 240 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4352 ]), &(nmpcWorkspace.d[ 256 ]), &(nmpcWorkspace.Qd[ 256 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4608 ]), &(nmpcWorkspace.d[ 272 ]), &(nmpcWorkspace.Qd[ 272 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4864 ]), &(nmpcWorkspace.d[ 288 ]), &(nmpcWorkspace.Qd[ 288 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5120 ]), &(nmpcWorkspace.d[ 304 ]), &(nmpcWorkspace.Qd[ 304 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5376 ]), &(nmpcWorkspace.d[ 320 ]), &(nmpcWorkspace.Qd[ 320 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5632 ]), &(nmpcWorkspace.d[ 336 ]), &(nmpcWorkspace.Qd[ 336 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5888 ]), &(nmpcWorkspace.d[ 352 ]), &(nmpcWorkspace.Qd[ 352 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6144 ]), &(nmpcWorkspace.d[ 368 ]), &(nmpcWorkspace.Qd[ 368 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6400 ]), &(nmpcWorkspace.d[ 384 ]), &(nmpcWorkspace.Qd[ 384 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6656 ]), &(nmpcWorkspace.d[ 400 ]), &(nmpcWorkspace.Qd[ 400 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6912 ]), &(nmpcWorkspace.d[ 416 ]), &(nmpcWorkspace.Qd[ 416 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 7168 ]), &(nmpcWorkspace.d[ 432 ]), &(nmpcWorkspace.Qd[ 432 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 7424 ]), &(nmpcWorkspace.d[ 448 ]), &(nmpcWorkspace.Qd[ 448 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 464 ]), &(nmpcWorkspace.Qd[ 464 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 512 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 768 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1024 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1280 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1536 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1792 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2048 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2304 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2560 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2816 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3072 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3328 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3584 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3840 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4096 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4352 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4608 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4864 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5120 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5376 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5632 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5888 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6144 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6400 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6656 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6912 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 7168 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 7424 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 64 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 16 ]) );
}
}
nmpcWorkspace.lb[16] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[0];
nmpcWorkspace.lb[17] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[1];
nmpcWorkspace.lb[18] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[2];
nmpcWorkspace.lb[19] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[3];
nmpcWorkspace.lb[20] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[4];
nmpcWorkspace.lb[21] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[5];
nmpcWorkspace.lb[22] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[6];
nmpcWorkspace.lb[23] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[7];
nmpcWorkspace.lb[24] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[8];
nmpcWorkspace.lb[25] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[9];
nmpcWorkspace.lb[26] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[10];
nmpcWorkspace.lb[27] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[11];
nmpcWorkspace.lb[28] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[12];
nmpcWorkspace.lb[29] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[13];
nmpcWorkspace.lb[30] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[14];
nmpcWorkspace.lb[31] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[15];
nmpcWorkspace.lb[32] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[16];
nmpcWorkspace.lb[33] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[17];
nmpcWorkspace.lb[34] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[18];
nmpcWorkspace.lb[35] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[19];
nmpcWorkspace.lb[36] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[20];
nmpcWorkspace.lb[37] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[21];
nmpcWorkspace.lb[38] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[22];
nmpcWorkspace.lb[39] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[23];
nmpcWorkspace.lb[40] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[24];
nmpcWorkspace.lb[41] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[25];
nmpcWorkspace.lb[42] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[26];
nmpcWorkspace.lb[43] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[27];
nmpcWorkspace.lb[44] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[28];
nmpcWorkspace.lb[45] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[29];
nmpcWorkspace.lb[46] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[30];
nmpcWorkspace.lb[47] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[31];
nmpcWorkspace.lb[48] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[32];
nmpcWorkspace.lb[49] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[33];
nmpcWorkspace.lb[50] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[34];
nmpcWorkspace.lb[51] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[35];
nmpcWorkspace.lb[52] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[36];
nmpcWorkspace.lb[53] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[37];
nmpcWorkspace.lb[54] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[38];
nmpcWorkspace.lb[55] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[39];
nmpcWorkspace.lb[56] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[40];
nmpcWorkspace.lb[57] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[41];
nmpcWorkspace.lb[58] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[42];
nmpcWorkspace.lb[59] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[43];
nmpcWorkspace.lb[60] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[44];
nmpcWorkspace.lb[61] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[45];
nmpcWorkspace.lb[62] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[46];
nmpcWorkspace.lb[63] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[47];
nmpcWorkspace.lb[64] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[48];
nmpcWorkspace.lb[65] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[49];
nmpcWorkspace.lb[66] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[50];
nmpcWorkspace.lb[67] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[51];
nmpcWorkspace.lb[68] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[52];
nmpcWorkspace.lb[69] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[53];
nmpcWorkspace.lb[70] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[54];
nmpcWorkspace.lb[71] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[55];
nmpcWorkspace.lb[72] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[56];
nmpcWorkspace.lb[73] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[57];
nmpcWorkspace.lb[74] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[58];
nmpcWorkspace.lb[75] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[59];
nmpcWorkspace.lb[76] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[60];
nmpcWorkspace.lb[77] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[61];
nmpcWorkspace.lb[78] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[62];
nmpcWorkspace.lb[79] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[63];
nmpcWorkspace.lb[80] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[64];
nmpcWorkspace.lb[81] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[65];
nmpcWorkspace.lb[82] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[66];
nmpcWorkspace.lb[83] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[67];
nmpcWorkspace.lb[84] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[68];
nmpcWorkspace.lb[85] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[69];
nmpcWorkspace.lb[86] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[70];
nmpcWorkspace.lb[87] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[71];
nmpcWorkspace.lb[88] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[72];
nmpcWorkspace.lb[89] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[73];
nmpcWorkspace.lb[90] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[74];
nmpcWorkspace.lb[91] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[75];
nmpcWorkspace.lb[92] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[76];
nmpcWorkspace.lb[93] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[77];
nmpcWorkspace.lb[94] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[78];
nmpcWorkspace.lb[95] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[79];
nmpcWorkspace.lb[96] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[80];
nmpcWorkspace.lb[97] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[81];
nmpcWorkspace.lb[98] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[82];
nmpcWorkspace.lb[99] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[83];
nmpcWorkspace.lb[100] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[84];
nmpcWorkspace.lb[101] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[85];
nmpcWorkspace.lb[102] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[86];
nmpcWorkspace.lb[103] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[87];
nmpcWorkspace.lb[104] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[88];
nmpcWorkspace.lb[105] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[89];
nmpcWorkspace.lb[106] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[90];
nmpcWorkspace.lb[107] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[91];
nmpcWorkspace.lb[108] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[92];
nmpcWorkspace.lb[109] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[93];
nmpcWorkspace.lb[110] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[94];
nmpcWorkspace.lb[111] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[95];
nmpcWorkspace.lb[112] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[96];
nmpcWorkspace.lb[113] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[97];
nmpcWorkspace.lb[114] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[98];
nmpcWorkspace.lb[115] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[99];
nmpcWorkspace.lb[116] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[100];
nmpcWorkspace.lb[117] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[101];
nmpcWorkspace.lb[118] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[102];
nmpcWorkspace.lb[119] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[103];
nmpcWorkspace.lb[120] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[104];
nmpcWorkspace.lb[121] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[105];
nmpcWorkspace.lb[122] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[106];
nmpcWorkspace.lb[123] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[107];
nmpcWorkspace.lb[124] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[108];
nmpcWorkspace.lb[125] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[109];
nmpcWorkspace.lb[126] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[110];
nmpcWorkspace.lb[127] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[111];
nmpcWorkspace.lb[128] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[112];
nmpcWorkspace.lb[129] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[113];
nmpcWorkspace.lb[130] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[114];
nmpcWorkspace.lb[131] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[115];
nmpcWorkspace.lb[132] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[116];
nmpcWorkspace.lb[133] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[117];
nmpcWorkspace.lb[134] = (real_t)-1.7453292519943295e+00 - nmpcVariables.u[118];
nmpcWorkspace.lb[135] = (real_t)1.1183399999999999e+01 - nmpcVariables.u[119];
nmpcWorkspace.ub[16] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[0];
nmpcWorkspace.ub[17] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[1];
nmpcWorkspace.ub[18] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[2];
nmpcWorkspace.ub[19] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[3];
nmpcWorkspace.ub[20] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[4];
nmpcWorkspace.ub[21] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[5];
nmpcWorkspace.ub[22] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[6];
nmpcWorkspace.ub[23] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[7];
nmpcWorkspace.ub[24] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[8];
nmpcWorkspace.ub[25] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[9];
nmpcWorkspace.ub[26] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[10];
nmpcWorkspace.ub[27] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[11];
nmpcWorkspace.ub[28] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[12];
nmpcWorkspace.ub[29] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[13];
nmpcWorkspace.ub[30] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[14];
nmpcWorkspace.ub[31] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[15];
nmpcWorkspace.ub[32] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[16];
nmpcWorkspace.ub[33] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[17];
nmpcWorkspace.ub[34] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[18];
nmpcWorkspace.ub[35] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[19];
nmpcWorkspace.ub[36] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[20];
nmpcWorkspace.ub[37] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[21];
nmpcWorkspace.ub[38] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[22];
nmpcWorkspace.ub[39] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[23];
nmpcWorkspace.ub[40] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[24];
nmpcWorkspace.ub[41] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[25];
nmpcWorkspace.ub[42] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[26];
nmpcWorkspace.ub[43] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[27];
nmpcWorkspace.ub[44] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[28];
nmpcWorkspace.ub[45] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[29];
nmpcWorkspace.ub[46] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[30];
nmpcWorkspace.ub[47] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[31];
nmpcWorkspace.ub[48] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[32];
nmpcWorkspace.ub[49] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[33];
nmpcWorkspace.ub[50] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[34];
nmpcWorkspace.ub[51] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[35];
nmpcWorkspace.ub[52] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[36];
nmpcWorkspace.ub[53] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[37];
nmpcWorkspace.ub[54] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[38];
nmpcWorkspace.ub[55] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[39];
nmpcWorkspace.ub[56] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[40];
nmpcWorkspace.ub[57] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[41];
nmpcWorkspace.ub[58] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[42];
nmpcWorkspace.ub[59] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[43];
nmpcWorkspace.ub[60] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[44];
nmpcWorkspace.ub[61] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[45];
nmpcWorkspace.ub[62] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[46];
nmpcWorkspace.ub[63] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[47];
nmpcWorkspace.ub[64] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[48];
nmpcWorkspace.ub[65] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[49];
nmpcWorkspace.ub[66] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[50];
nmpcWorkspace.ub[67] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[51];
nmpcWorkspace.ub[68] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[52];
nmpcWorkspace.ub[69] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[53];
nmpcWorkspace.ub[70] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[54];
nmpcWorkspace.ub[71] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[55];
nmpcWorkspace.ub[72] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[56];
nmpcWorkspace.ub[73] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[57];
nmpcWorkspace.ub[74] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[58];
nmpcWorkspace.ub[75] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[59];
nmpcWorkspace.ub[76] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[60];
nmpcWorkspace.ub[77] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[61];
nmpcWorkspace.ub[78] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[62];
nmpcWorkspace.ub[79] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[63];
nmpcWorkspace.ub[80] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[64];
nmpcWorkspace.ub[81] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[65];
nmpcWorkspace.ub[82] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[66];
nmpcWorkspace.ub[83] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[67];
nmpcWorkspace.ub[84] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[68];
nmpcWorkspace.ub[85] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[69];
nmpcWorkspace.ub[86] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[70];
nmpcWorkspace.ub[87] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[71];
nmpcWorkspace.ub[88] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[72];
nmpcWorkspace.ub[89] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[73];
nmpcWorkspace.ub[90] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[74];
nmpcWorkspace.ub[91] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[75];
nmpcWorkspace.ub[92] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[76];
nmpcWorkspace.ub[93] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[77];
nmpcWorkspace.ub[94] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[78];
nmpcWorkspace.ub[95] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[79];
nmpcWorkspace.ub[96] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[80];
nmpcWorkspace.ub[97] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[81];
nmpcWorkspace.ub[98] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[82];
nmpcWorkspace.ub[99] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[83];
nmpcWorkspace.ub[100] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[84];
nmpcWorkspace.ub[101] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[85];
nmpcWorkspace.ub[102] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[86];
nmpcWorkspace.ub[103] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[87];
nmpcWorkspace.ub[104] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[88];
nmpcWorkspace.ub[105] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[89];
nmpcWorkspace.ub[106] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[90];
nmpcWorkspace.ub[107] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[91];
nmpcWorkspace.ub[108] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[92];
nmpcWorkspace.ub[109] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[93];
nmpcWorkspace.ub[110] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[94];
nmpcWorkspace.ub[111] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[95];
nmpcWorkspace.ub[112] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[96];
nmpcWorkspace.ub[113] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[97];
nmpcWorkspace.ub[114] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[98];
nmpcWorkspace.ub[115] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[99];
nmpcWorkspace.ub[116] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[100];
nmpcWorkspace.ub[117] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[101];
nmpcWorkspace.ub[118] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[102];
nmpcWorkspace.ub[119] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[103];
nmpcWorkspace.ub[120] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[104];
nmpcWorkspace.ub[121] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[105];
nmpcWorkspace.ub[122] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[106];
nmpcWorkspace.ub[123] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[107];
nmpcWorkspace.ub[124] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[108];
nmpcWorkspace.ub[125] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[109];
nmpcWorkspace.ub[126] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[110];
nmpcWorkspace.ub[127] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[111];
nmpcWorkspace.ub[128] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[112];
nmpcWorkspace.ub[129] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[113];
nmpcWorkspace.ub[130] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[114];
nmpcWorkspace.ub[131] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[115];
nmpcWorkspace.ub[132] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[116];
nmpcWorkspace.ub[133] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[117];
nmpcWorkspace.ub[134] = (real_t)1.7453292519943295e+00 - nmpcVariables.u[118];
nmpcWorkspace.ub[135] = (real_t)7.4555999999999997e+01 - nmpcVariables.u[119];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];
nmpcWorkspace.Dx0[8] = nmpcVariables.x0[8] - nmpcVariables.x[8];
nmpcWorkspace.Dx0[9] = nmpcVariables.x0[9] - nmpcVariables.x[9];
nmpcWorkspace.Dx0[10] = nmpcVariables.x0[10] - nmpcVariables.x[10];
nmpcWorkspace.Dx0[11] = nmpcVariables.x0[11] - nmpcVariables.x[11];
nmpcWorkspace.Dx0[12] = nmpcVariables.x0[12] - nmpcVariables.x[12];
nmpcWorkspace.Dx0[13] = nmpcVariables.x0[13] - nmpcVariables.x[13];
nmpcWorkspace.Dx0[14] = nmpcVariables.x0[14] - nmpcVariables.x[14];
nmpcWorkspace.Dx0[15] = nmpcVariables.x0[15] - nmpcVariables.x[15];

for (lRun2 = 0; lRun2 < 510; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] -= nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] -= nmpcVariables.yN[7];
nmpcWorkspace.DyN[8] -= nmpcVariables.yN[8];
nmpcWorkspace.DyN[9] -= nmpcVariables.yN[9];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 68 ]), &(nmpcWorkspace.Dy[ 17 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 136 ]), &(nmpcWorkspace.Dy[ 34 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 204 ]), &(nmpcWorkspace.Dy[ 51 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 272 ]), &(nmpcWorkspace.Dy[ 68 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 340 ]), &(nmpcWorkspace.Dy[ 85 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 408 ]), &(nmpcWorkspace.Dy[ 102 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 476 ]), &(nmpcWorkspace.Dy[ 119 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 544 ]), &(nmpcWorkspace.Dy[ 136 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 612 ]), &(nmpcWorkspace.Dy[ 153 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 680 ]), &(nmpcWorkspace.Dy[ 170 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 748 ]), &(nmpcWorkspace.Dy[ 187 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 816 ]), &(nmpcWorkspace.Dy[ 204 ]), &(nmpcWorkspace.g[ 64 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 884 ]), &(nmpcWorkspace.Dy[ 221 ]), &(nmpcWorkspace.g[ 68 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 952 ]), &(nmpcWorkspace.Dy[ 238 ]), &(nmpcWorkspace.g[ 72 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1020 ]), &(nmpcWorkspace.Dy[ 255 ]), &(nmpcWorkspace.g[ 76 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1088 ]), &(nmpcWorkspace.Dy[ 272 ]), &(nmpcWorkspace.g[ 80 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1156 ]), &(nmpcWorkspace.Dy[ 289 ]), &(nmpcWorkspace.g[ 84 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1224 ]), &(nmpcWorkspace.Dy[ 306 ]), &(nmpcWorkspace.g[ 88 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1292 ]), &(nmpcWorkspace.Dy[ 323 ]), &(nmpcWorkspace.g[ 92 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1360 ]), &(nmpcWorkspace.Dy[ 340 ]), &(nmpcWorkspace.g[ 96 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1428 ]), &(nmpcWorkspace.Dy[ 357 ]), &(nmpcWorkspace.g[ 100 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1496 ]), &(nmpcWorkspace.Dy[ 374 ]), &(nmpcWorkspace.g[ 104 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1564 ]), &(nmpcWorkspace.Dy[ 391 ]), &(nmpcWorkspace.g[ 108 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1632 ]), &(nmpcWorkspace.Dy[ 408 ]), &(nmpcWorkspace.g[ 112 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1700 ]), &(nmpcWorkspace.Dy[ 425 ]), &(nmpcWorkspace.g[ 116 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1768 ]), &(nmpcWorkspace.Dy[ 442 ]), &(nmpcWorkspace.g[ 120 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1836 ]), &(nmpcWorkspace.Dy[ 459 ]), &(nmpcWorkspace.g[ 124 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1904 ]), &(nmpcWorkspace.Dy[ 476 ]), &(nmpcWorkspace.g[ 128 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1972 ]), &(nmpcWorkspace.Dy[ 493 ]), &(nmpcWorkspace.g[ 132 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 272 ]), &(nmpcWorkspace.Dy[ 17 ]), &(nmpcWorkspace.QDy[ 16 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 544 ]), &(nmpcWorkspace.Dy[ 34 ]), &(nmpcWorkspace.QDy[ 32 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 816 ]), &(nmpcWorkspace.Dy[ 51 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1088 ]), &(nmpcWorkspace.Dy[ 68 ]), &(nmpcWorkspace.QDy[ 64 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1360 ]), &(nmpcWorkspace.Dy[ 85 ]), &(nmpcWorkspace.QDy[ 80 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1632 ]), &(nmpcWorkspace.Dy[ 102 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1904 ]), &(nmpcWorkspace.Dy[ 119 ]), &(nmpcWorkspace.QDy[ 112 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2176 ]), &(nmpcWorkspace.Dy[ 136 ]), &(nmpcWorkspace.QDy[ 128 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2448 ]), &(nmpcWorkspace.Dy[ 153 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2720 ]), &(nmpcWorkspace.Dy[ 170 ]), &(nmpcWorkspace.QDy[ 160 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2992 ]), &(nmpcWorkspace.Dy[ 187 ]), &(nmpcWorkspace.QDy[ 176 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3264 ]), &(nmpcWorkspace.Dy[ 204 ]), &(nmpcWorkspace.QDy[ 192 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3536 ]), &(nmpcWorkspace.Dy[ 221 ]), &(nmpcWorkspace.QDy[ 208 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3808 ]), &(nmpcWorkspace.Dy[ 238 ]), &(nmpcWorkspace.QDy[ 224 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4080 ]), &(nmpcWorkspace.Dy[ 255 ]), &(nmpcWorkspace.QDy[ 240 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4352 ]), &(nmpcWorkspace.Dy[ 272 ]), &(nmpcWorkspace.QDy[ 256 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4624 ]), &(nmpcWorkspace.Dy[ 289 ]), &(nmpcWorkspace.QDy[ 272 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4896 ]), &(nmpcWorkspace.Dy[ 306 ]), &(nmpcWorkspace.QDy[ 288 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5168 ]), &(nmpcWorkspace.Dy[ 323 ]), &(nmpcWorkspace.QDy[ 304 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5440 ]), &(nmpcWorkspace.Dy[ 340 ]), &(nmpcWorkspace.QDy[ 320 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5712 ]), &(nmpcWorkspace.Dy[ 357 ]), &(nmpcWorkspace.QDy[ 336 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5984 ]), &(nmpcWorkspace.Dy[ 374 ]), &(nmpcWorkspace.QDy[ 352 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6256 ]), &(nmpcWorkspace.Dy[ 391 ]), &(nmpcWorkspace.QDy[ 368 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6528 ]), &(nmpcWorkspace.Dy[ 408 ]), &(nmpcWorkspace.QDy[ 384 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6800 ]), &(nmpcWorkspace.Dy[ 425 ]), &(nmpcWorkspace.QDy[ 400 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7072 ]), &(nmpcWorkspace.Dy[ 442 ]), &(nmpcWorkspace.QDy[ 416 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7344 ]), &(nmpcWorkspace.Dy[ 459 ]), &(nmpcWorkspace.QDy[ 432 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7616 ]), &(nmpcWorkspace.Dy[ 476 ]), &(nmpcWorkspace.QDy[ 448 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7888 ]), &(nmpcWorkspace.Dy[ 493 ]), &(nmpcWorkspace.QDy[ 464 ]) );

nmpcWorkspace.QDy[480] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[481] = + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[482] = + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[483] = + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[484] = + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[485] = + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[486] = + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[64]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[65]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[66]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[67]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[68]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[69]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[487] = + nmpcWorkspace.QN2[70]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[71]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[72]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[73]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[74]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[75]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[76]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[77]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[78]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[79]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[488] = + nmpcWorkspace.QN2[80]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[81]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[82]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[83]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[84]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[85]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[86]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[87]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[88]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[89]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[489] = + nmpcWorkspace.QN2[90]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[91]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[92]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[93]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[94]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[95]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[96]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[97]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[98]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[99]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[490] = + nmpcWorkspace.QN2[100]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[101]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[102]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[103]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[104]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[105]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[106]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[107]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[108]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[109]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[491] = + nmpcWorkspace.QN2[110]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[111]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[112]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[113]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[114]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[115]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[116]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[117]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[118]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[119]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[492] = + nmpcWorkspace.QN2[120]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[121]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[122]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[123]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[124]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[125]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[126]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[127]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[128]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[129]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[493] = + nmpcWorkspace.QN2[130]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[131]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[132]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[133]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[134]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[135]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[136]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[137]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[138]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[139]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[494] = + nmpcWorkspace.QN2[140]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[141]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[142]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[143]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[144]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[145]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[146]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[147]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[148]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[149]*nmpcWorkspace.DyN[9];
nmpcWorkspace.QDy[495] = + nmpcWorkspace.QN2[150]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[151]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[152]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[153]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[154]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[155]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[156]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[157]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[158]*nmpcWorkspace.DyN[8] + nmpcWorkspace.QN2[159]*nmpcWorkspace.DyN[9];

for (lRun2 = 0; lRun2 < 480; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 16] += nmpcWorkspace.Qd[lRun2];


for (lRun2 = 0; lRun2 < 16; ++lRun2)
{
for (lRun4 = 0; lRun4 < 1; ++lRun4)
{
real_t t = 0.0;
for (lRun5 = 0; lRun5 < 480; ++lRun5)
{
t += + nmpcWorkspace.evGx[(lRun5 * 16) + (lRun2)]*nmpcWorkspace.QDy[(lRun5 + 16) + (lRun4)];
}
nmpcWorkspace.g[(lRun2) + (lRun4)] = t;
}
}


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.QDy[ lRun2 * 16 + 16 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 16 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.lb[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.lb[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.lb[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.lb[8] = nmpcWorkspace.Dx0[8];
nmpcWorkspace.lb[9] = nmpcWorkspace.Dx0[9];
nmpcWorkspace.lb[10] = nmpcWorkspace.Dx0[10];
nmpcWorkspace.lb[11] = nmpcWorkspace.Dx0[11];
nmpcWorkspace.lb[12] = nmpcWorkspace.Dx0[12];
nmpcWorkspace.lb[13] = nmpcWorkspace.Dx0[13];
nmpcWorkspace.lb[14] = nmpcWorkspace.Dx0[14];
nmpcWorkspace.lb[15] = nmpcWorkspace.Dx0[15];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.ub[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.ub[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.ub[8] = nmpcWorkspace.Dx0[8];
nmpcWorkspace.ub[9] = nmpcWorkspace.Dx0[9];
nmpcWorkspace.ub[10] = nmpcWorkspace.Dx0[10];
nmpcWorkspace.ub[11] = nmpcWorkspace.Dx0[11];
nmpcWorkspace.ub[12] = nmpcWorkspace.Dx0[12];
nmpcWorkspace.ub[13] = nmpcWorkspace.Dx0[13];
nmpcWorkspace.ub[14] = nmpcWorkspace.Dx0[14];
nmpcWorkspace.ub[15] = nmpcWorkspace.Dx0[15];
}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];
nmpcVariables.x[5] += nmpcWorkspace.x[5];
nmpcVariables.x[6] += nmpcWorkspace.x[6];
nmpcVariables.x[7] += nmpcWorkspace.x[7];
nmpcVariables.x[8] += nmpcWorkspace.x[8];
nmpcVariables.x[9] += nmpcWorkspace.x[9];
nmpcVariables.x[10] += nmpcWorkspace.x[10];
nmpcVariables.x[11] += nmpcWorkspace.x[11];
nmpcVariables.x[12] += nmpcWorkspace.x[12];
nmpcVariables.x[13] += nmpcWorkspace.x[13];
nmpcVariables.x[14] += nmpcWorkspace.x[14];
nmpcVariables.x[15] += nmpcWorkspace.x[15];

nmpcVariables.u[0] += nmpcWorkspace.x[16];
nmpcVariables.u[1] += nmpcWorkspace.x[17];
nmpcVariables.u[2] += nmpcWorkspace.x[18];
nmpcVariables.u[3] += nmpcWorkspace.x[19];
nmpcVariables.u[4] += nmpcWorkspace.x[20];
nmpcVariables.u[5] += nmpcWorkspace.x[21];
nmpcVariables.u[6] += nmpcWorkspace.x[22];
nmpcVariables.u[7] += nmpcWorkspace.x[23];
nmpcVariables.u[8] += nmpcWorkspace.x[24];
nmpcVariables.u[9] += nmpcWorkspace.x[25];
nmpcVariables.u[10] += nmpcWorkspace.x[26];
nmpcVariables.u[11] += nmpcWorkspace.x[27];
nmpcVariables.u[12] += nmpcWorkspace.x[28];
nmpcVariables.u[13] += nmpcWorkspace.x[29];
nmpcVariables.u[14] += nmpcWorkspace.x[30];
nmpcVariables.u[15] += nmpcWorkspace.x[31];
nmpcVariables.u[16] += nmpcWorkspace.x[32];
nmpcVariables.u[17] += nmpcWorkspace.x[33];
nmpcVariables.u[18] += nmpcWorkspace.x[34];
nmpcVariables.u[19] += nmpcWorkspace.x[35];
nmpcVariables.u[20] += nmpcWorkspace.x[36];
nmpcVariables.u[21] += nmpcWorkspace.x[37];
nmpcVariables.u[22] += nmpcWorkspace.x[38];
nmpcVariables.u[23] += nmpcWorkspace.x[39];
nmpcVariables.u[24] += nmpcWorkspace.x[40];
nmpcVariables.u[25] += nmpcWorkspace.x[41];
nmpcVariables.u[26] += nmpcWorkspace.x[42];
nmpcVariables.u[27] += nmpcWorkspace.x[43];
nmpcVariables.u[28] += nmpcWorkspace.x[44];
nmpcVariables.u[29] += nmpcWorkspace.x[45];
nmpcVariables.u[30] += nmpcWorkspace.x[46];
nmpcVariables.u[31] += nmpcWorkspace.x[47];
nmpcVariables.u[32] += nmpcWorkspace.x[48];
nmpcVariables.u[33] += nmpcWorkspace.x[49];
nmpcVariables.u[34] += nmpcWorkspace.x[50];
nmpcVariables.u[35] += nmpcWorkspace.x[51];
nmpcVariables.u[36] += nmpcWorkspace.x[52];
nmpcVariables.u[37] += nmpcWorkspace.x[53];
nmpcVariables.u[38] += nmpcWorkspace.x[54];
nmpcVariables.u[39] += nmpcWorkspace.x[55];
nmpcVariables.u[40] += nmpcWorkspace.x[56];
nmpcVariables.u[41] += nmpcWorkspace.x[57];
nmpcVariables.u[42] += nmpcWorkspace.x[58];
nmpcVariables.u[43] += nmpcWorkspace.x[59];
nmpcVariables.u[44] += nmpcWorkspace.x[60];
nmpcVariables.u[45] += nmpcWorkspace.x[61];
nmpcVariables.u[46] += nmpcWorkspace.x[62];
nmpcVariables.u[47] += nmpcWorkspace.x[63];
nmpcVariables.u[48] += nmpcWorkspace.x[64];
nmpcVariables.u[49] += nmpcWorkspace.x[65];
nmpcVariables.u[50] += nmpcWorkspace.x[66];
nmpcVariables.u[51] += nmpcWorkspace.x[67];
nmpcVariables.u[52] += nmpcWorkspace.x[68];
nmpcVariables.u[53] += nmpcWorkspace.x[69];
nmpcVariables.u[54] += nmpcWorkspace.x[70];
nmpcVariables.u[55] += nmpcWorkspace.x[71];
nmpcVariables.u[56] += nmpcWorkspace.x[72];
nmpcVariables.u[57] += nmpcWorkspace.x[73];
nmpcVariables.u[58] += nmpcWorkspace.x[74];
nmpcVariables.u[59] += nmpcWorkspace.x[75];
nmpcVariables.u[60] += nmpcWorkspace.x[76];
nmpcVariables.u[61] += nmpcWorkspace.x[77];
nmpcVariables.u[62] += nmpcWorkspace.x[78];
nmpcVariables.u[63] += nmpcWorkspace.x[79];
nmpcVariables.u[64] += nmpcWorkspace.x[80];
nmpcVariables.u[65] += nmpcWorkspace.x[81];
nmpcVariables.u[66] += nmpcWorkspace.x[82];
nmpcVariables.u[67] += nmpcWorkspace.x[83];
nmpcVariables.u[68] += nmpcWorkspace.x[84];
nmpcVariables.u[69] += nmpcWorkspace.x[85];
nmpcVariables.u[70] += nmpcWorkspace.x[86];
nmpcVariables.u[71] += nmpcWorkspace.x[87];
nmpcVariables.u[72] += nmpcWorkspace.x[88];
nmpcVariables.u[73] += nmpcWorkspace.x[89];
nmpcVariables.u[74] += nmpcWorkspace.x[90];
nmpcVariables.u[75] += nmpcWorkspace.x[91];
nmpcVariables.u[76] += nmpcWorkspace.x[92];
nmpcVariables.u[77] += nmpcWorkspace.x[93];
nmpcVariables.u[78] += nmpcWorkspace.x[94];
nmpcVariables.u[79] += nmpcWorkspace.x[95];
nmpcVariables.u[80] += nmpcWorkspace.x[96];
nmpcVariables.u[81] += nmpcWorkspace.x[97];
nmpcVariables.u[82] += nmpcWorkspace.x[98];
nmpcVariables.u[83] += nmpcWorkspace.x[99];
nmpcVariables.u[84] += nmpcWorkspace.x[100];
nmpcVariables.u[85] += nmpcWorkspace.x[101];
nmpcVariables.u[86] += nmpcWorkspace.x[102];
nmpcVariables.u[87] += nmpcWorkspace.x[103];
nmpcVariables.u[88] += nmpcWorkspace.x[104];
nmpcVariables.u[89] += nmpcWorkspace.x[105];
nmpcVariables.u[90] += nmpcWorkspace.x[106];
nmpcVariables.u[91] += nmpcWorkspace.x[107];
nmpcVariables.u[92] += nmpcWorkspace.x[108];
nmpcVariables.u[93] += nmpcWorkspace.x[109];
nmpcVariables.u[94] += nmpcWorkspace.x[110];
nmpcVariables.u[95] += nmpcWorkspace.x[111];
nmpcVariables.u[96] += nmpcWorkspace.x[112];
nmpcVariables.u[97] += nmpcWorkspace.x[113];
nmpcVariables.u[98] += nmpcWorkspace.x[114];
nmpcVariables.u[99] += nmpcWorkspace.x[115];
nmpcVariables.u[100] += nmpcWorkspace.x[116];
nmpcVariables.u[101] += nmpcWorkspace.x[117];
nmpcVariables.u[102] += nmpcWorkspace.x[118];
nmpcVariables.u[103] += nmpcWorkspace.x[119];
nmpcVariables.u[104] += nmpcWorkspace.x[120];
nmpcVariables.u[105] += nmpcWorkspace.x[121];
nmpcVariables.u[106] += nmpcWorkspace.x[122];
nmpcVariables.u[107] += nmpcWorkspace.x[123];
nmpcVariables.u[108] += nmpcWorkspace.x[124];
nmpcVariables.u[109] += nmpcWorkspace.x[125];
nmpcVariables.u[110] += nmpcWorkspace.x[126];
nmpcVariables.u[111] += nmpcWorkspace.x[127];
nmpcVariables.u[112] += nmpcWorkspace.x[128];
nmpcVariables.u[113] += nmpcWorkspace.x[129];
nmpcVariables.u[114] += nmpcWorkspace.x[130];
nmpcVariables.u[115] += nmpcWorkspace.x[131];
nmpcVariables.u[116] += nmpcWorkspace.x[132];
nmpcVariables.u[117] += nmpcWorkspace.x[133];
nmpcVariables.u[118] += nmpcWorkspace.x[134];
nmpcVariables.u[119] += nmpcWorkspace.x[135];

for (lRun1 = 0; lRun1 < 480; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 16; ++lRun3)
{
t += + nmpcWorkspace.evGx[(lRun1 * 16) + (lRun3)]*nmpcWorkspace.x[(lRun3) + (lRun2)];
}
nmpcVariables.x[(lRun1 + 16) + (lRun2)] += t + nmpcWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 64 ]), &(nmpcWorkspace.x[ lRun2 * 4 + 16 ]), &(nmpcVariables.x[ lRun1 * 16 + 16 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 16];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 16 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 16 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 16 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 16 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 16 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[index * 16 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[index * 16 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[index * 16 + 8];
nmpcWorkspace.state[9] = nmpcVariables.x[index * 16 + 9];
nmpcWorkspace.state[10] = nmpcVariables.x[index * 16 + 10];
nmpcWorkspace.state[11] = nmpcVariables.x[index * 16 + 11];
nmpcWorkspace.state[12] = nmpcVariables.x[index * 16 + 12];
nmpcWorkspace.state[13] = nmpcVariables.x[index * 16 + 13];
nmpcWorkspace.state[14] = nmpcVariables.x[index * 16 + 14];
nmpcWorkspace.state[15] = nmpcVariables.x[index * 16 + 15];
nmpcWorkspace.state[336] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[337] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[338] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[339] = nmpcVariables.u[index * 4 + 3];
nmpcWorkspace.state[340] = nmpcVariables.od[index * 9];
nmpcWorkspace.state[341] = nmpcVariables.od[index * 9 + 1];
nmpcWorkspace.state[342] = nmpcVariables.od[index * 9 + 2];
nmpcWorkspace.state[343] = nmpcVariables.od[index * 9 + 3];
nmpcWorkspace.state[344] = nmpcVariables.od[index * 9 + 4];
nmpcWorkspace.state[345] = nmpcVariables.od[index * 9 + 5];
nmpcWorkspace.state[346] = nmpcVariables.od[index * 9 + 6];
nmpcWorkspace.state[347] = nmpcVariables.od[index * 9 + 7];
nmpcWorkspace.state[348] = nmpcVariables.od[index * 9 + 8];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 16 + 16] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 16 + 17] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 16 + 18] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 16 + 19] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 16 + 20] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 16 + 21] = nmpcWorkspace.state[5];
nmpcVariables.x[index * 16 + 22] = nmpcWorkspace.state[6];
nmpcVariables.x[index * 16 + 23] = nmpcWorkspace.state[7];
nmpcVariables.x[index * 16 + 24] = nmpcWorkspace.state[8];
nmpcVariables.x[index * 16 + 25] = nmpcWorkspace.state[9];
nmpcVariables.x[index * 16 + 26] = nmpcWorkspace.state[10];
nmpcVariables.x[index * 16 + 27] = nmpcWorkspace.state[11];
nmpcVariables.x[index * 16 + 28] = nmpcWorkspace.state[12];
nmpcVariables.x[index * 16 + 29] = nmpcWorkspace.state[13];
nmpcVariables.x[index * 16 + 30] = nmpcWorkspace.state[14];
nmpcVariables.x[index * 16 + 31] = nmpcWorkspace.state[15];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 16] = nmpcVariables.x[index * 16 + 16];
nmpcVariables.x[index * 16 + 1] = nmpcVariables.x[index * 16 + 17];
nmpcVariables.x[index * 16 + 2] = nmpcVariables.x[index * 16 + 18];
nmpcVariables.x[index * 16 + 3] = nmpcVariables.x[index * 16 + 19];
nmpcVariables.x[index * 16 + 4] = nmpcVariables.x[index * 16 + 20];
nmpcVariables.x[index * 16 + 5] = nmpcVariables.x[index * 16 + 21];
nmpcVariables.x[index * 16 + 6] = nmpcVariables.x[index * 16 + 22];
nmpcVariables.x[index * 16 + 7] = nmpcVariables.x[index * 16 + 23];
nmpcVariables.x[index * 16 + 8] = nmpcVariables.x[index * 16 + 24];
nmpcVariables.x[index * 16 + 9] = nmpcVariables.x[index * 16 + 25];
nmpcVariables.x[index * 16 + 10] = nmpcVariables.x[index * 16 + 26];
nmpcVariables.x[index * 16 + 11] = nmpcVariables.x[index * 16 + 27];
nmpcVariables.x[index * 16 + 12] = nmpcVariables.x[index * 16 + 28];
nmpcVariables.x[index * 16 + 13] = nmpcVariables.x[index * 16 + 29];
nmpcVariables.x[index * 16 + 14] = nmpcVariables.x[index * 16 + 30];
nmpcVariables.x[index * 16 + 15] = nmpcVariables.x[index * 16 + 31];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[480] = xEnd[0];
nmpcVariables.x[481] = xEnd[1];
nmpcVariables.x[482] = xEnd[2];
nmpcVariables.x[483] = xEnd[3];
nmpcVariables.x[484] = xEnd[4];
nmpcVariables.x[485] = xEnd[5];
nmpcVariables.x[486] = xEnd[6];
nmpcVariables.x[487] = xEnd[7];
nmpcVariables.x[488] = xEnd[8];
nmpcVariables.x[489] = xEnd[9];
nmpcVariables.x[490] = xEnd[10];
nmpcVariables.x[491] = xEnd[11];
nmpcVariables.x[492] = xEnd[12];
nmpcVariables.x[493] = xEnd[13];
nmpcVariables.x[494] = xEnd[14];
nmpcVariables.x[495] = xEnd[15];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[480];
nmpcWorkspace.state[1] = nmpcVariables.x[481];
nmpcWorkspace.state[2] = nmpcVariables.x[482];
nmpcWorkspace.state[3] = nmpcVariables.x[483];
nmpcWorkspace.state[4] = nmpcVariables.x[484];
nmpcWorkspace.state[5] = nmpcVariables.x[485];
nmpcWorkspace.state[6] = nmpcVariables.x[486];
nmpcWorkspace.state[7] = nmpcVariables.x[487];
nmpcWorkspace.state[8] = nmpcVariables.x[488];
nmpcWorkspace.state[9] = nmpcVariables.x[489];
nmpcWorkspace.state[10] = nmpcVariables.x[490];
nmpcWorkspace.state[11] = nmpcVariables.x[491];
nmpcWorkspace.state[12] = nmpcVariables.x[492];
nmpcWorkspace.state[13] = nmpcVariables.x[493];
nmpcWorkspace.state[14] = nmpcVariables.x[494];
nmpcWorkspace.state[15] = nmpcVariables.x[495];
if (uEnd != 0)
{
nmpcWorkspace.state[336] = uEnd[0];
nmpcWorkspace.state[337] = uEnd[1];
nmpcWorkspace.state[338] = uEnd[2];
nmpcWorkspace.state[339] = uEnd[3];
}
else
{
nmpcWorkspace.state[336] = nmpcVariables.u[116];
nmpcWorkspace.state[337] = nmpcVariables.u[117];
nmpcWorkspace.state[338] = nmpcVariables.u[118];
nmpcWorkspace.state[339] = nmpcVariables.u[119];
}
nmpcWorkspace.state[340] = nmpcVariables.od[270];
nmpcWorkspace.state[341] = nmpcVariables.od[271];
nmpcWorkspace.state[342] = nmpcVariables.od[272];
nmpcWorkspace.state[343] = nmpcVariables.od[273];
nmpcWorkspace.state[344] = nmpcVariables.od[274];
nmpcWorkspace.state[345] = nmpcVariables.od[275];
nmpcWorkspace.state[346] = nmpcVariables.od[276];
nmpcWorkspace.state[347] = nmpcVariables.od[277];
nmpcWorkspace.state[348] = nmpcVariables.od[278];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[480] = nmpcWorkspace.state[0];
nmpcVariables.x[481] = nmpcWorkspace.state[1];
nmpcVariables.x[482] = nmpcWorkspace.state[2];
nmpcVariables.x[483] = nmpcWorkspace.state[3];
nmpcVariables.x[484] = nmpcWorkspace.state[4];
nmpcVariables.x[485] = nmpcWorkspace.state[5];
nmpcVariables.x[486] = nmpcWorkspace.state[6];
nmpcVariables.x[487] = nmpcWorkspace.state[7];
nmpcVariables.x[488] = nmpcWorkspace.state[8];
nmpcVariables.x[489] = nmpcWorkspace.state[9];
nmpcVariables.x[490] = nmpcWorkspace.state[10];
nmpcVariables.x[491] = nmpcWorkspace.state[11];
nmpcVariables.x[492] = nmpcWorkspace.state[12];
nmpcVariables.x[493] = nmpcWorkspace.state[13];
nmpcVariables.x[494] = nmpcWorkspace.state[14];
nmpcVariables.x[495] = nmpcWorkspace.state[15];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[116] = uEnd[0];
nmpcVariables.u[117] = uEnd[1];
nmpcVariables.u[118] = uEnd[2];
nmpcVariables.u[119] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119] + nmpcWorkspace.g[120]*nmpcWorkspace.x[120] + nmpcWorkspace.g[121]*nmpcWorkspace.x[121] + nmpcWorkspace.g[122]*nmpcWorkspace.x[122] + nmpcWorkspace.g[123]*nmpcWorkspace.x[123] + nmpcWorkspace.g[124]*nmpcWorkspace.x[124] + nmpcWorkspace.g[125]*nmpcWorkspace.x[125] + nmpcWorkspace.g[126]*nmpcWorkspace.x[126] + nmpcWorkspace.g[127]*nmpcWorkspace.x[127] + nmpcWorkspace.g[128]*nmpcWorkspace.x[128] + nmpcWorkspace.g[129]*nmpcWorkspace.x[129] + nmpcWorkspace.g[130]*nmpcWorkspace.x[130] + nmpcWorkspace.g[131]*nmpcWorkspace.x[131] + nmpcWorkspace.g[132]*nmpcWorkspace.x[132] + nmpcWorkspace.g[133]*nmpcWorkspace.x[133] + nmpcWorkspace.g[134]*nmpcWorkspace.x[134] + nmpcWorkspace.g[135]*nmpcWorkspace.x[135];
kkt = fabs( kkt );
for (index = 0; index < 136; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 17 */
real_t tmpDy[ 17 ];

/** Row vector of size: 10 */
real_t tmpDyN[ 10 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 16];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 16 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 16 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 16 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 16 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 16 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[lRun1 * 16 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[lRun1 * 16 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[lRun1 * 16 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[lRun1 * 16 + 9];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[lRun1 * 16 + 10];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[lRun1 * 16 + 11];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[lRun1 * 16 + 12];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[lRun1 * 16 + 13];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[lRun1 * 16 + 14];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[lRun1 * 16 + 15];
nmpcWorkspace.objValueIn[16] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[17] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[18] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[19] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[lRun1 * 9];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[lRun1 * 9 + 1];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[lRun1 * 9 + 2];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[lRun1 * 9 + 3];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[lRun1 * 9 + 4];
nmpcWorkspace.objValueIn[25] = nmpcVariables.od[lRun1 * 9 + 5];
nmpcWorkspace.objValueIn[26] = nmpcVariables.od[lRun1 * 9 + 6];
nmpcWorkspace.objValueIn[27] = nmpcVariables.od[lRun1 * 9 + 7];
nmpcWorkspace.objValueIn[28] = nmpcVariables.od[lRun1 * 9 + 8];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 17] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 17];
nmpcWorkspace.Dy[lRun1 * 17 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 17 + 1];
nmpcWorkspace.Dy[lRun1 * 17 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 17 + 2];
nmpcWorkspace.Dy[lRun1 * 17 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 17 + 3];
nmpcWorkspace.Dy[lRun1 * 17 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 17 + 4];
nmpcWorkspace.Dy[lRun1 * 17 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 17 + 5];
nmpcWorkspace.Dy[lRun1 * 17 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 17 + 6];
nmpcWorkspace.Dy[lRun1 * 17 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 17 + 7];
nmpcWorkspace.Dy[lRun1 * 17 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 17 + 8];
nmpcWorkspace.Dy[lRun1 * 17 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 17 + 9];
nmpcWorkspace.Dy[lRun1 * 17 + 10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.y[lRun1 * 17 + 10];
nmpcWorkspace.Dy[lRun1 * 17 + 11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.y[lRun1 * 17 + 11];
nmpcWorkspace.Dy[lRun1 * 17 + 12] = nmpcWorkspace.objValueOut[12] - nmpcVariables.y[lRun1 * 17 + 12];
nmpcWorkspace.Dy[lRun1 * 17 + 13] = nmpcWorkspace.objValueOut[13] - nmpcVariables.y[lRun1 * 17 + 13];
nmpcWorkspace.Dy[lRun1 * 17 + 14] = nmpcWorkspace.objValueOut[14] - nmpcVariables.y[lRun1 * 17 + 14];
nmpcWorkspace.Dy[lRun1 * 17 + 15] = nmpcWorkspace.objValueOut[15] - nmpcVariables.y[lRun1 * 17 + 15];
nmpcWorkspace.Dy[lRun1 * 17 + 16] = nmpcWorkspace.objValueOut[16] - nmpcVariables.y[lRun1 * 17 + 16];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[480];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[481];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[482];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[483];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[484];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[485];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[486];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[487];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[488];
nmpcWorkspace.objValueIn[9] = nmpcVariables.x[489];
nmpcWorkspace.objValueIn[10] = nmpcVariables.x[490];
nmpcWorkspace.objValueIn[11] = nmpcVariables.x[491];
nmpcWorkspace.objValueIn[12] = nmpcVariables.x[492];
nmpcWorkspace.objValueIn[13] = nmpcVariables.x[493];
nmpcWorkspace.objValueIn[14] = nmpcVariables.x[494];
nmpcWorkspace.objValueIn[15] = nmpcVariables.x[495];
nmpcWorkspace.objValueIn[16] = nmpcVariables.od[270];
nmpcWorkspace.objValueIn[17] = nmpcVariables.od[271];
nmpcWorkspace.objValueIn[18] = nmpcVariables.od[272];
nmpcWorkspace.objValueIn[19] = nmpcVariables.od[273];
nmpcWorkspace.objValueIn[20] = nmpcVariables.od[274];
nmpcWorkspace.objValueIn[21] = nmpcVariables.od[275];
nmpcWorkspace.objValueIn[22] = nmpcVariables.od[276];
nmpcWorkspace.objValueIn[23] = nmpcVariables.od[277];
nmpcWorkspace.objValueIn[24] = nmpcVariables.od[278];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.yN[7];
nmpcWorkspace.DyN[8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.yN[8];
nmpcWorkspace.DyN[9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.yN[9];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 17]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 17 + 1]*nmpcVariables.W[18];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 17 + 2]*nmpcVariables.W[36];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 17 + 3]*nmpcVariables.W[54];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 17 + 4]*nmpcVariables.W[72];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 17 + 5]*nmpcVariables.W[90];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 17 + 6]*nmpcVariables.W[108];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 17 + 7]*nmpcVariables.W[126];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 17 + 8]*nmpcVariables.W[144];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 17 + 9]*nmpcVariables.W[162];
tmpDy[10] = + nmpcWorkspace.Dy[lRun1 * 17 + 10]*nmpcVariables.W[180];
tmpDy[11] = + nmpcWorkspace.Dy[lRun1 * 17 + 11]*nmpcVariables.W[198];
tmpDy[12] = + nmpcWorkspace.Dy[lRun1 * 17 + 12]*nmpcVariables.W[216];
tmpDy[13] = + nmpcWorkspace.Dy[lRun1 * 17 + 13]*nmpcVariables.W[234];
tmpDy[14] = + nmpcWorkspace.Dy[lRun1 * 17 + 14]*nmpcVariables.W[252];
tmpDy[15] = + nmpcWorkspace.Dy[lRun1 * 17 + 15]*nmpcVariables.W[270];
tmpDy[16] = + nmpcWorkspace.Dy[lRun1 * 17 + 16]*nmpcVariables.W[288];
objVal += + nmpcWorkspace.Dy[lRun1 * 17]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 17 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 17 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 17 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 17 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 17 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 17 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 17 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 17 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 17 + 9]*tmpDy[9] + nmpcWorkspace.Dy[lRun1 * 17 + 10]*tmpDy[10] + nmpcWorkspace.Dy[lRun1 * 17 + 11]*tmpDy[11] + nmpcWorkspace.Dy[lRun1 * 17 + 12]*tmpDy[12] + nmpcWorkspace.Dy[lRun1 * 17 + 13]*tmpDy[13] + nmpcWorkspace.Dy[lRun1 * 17 + 14]*tmpDy[14] + nmpcWorkspace.Dy[lRun1 * 17 + 15]*tmpDy[15] + nmpcWorkspace.Dy[lRun1 * 17 + 16]*tmpDy[16];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[11];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[22];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[33];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[44];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[55];
tmpDyN[6] = + nmpcWorkspace.DyN[6]*nmpcVariables.WN[66];
tmpDyN[7] = + nmpcWorkspace.DyN[7]*nmpcVariables.WN[77];
tmpDyN[8] = + nmpcWorkspace.DyN[8]*nmpcVariables.WN[88];
tmpDyN[9] = + nmpcWorkspace.DyN[9]*nmpcVariables.WN[99];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5] + nmpcWorkspace.DyN[6]*tmpDyN[6] + nmpcWorkspace.DyN[7]*tmpDyN[7] + nmpcWorkspace.DyN[8]*tmpDyN[8] + nmpcWorkspace.DyN[9]*tmpDyN[9];

objVal *= 0.5;
return objVal;
}

