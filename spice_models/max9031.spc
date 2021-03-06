*** MACROMODEL FOR MAX9031 ***
*** ----------------------------
*** Revision 1. 1/2014
*** Revision 2. 2/2014
*** ----------------------------
*******************************************
.subckt MAX9031 IN+ VSS IN- OUT VDD
X_1  IN+ IN- OUT VDD VSS CM1
.ends
*******************************************
.SUBCKT CM1   311  2  611  401  5
RINB 2 18 1000
RINA 3 19 1000
DIN1 5 18 DMOD2
DIN2 18 4 DMOD2
DIN3 5 19 DMOD2
DIN4 19 4 DMOD2
FIN1 18 5 VTEMP 0.75
FIN2 19 5 VTEMP 1.25
Ib1 3 5 9n
Ib2 2 5 7n
CIN1 2 10 1e-12
CIN2 3 10 1e-12
RD1 18 11 5e+11  
RD2 19 11 5e+11
RCM 11 10 9.975e+12
EXX 10 5 17 5 1.0
EEE 10 50 17 5 1.0
ECC 40 10 VALUE={V(4)-V(17)-0.2}
RAA 4 17 100MEG
RBB 17 5 100MEG
RSLOPE 4 5 1e+12
GPWR 4 5 26 10 9.5e-7  
EOX 120 10 31 32 0.6
RCX 120 121 1K
RDX 121 10 1K
RBX 120 122 1K
RAX 122 10 MRAX 1.0105e3
.MODEL MRAX RES (TC1=4e-6)  
RX8 40 815 10K
RY8 815 50 5K
RBA8 815 50 5K
RBB8 815 811 1K
EIN8 810 811 3 2  -1 
EVOSS 814  811a 122 121  1    
V_F   811a  811  0
RCA8 40 812  0.82K  
RCB8 40 813   0.79K 
DDA8 812 813 DDEL1
DDB8 813 812 DDEL2
CDB8 813 812 1P
RCDB8 813 812 1345
FSET8 809 50 VSENS1   1 
GSET8x   809  50   4    5    0.02e-3
****************
CCC 809 50 5P
QDN1 812 810 809 NPNX
QDN2 813 814 809 NPNX
.MODEL NPNX NPN (BF=100 RE=25)
.MODEL  DDEL1  D ( RS=0.1  TT=0.1U N=0.7 )
.MODEL DDEL2 D ( RS=0.1  TT=0.1U N=0.7 )
GDM 10 57 812 813  0.1  
ISET 10 24 1e-3
DA1 24 23 DMOD1
RBAL 23 22 1000
ESUPP 22 21 4 5 1.0
VOFF 21 10 -1.25
DA2 24 25 DMOD1
VSENS1 25 26 DC 0
RSET 26 10 1K
CSET 26 10 1e-10
FSET 10 31 VSENS1 1.0
RVOS 31 32 1K
RIB 32 33 MRIB 1K 
.MODEL MRIB RES (TC1=0.0029713)
RISC 33 34 MRISC 1K 
.MODEL MRISC RES (TC1=0)
R001 34 10 1K
ECMR 38 10 11 10 1.0
VCMX 38 39 DC 0
RCM2 41 10 1MEG
RCM1 39 41 1e6
CCM 41 10 1.59155e-10
EPSR 42 10 4 10 1.0
CDC1 43 42 10U
VPSX 43 44 DC 0
RPSR2 45 10 1MEG
RPSR1 44 45 1e6
CPSR 45 10 1.59155e-10
FTEMP 10 27 VSENS1 1.0
ETEMP 27 28 32 33 0.632  
DTA 27 10 DMOD2
DTB 28 29 DMOD2
VTEMP 29 10 DC 0
FX 10 93 VOX 1.0
DFX1 93 91 DMOD1
VFX1 91 10 DC 0
DFX2 92 93 DMOD1
VFX2 10 92 DC 0
FPX 4 10 VFX1 1.0
FNX 10 5 VFX2 1.0
DCX1 98 97 DMOD1
DCX2 95 94 DMOD1
RCX1 99 98 100
RCX2 94 99 100
VCXX 99 96 DC 0
ECMX 96 10 11 10 1.0
ECMP 40 97 26 10 -5e-1
ECMN 95 50 26 10   0.1
GOS 10 57 122 121 1
FCMR 10 57 VCMX 100
FPSR 10 57 VPSX 10
FCXX 57 10 VCXX 10
RDM 57 10 2091   
C2 57 10  6.59e-12
DLIM1 52 57 DMOD1
DLIM2 57 51 DMOD1
ELIMP 51 10 26 10 99.3
ELIMN 10 52 26 10 4.16e2  
G2 58 10 57 10 18e-5  
R2 58 10 79.7  
GO2 59 10 58 10 1e4  
RO2 59 10 1K
DCLMP2 59 40 DMOD1
DCLMP1 50 59 DMOD1
GO3 10 71 59 10 1
RO3 71 10 5 
RDN2 710 71 100
RDP 720 72 100
DDN1 73 74 DMOD1
DDN2 73 710 DMOD1
RNO 78 81 1
RPO 79 81 0.1
DDP1 75 72 DMOD1
DDP2 71 720 DMOD1
C1 58 59 1e-10
VOOP 40 76 DC 0
VOON 77 50 DC 0
QNO 76 73 78 NPN1
QNP 77 72 79 PNP1
*****************
MN1  79  350  77  77  MNMOD
E_MN1  350  77  10  71  1.0
MP1  78 360 76  76 MPMOD
E_MP1    76  360  71  10  1.0
*****************
VOX 86 6 DC 0
RNT 76 81 100MEG
RPT 81 77 1MEG
EPOS 40 74 26 10 0.0
ENEG 75 50 26 10 0.1
GSOURCE 74 73 33 34 1e-1
GSINK 72 75 33 34  3.5e-5
ROO 81 86  0.01
D00 oo 6 DMOD23
V00 oo 5 1m
Dsc1 6 601 dmod
Dsc2 602 6 dmod
Dsc3 611 601 dmod
Dsc4 602 611 dmod
Isc 601 602 45m
E400 400 5 VALUE={V(401)+0.2}
R66 6 5 10k
E311 311 3 VALUE={(V(6)-V(10))/V(10)*(-2m)}
V400 400 4 0
F400 400 5 V400 -1
G400 400 5 VALUE={-1.5333u*V(400)-27.1u}
.MODEL dmod D()
.MODEL DMOD1 D
.MODEL DMOD2 D (IS=1e-17)
.MODEL DMOD23 D (IS=1e-4)
.MODEL NPN1 NPN (BF=100 IS=1e-15)
.MODEL PNP1 PNP (BF=100 IS=1e-15)
.MODEL MNMOD  NMOS  VTO=0.5  W=40u  L=1u
.MODEL MPMOD PMOS  VTO=-0.5  W=40u  L=1u
RA 73 40 10e6
RB 72 50 10e6
RC 72 73 10e6
RD 10 57 10e6
RE 24 10 10e6
RF 93 10 10e6
E_TEST   100a   5   814  810   1.0
R_TEST  100a   5    1K
.ENDS 


* Copyright (c) 2003-2012 Maxim Integrated Products.  All Rights Reserved.