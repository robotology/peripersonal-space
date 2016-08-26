%torso
%script create matrix M with positions of taxels like in iCub visualization
%tool, see forearm_ini_generator.xlsx
%M-matrix, first row-positions x, second row positions y, third row
%visible=1 invisible=0

Trian1=[  %positions of triangles and their angles and if visible(1) or invisible(1)
0	0	0	0;
0	0	0	0;
0	0	0	0;
0	0	0	0;
0	0	0	0;
-50	43	2/3*pi	1;
-50	80	pi/3	1;
-50	99	0	1;
-18	80	pi/3	1;
-34	71	2/3*pi	1;
-34	52	pi	1;
-18	43	4/3*pi	1;
0	0	0	0;
0	0	0	0;
0	0	0	0;
0	0	0	0;
0	0	0	0;
-66	-13	-2/3*pi	1;
-66	-31	-pi	1;
-50	-41	-2/3*pi	1;
0	0	0	0;
0	0	0	0;
-2	24	-pi/3	1;
14	15	-2/3*pi	1;
-18	-4	-pi/3	1;
-18	15	0	1;
-34	24	pi/3	1;
-50	15	2/3*pi	1;
-50	-4	-pi/3	1;
-34	-13	-2/3*pi	1;
0	0	0	0;
0	0	0	0;
63	-13	2/3*pi	1;
47	-4	pi/3	1;
47	14	0	1;
-17	-60	pi	1;
-1	-69	2/3*pi	1;
15	-60	pi	1;
-17	-41	2/3*pi	1;
-33	-32	pi/3	1;
-1	-13	2/3*pi	1;
-1	-32	pi	1;
15	-41	4/3*pi	1;
31	-32	10/6*pi	1;
31	-13	2/3*pi	1;
15	-4	pi/3	1;
47	-41	4/3*pi	1;
63	-32	10/6*pi	1;
0	0	0	0;
47	80	pi/3	1;
47	98	0	1;
0	0	0	0;
31	24	pi/3	1;
0	0	0	0;
0	0	0	0;
0	0	0	0;
-1	71	2/3*pi	1;
-1	52	pi	1;
15	43	4/3*pi	1;
31	52	10/6*pi	1;
31	71	2/3*pi	1;
15	80	pi/3	1;
47	43	4/3*pi	1;
0	0	0	0;
]';

Tax1=[6.533,0,1; %positions of taxels in triangle
9.8,-5.66,1;
3.267,-5.66,1;
0,0,1;
-3.267,-5.66,1;
-9.8,-5.66,1;
-6.51,-3.75,0;
-6.533,0,1;
-3.267,5.66,1;
0,11.317,1;
0,7.507,0;
3.267,5.66,1;
]';


M=createTaxelPos(Trian1,Tax1);


clear M1 M2 M1p M2p Tax1 Trans Trans2 Trian1 Trian2 alpha1 alpha2;