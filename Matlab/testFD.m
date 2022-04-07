%% Dynamic of right leg

cm=[
1,0,0,0,0
0,1,0,0,0
0,0,1,0,0
0,0,0,1,0
0,0,0,0,1
0,0,0,0,0];

% 关节的位置与角度，z轴是关节的转动轴
j1_rpy = [pi/2 0.0 pi/2];
j1_xyz = [0.0 0.0 0.0];

j2_rpy = [0.0 0.0 -pi/2];
j2_xyz = [0.0 -100.0 0.0]*0.001;

j3_rpy = [0.0 0.0 0.0];
j3_xyz = [0.0 -154.5 0.0]*0.001;

j4_rpy = [0.0 0.0 0.0];
j4_xyz = [54.7284365139 -294.1595798245 0.0]*0.001;

j5_rpy = [0.0 0.0 0.0];
j5_xyz = [-114.2353940021 -680.8792513921 0.0]*0.001;

j6_rpy = [0.0 0.0 0.0];
j6_xyz = [-169.6671153865 -611.4795636092 0.0]*0.001;

j7_rpy = [0.0 0.0 0.0];
j7_xyz = [-87.5141498506 -777.0250740314 0.0]*0.001;

j8_rpy = [0.0 0.0 0.0];
j8_xyz = [0.0 -1091.9101101572 0.0]*0.001;

j9_rpy = [0.0 0.0 0.0];
j9_xyz = [-85.0 -1131.9101101572 0.0]*0.001;

j10_rpy = [0.0 0.0 0.0];
j10_xyz = [-169.2487616966 -864.8854930287 0.0]*0.001;

pm_j1o = [eul2rotm(j1_rpy,'ZYX'), j1_xyz'
    0,0,0,1];%将坐标与旋转方向转换为位姿矩阵
j1_vso = Tv(pm_j1o)*[0;0;0;0;0;1];%将位姿矩阵转换为速度螺旋转换矩阵，将关节速度螺旋转换到基座标系下表示
j1_cmo = Tf(pm_j1o)*cm;%关节约束矩阵
m1_cmo = Tf(pm_j1o)*[0;0;0;0;0;1];%将关节力螺旋转换到基座标系下表示

pm_j2o = [eul2rotm(j2_rpy,'ZYX'), j2_xyz'
    0,0,0,1];
j2_vso = Tv(pm_j2o)*[0;0;0;0;0;1];
j2_cmo = Tf(pm_j2o)*cm;
m2_cmo = Tf(pm_j2o)*[0;0;0;0;0;1];

pm_j3o = [eul2rotm(j3_rpy,'ZYX'), j3_xyz'
    0,0,0,1];
j3_vso = Tv(pm_j3o)*[0;0;0;0;0;1];
j3_cmo = Tf(pm_j3o)*cm;
m3_cmo = Tf(pm_j3o)*[0;0;0;0;0;1];

pm_j4o = [eul2rotm(j4_rpy,'ZYX'), j4_xyz'
    0,0,0,1];
j4_vso = Tv(pm_j4o)*[0;0;0;0;0;1];
j4_cmo = Tf(pm_j4o)*cm;
m4_cmo = Tf(pm_j4o)*[0;0;0;0;0;1];

pm_j5o = [eul2rotm(j5_rpy,'ZYX'), j5_xyz'
    0,0,0,1];
j5_vso = Tv(pm_j5o)*[0;0;0;0;0;1];
j5_cmo = Tf(pm_j5o)*cm;

pm_j6o = [eul2rotm(j6_rpy,'ZYX'), j6_xyz'
    0,0,0,1];
j6_vso = Tv(pm_j6o)*[0;0;0;0;0;1];
j6_cmo = Tf(pm_j6o)*cm;

pm_j7o = [eul2rotm(j7_rpy,'ZYX'), j7_xyz'
    0,0,0,1];
j7_vso = Tv(pm_j7o)*[0;0;0;0;0;1];
j7_cmo = Tf(pm_j7o)*cm;
m5_cmo = Tf(pm_j7o)*[0;0;0;0;0;1];

pm_j8o = [eul2rotm(j8_rpy,'ZYX'), j8_xyz'
    0,0,0,1];
j8_vso = Tv(pm_j8o)*[0;0;0;0;0;1];
j8_cmo = Tf(pm_j8o)*cm;

pm_j9o = [eul2rotm(j9_rpy,'ZYX'), j9_xyz'
    0,0,0,1];
j9_vso = Tv(pm_j9o)*[0;0;0;0;0;1];
j9_cmo = Tf(pm_j9o)*cm;

pm_j10o = [eul2rotm(j10_rpy,'ZYX'), j10_xyz'
    0,0,0,1];
j10_vso = Tv(pm_j10o)*[0;0;0;0;0;1];
j10_cmo = Tf(pm_j10o)*cm;

% 求出起始位置各个杆件的惯量
pm = [eye(3), [0.0500 0.0 0.0]';0,0,0,1];
I0o = Tf(pm) * [eye(3)*0.8169313258, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), [0.0 -0.0500 0.0]';0,0,0,1];
I1o = Tf(pm) * [eye(3)*0.8169313258, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), [0.0 -0.12725 0.0]';0,0,0,1];
I2o = Tf(pm) * [eye(3)*0.5329749258, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), [0.027364218257 -0.2243297899122 0.0]';0,0,0,1];
I3o = Tf(pm) * [eye(3)*1.1289713258, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), [-0.0297534787441 -0.4875194156083 0.0]';0,0,0,1];
I4o = Tf(pm) * [eye(3)*2.8265937418, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
%pm = [eye(3), [-84.8335576933 -382.9897818046 0.0]';0,0,0,1];
I5o = [eye(3)*3.2349916938, zeros(3,3);zeros(3,3), eye(3)];
pm = [eye(3), [-0.0747126901462 -0.8365725796917 0.0]';0,0,0,1];
I6o = Tf(pm) * [eye(3)*3.6023981957, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), [0.073191580981 -1.1290025457424 0.0]';0,0,0,1];
I7o = Tf(pm) * [eye(3)*3.0436699775, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
%pm = [eye(3), [-127.1243808483 -998.397801593 0.0]';0,0,0,1];
I8o = [eye(3)*1.9402753258, zeros(3,3);zeros(3,3), eye(3)];
pm = [eye(3), [-0.1283814557736 -0.82095528353 0.0]';0,0,0,1];
I9o = Tf(pm) * [eye(3)*0.9417473258, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';

% 末端
ee_rpy = [0.0 0.0 0.0];
ee_xyz = [0.0 -1131.9101101572 0.0]*0.001;

pm_eeo = [eul2rotm(ee_rpy,'ZYX'), ee_xyz'
    0,0,0,1];
%% 输入
% 仿真中q3, q5正方向与实际相反
if(~exist('q','var'))
    q = [0.4,0.5,0.4,-0.65,-0.4]';
end
if(~exist('dq','var'))
    dq = [0.1,0.1,0.1,0.1,0.1]';
end
if(~exist('ddq','var'))
    ddq = [0.1,0.1,0.1,0.1,0.1]';
end
if(~exist('qf','var'))
    qf = -[1,1,1,1,1]';
end
%% 求正解
Q4 = q4toq4f(q(4),j3_xyz,j4_xyz,j5_xyz,j6_xyz);
Q5 = q5toq5f(q(5),j7_xyz,j8_xyz,j9_xyz,j10_xyz);

P0 = eye(4);
P1 = P(j1_vso*q(1));
P2 = P1*P(j2_vso*q(2));
P3 = P2*P(j3_vso*q(3));
P4 = P3*P(j4_vso*q(4));
P5 = P4*P(j5_vso*Q4);
P6 = P5*P(j8_vso*Q5);
P7 = P5*P(j7_vso*q(5));

ee = P6*pm_eeo;
% 各关节位置测试
pm_j2 = P1*pm_j2o;
pm_j3 = P2*pm_j3o;
pm_j4 = P3*pm_j4o;
pm_j5 = P4*pm_j5o;
pm_j6 = P5*pm_j6o;
pm_j7 = P5*pm_j7o;
pm_j8 = P5*pm_j8o;
pm_j9 = P6*pm_j9o;
pm_j10 = P7*pm_j10o;
%% 计算所有杆件的速度

% 关节约束
j1_cm = j1_cmo;
j2_cm = Tf(P1) * j2_cmo;
j3_cm = Tf(P2) * j3_cmo;
j4_cm = Tf(P3) * j4_cmo;
j5_cm = Tf(P4) * j5_cmo;
j6_cm = Tf(P5) * j6_cmo;
j7_cm = Tf(P5) * j7_cmo;
j8_cm = Tf(P5) * j8_cmo;
j9_cm = Tf(P6) * j9_cmo;
j10_cm = Tf(P7) * j10_cmo;

% 驱动约束
m1_cm = m1_cmo;
m2_cm = Tf(P1) * m2_cmo;
m3_cm = Tf(P2) * m3_cmo;
m4_cm = Tf(P3) * m4_cmo;
m5_cm = Tf(P5) * m5_cmo;

% 大约束矩阵
%    Fix R1 R2 R3 r3 R4 R5 R6 R7 R8 R9 R10 M1 M2 M3 M4 M5
% GR  1  -1                                -1
% L1      1 -1                              1 -1
% L2         1 -1                              1 -1
% L3            1 -1 -1                           1 -1
% L4                  1 -1                           1
% L5               1       -1 
% L6                     1  1 -1 -1                    -1   
% L7                              1 -1
% L8                                 1  -1
% L9                           1         1              1
C=[
eye(6,6),       -j1_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),     -m1_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6),      j1_cm,     -j2_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      m1_cm,     -m2_cm, zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5),      j2_cm,     -j3_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1),      m2_cm,     -m3_cm, zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5),      j3_cm,     -j3_cm,     -j4_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1),      m3_cm,     -m4_cm, zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j4_cm,     -j5_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1),      m4_cm, zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5),      j3_cm, zeros(6,5), zeros(6,5),     -j6_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j5_cm,      j6_cm,     -j7_cm,     -j8_cm, zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1),     -m5_cm
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j8_cm,     -j9_cm, zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j9_cm,    -j10_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j7_cm, zeros(6,5), zeros(6,5),     j10_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1),     m5_cm];

cv = [zeros(61,1);dq];

v = pinv(C')*cv;

v0 = v(1:6);
v1 = v(7:12);
v2 = v(13:18);
v3 = v(19:24);
v4 = v(25:30);
v5 = v(31:36);
v6 = v(37:42);
v7 = v(43:48);
v8 = v(49:54);
v9 = v(55:60);
%% 求所有杆件加速度
dC=[
zeros(6,6),-Cf(v0)*j1_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),-Cf(v0)*m1_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), Cf(v0)*j1_cm,-Cf(v1)*j2_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v0)*m1_cm,-Cf(v1)*m2_cm, zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), Cf(v1)*j2_cm,-Cf(v2)*j3_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), Cf(v1)*m2_cm,-Cf(v2)*m3_cm, zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), Cf(v2)*j3_cm,-Cf(v2)*j3_cm,-Cf(v3)*j4_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), Cf(v2)*m3_cm,-Cf(v3)*m4_cm, zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v3)*j4_cm,-Cf(v4)*j5_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1),Cf(v3)*m4_cm, zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v2)*j3_cm, zeros(6,5), zeros(6,5),-Cf(v5)*j6_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v4)*j5_cm, Cf(v5)*j6_cm,-Cf(v6)*j7_cm,-Cf(v6)*j8_cm, zeros(6,5), zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1),-Cf(v6)*m5_cm
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v6)*j8_cm,-Cf(v8)*j9_cm, zeros(6,5), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v8)*j9_cm,-Cf(v9)*j10_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v6)*j7_cm, zeros(6,5), zeros(6,5), Cf(v9)*j10_cm, zeros(6,1), zeros(6,1), zeros(6,1), zeros(6,1), Cf(v6)*m5_cm];

ca = [zeros(61,1);ddq] - dC'*v;

% a = pinv(C')*ca;
% 
% a0 = a(1:6);
% a1 = a(7:12);
% a2 = a(13:18);
% a3 = a(19:24);
% a4 = a(25:30);
% a5 = a(31:36);
% a6 = a(37:42);
% a7 = a(43:48);
% a8 = a(49:54);
% a9 = a(55:60);
%% 动力学逆解
I0=Tf(P0) * I0o * Tf(P0)';
I1=Tf(P1) * I1o * Tf(P1)';
I2=Tf(P2) * I2o * Tf(P2)';
I3=Tf(P3) * I3o * Tf(P3)';
I4=Tf(P4) * I4o * Tf(P4)';
TF = Tf([pm_j6(1:3,1:3),(pm_j3(1:3,4)+pm_j6(1:3,4))/2;0,0,0,1]);%取两关节中点位置作为杆件质心位置
I5=TF * I5o * TF';
I6=Tf(P5) * I6o * Tf(P5)';
I7=Tf(P6) * I7o * Tf(P6)';
TF = Tf([pm_j9(1:3,1:3),(pm_j9(1:3,4)+pm_j10(1:3,4))/2;0,0,0,1]);
I8=TF * I8o * TF';
I9=Tf(P7) * I9o * Tf(P7)';

I=blkdiag(I0,I1,I2,I3,I4,I5,I6,I7,I8,I9);

g=[0,-9.8,0,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0;
f1=-I1*g + Cf(v1)*I1*v1;
f2=-I2*g + Cf(v2)*I2*v2;
f3=-I3*g + Cf(v3)*I3*v3;
f4=-I4*g + Cf(v4)*I4*v4;
f5=-I5*g + Cf(v5)*I5*v5;
f6=-I6*g + Cf(v6)*I6*v6;
f7=-I7*g + Cf(v7)*I7*v7;
f8=-I8*g + Cf(v8)*I8*v8;
f9=-I9*g + Cf(v9)*I9*v9;

fp=[f0;f1;f2;f3;f4;f5;f6;f7;f8;f9];

%ca = [zeros(61,1);ddq] - dC'*v;

A = [-I, C;C', zeros(66,66)];
b = [fp;ca];

x = pinv(A)*b;

% x包含了所有的杆件加速度和所有的约束力，现在列出五个驱动力
actuation_force = x(end-4:end);
a0 = x(1:6);
a1 = x(7:12);
a2 = x(13:18);
a3 = x(19:24);
a4 = x(25:30);
a5 = x(31:36);
a6 = x(37:42);
a7 = x(43:48);
a8 = x(49:54);
a9 = x(55:60);

% n0 = x(61:66);
% n1 = x(67:71);
% n2 = x(72:76);
% n31 = x(77:81);
% n32 = x(82:86);
% n4 = x(87:91);
% n5 = x(92:96);
% n6 = x(97:101);
% n7 = x(102:106);
% n8 = x(107:111);
% n9 = x(112:116);
% n10 = x(117:121);
%% 动力学正解
C2=[
eye(6,6),       -j1_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6),      j1_cm,     -j2_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5),      j2_cm,     -j3_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5),      j3_cm,     -j3_cm,     -j4_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j4_cm,     -j5_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5),      j3_cm, zeros(6,5), zeros(6,5),     -j6_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j5_cm,      j6_cm,     -j7_cm,     -j8_cm, zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j8_cm,     -j9_cm, zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j9_cm,    -j10_cm
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5),      j7_cm, zeros(6,5), zeros(6,5),     j10_cm];

dC2=[
zeros(6,6),-Cf(v0)*j1_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), Cf(v0)*j1_cm,-Cf(v1)*j2_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), Cf(v1)*j2_cm,-Cf(v2)*j3_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), Cf(v2)*j3_cm,-Cf(v2)*j3_cm,-Cf(v3)*j4_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v3)*j4_cm,-Cf(v4)*j5_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v2)*j3_cm, zeros(6,5), zeros(6,5),-Cf(v5)*j6_cm, zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v4)*j5_cm, Cf(v5)*j6_cm,-Cf(v6)*j7_cm,-Cf(v6)*j8_cm, zeros(6,5), zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v6)*j8_cm,-Cf(v8)*j9_cm, zeros(6,5)
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v8)*j9_cm,-Cf(v9)*j10_cm
zeros(6,6), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), zeros(6,5), Cf(v6)*j7_cm, zeros(6,5), zeros(6,5), Cf(v9)*j10_cm];

f0=-I0*g + Cf(v0)*I0*v0 - m1_cm*qf(1);
f1=-I1*g + Cf(v1)*I1*v1 + m1_cm*qf(1) - m2_cm*qf(2);
f2=-I2*g + Cf(v2)*I2*v2 + m2_cm*qf(2) - m3_cm*qf(3);
f3=-I3*g + Cf(v3)*I3*v3 + m3_cm*qf(3) - m4_cm*qf(4);
f4=-I4*g + Cf(v4)*I4*v4 + m4_cm*qf(4);
f5=-I5*g + Cf(v5)*I5*v5;
f6=-I6*g + Cf(v6)*I6*v6 - m5_cm*qf(5);
f7=-I7*g + Cf(v7)*I7*v7;
f8=-I8*g + Cf(v8)*I8*v8;
f9=-I9*g + Cf(v9)*I9*v9 + m5_cm*qf(5);

fp2=[f0;f1;f2;f3;f4;f5;f6;f7;f8;f9];

dcv2 = zeros(61,1);
ca2 = -dC2'*v + dcv2;

A = [-I, C2;C2', zeros(61,61)];
b = [fp2;ca2];

x = pinv(A)*b;

aj1 = x(7:12)-x(1:6) - Cv(v0)*v1;
aj2 = x(13:18)-x(7:12) - Cv(v1)*v2;
aj3 = x(19:24)-x(13:18) - Cv(v2)*v3;
aj4 = x(25:30)-x(19:24) - Cv(v3)*v4;
aj5 = x(55:60)-x(37:42) - Cv(v6)*v9;% a9-a6-v6×v9

input_accleration = [norm(aj1(4:6));norm(aj2(4:6));norm(aj3(4:6));norm(aj4(4:6));norm(aj5(4:6))];
%% 转换关系
function Q4 = q4toq4f(q4,j3_xyz,j4_xyz,j5_xyz,j6_xyz)
AB = norm(j6_xyz-j3_xyz);
BC = norm(j5_xyz-j6_xyz);
CD = norm(j5_xyz-j4_xyz);
AD = norm(j4_xyz-j3_xyz);
q40 = acosine(AD,CD,norm(j5_xyz-j3_xyz));
AC0 = lcosine(AD,CD,q40);
AC = lcosine(AD,CD,q40+q4);
Q40 = acosine(BC,AC0,AB)+acosine(AC0,CD,AD);
Q4f = acosine(BC,AC,AB)+acosine(AC,CD,AD);
Q4 = Q4f - Q40;
end

function Q5 = q5toq5f(q5,j7_xyz,j8_xyz,j9_xyz,j10_xyz)
EG = norm(j8_xyz-j7_xyz);
EI = sqrt(0.04^2+0.085^2);
GH = norm(j10_xyz-j7_xyz);
HI = norm(j9_xyz-j10_xyz);
q50 = acosine(EG,GH,norm(j10_xyz-j8_xyz));
EH0 = lcosine(EG,GH,q50);
EH = lcosine(EG,GH,q50-q5);
Q50 = acosine(EG,EH0,GH)+acosine(EH0,EI,HI);
Q5f = acosine(EG,EH,GH)+acosine(EH,EI,HI);
Q5 = Q5f - Q50;
end