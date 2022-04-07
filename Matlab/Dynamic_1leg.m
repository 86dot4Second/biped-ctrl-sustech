%% Dynamic of right leg

cm=[
1,0,0,0,0
0,1,0,0,0
0,0,1,0,0
0,0,0,1,0
0,0,0,0,1
0,0,0,0,0];

% 关节的位置与角度，z轴是关节的转动轴
j1_rpy = [0.0 pi/2 0.0];
j1_xyz = [-0.112 0.0 0.2025];

j2_rpy = [0.0 0.0 -pi/2];
j2_xyz = [-0.112 0.0 0.2025];

j3_rpy = [0.0 0.0 0.0];
j3_xyz = [-0.112 -0.1545 0.2025];

j4_rpy = [0.0 0.0 0.0];
j4_xyz = [-0.0414 -0.28685 0.2025];

j5_rpy = [0.0 0.0 0.0];
j5_xyz = [-0.22024 -0.6691 0.2025];

j6_rpy = [0.0 0.0 0.0];
j6_xyz = [-0.39374 -0.63953 0.2025];

j7_rpy = [0.0 0.0 0.0];
j7_xyz = [-0.17054 -0.75588 0.2025];

j8_rpy = [0.0 0.0 0.0];
j8_xyz = [-0.02945 -1.05067 0.2025];

j9_rpy = [0.0 0.0 0.0];
j9_xyz = [-0.11445 -1.09067 0.2025];

j10_rpy = [0.0 0.0 0.0];
j10_xyz = [-0.21924 -0.79893 0.2025];

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
m7_cmo = Tf(pm_j7o)*[0;0;0;0;0;1];

pm_j8o = [eul2rotm(j8_rpy,'ZYX'), j8_xyz'
    0,0,0,1];
j8_vso = Tv(pm_j8o)*[0;0;0;0;0;1];
j8_cmo = Tf(pm_j8o)*cm;

pm_j9o = [eul2rotm(j6_rpy,'ZYX'), j6_xyz'
    0,0,0,1];
j9_vso = Tv(pm_j6o)*[0;0;0;0;0;1];
j9_cmo = Tf(pm_j6o)*cm;

pm_j10o = [eul2rotm(j10_rpy,'ZYX'), j10_xyz'
    0,0,0,1];
j10_vso = Tv(pm_j10o)*[0;0;0;0;0;1];
j10_cmo = Tf(pm_j10o)*cm;

% 求出起始位置各个杆件的惯量
% 注意杆件编号与关节编号不一定相同
I0o = [eye(3)*4.188, zeros(3,3);zeros(3,3), eye(3)];
pm = [eye(3), j1_xyz';0,0,0,1];
I1o = Tf(pm) * [eye(3)*1.44295, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j2_xyz' + [0.2125 -0.024201 0.0]';0,0,0,1];
I2o = Tf(pm) * [eye(3)*1.51937, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j3_xyz' + [0.110949 0.0 0.01634]';0,0,0,1];
I3o = Tf(pm) * [eye(3)*0.76742, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j4_xyz';0,0,0,1];
I4o = Tf(pm) * [eye(3)*0.90423, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j5_xyz';0,0,0,1];
I5o = Tf(pm) * [eye(3)*0.11386, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j6_xyz';0,0,0,1];
I6o = Tf(pm) * [eye(3)*0.61403, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j6_xyz';0,0,0,1];
I7o = Tf(pm) * [eye(3)*0.58479, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j6_xyz';0,0,0,1];
I8o = Tf(pm) * [eye(3)*0.069, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
pm = [eye(3), j6_xyz';0,0,0,1];
I9o = Tf(pm) * [eye(3)*0.03185, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';

% end effector（末端）
ee_rpy = [0.0 0.0 0.0];
ee_xyz = [-0.03 -1.09068 0.2025];

pm_eeo = [eul2rotm(ee_rpy,'ZYX'), ee_xyz'
    0,0,0,1];
%% input
if(~exist('q','var'))
    q = [0.1,0.2,0.3,0.4,0.5]';
end
if(~exist('dq','var'))
    dq = [0.1,0.2,0.3,0.4,0.5]';
end
if(~exist('ddq','var'))
    ddq =  [0.1,0.2,0.3,0.4,0.5]';
end
if(~exist('qf','var'))
    qf = -[1.36817604774081,-43.5520354075637,-7.09576204454976, 3.28091697296027,1.20096689859854]';
end
%% 位置正解
Q4 = q4toq4f(q(4));
Q5 = q5toq5f(q(4));

P0 = eye(4);
P1 = P(j1_vso*q(1));
P2 = P1*P(j2_vso*q(2));
P3 = P2*P(j3_vso*q(3));
P4 = P3*P(j4_vso*q(4));
P5 = P4*P(j5_vso*Q4);
P6 = P5*P(j8_vso*Q5);

ee = P6*pm_eeo;
%% 转换关系
function Q4 = q4toq4f(q4)
AB = 560.92113834;
BC = 176.00866644;
CD = 422.02;
AD = 150;
q40 = 126.85/180*pi;
AC0 = lcosine(AD,CD,q40);
AC = lcosine(AD,CD,q40+q4);
Q40 = acosine(BC,AC0,AB)+acosine(AC0,CD,AD);
Q4f = acosine(BC,AC,AB)+acosine(AC,CD,AD);
Q4 = Q4f - Q40;
end

function Q5 = q5toq5f(q5)
EG = 326.82;
EI = sqrt(40^2+85^2);
GH = 65;
HI = 309.99078879;
q50 = 74.1/180*pi;
EH0 = lcosine(EG,GH,q50);
EH = lcosine(EG,GH,q50-q5);
Q50 = acosine(EG,EH0,GH)+acosine(EH0,EI,HI);
Q5f = acosine(EG,EH,GH)+acosine(EH,EI,HI);
Q5 = Q5f - Q50;
end