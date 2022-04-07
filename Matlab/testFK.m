% 关节的位置与角度，z轴是关节的转动轴
clear;
clc;

j1_rpy = [0.0 pi/2 0.0];
j1_xyz = [0.0 0.0 0.0];

j2_rpy = [0.0 0.0 -pi/2];
j2_xyz = [0.0 -100.0 0.0];

j3_rpy = [0.0 0.0 0.0];
j3_xyz = [0.0 -154.5 0.0];

j4_rpy = [0.0 0.0 0.0];
j4_xyz = [54.7284365139 -294.1595798245 0.0];

j5_rpy = [0.0 0.0 0.0];
j5_xyz = [-114.2353940021 -680.8792513921 0.0];

j6_rpy = [0.0 0.0 0.0];
j6_xyz = [-169.6671153865 -611.4795636092 0.0];

j7_rpy = [0.0 0.0 0.0];
j7_xyz = [-87.5141498506 -777.0250740314 0.0];

j8_rpy = [0.0 0.0 0.0];
j8_xyz = [0.0 -1091.9101101572 0.0];

j9_rpy = [0.0 0.0 0.0];
j9_xyz = [-85.0 -1131.9101101572 0.0];

j10_rpy = [0.0 0.0 0.0];
j10_xyz = [-169.2487616966 -864.8854930287 0.0];

pm_j1o = [eul2rotm(j1_rpy,'ZYX'), j1_xyz'
    0,0,0,1];
j1_vso = Tv(pm_j1o)*[0;0;0;0;0;1];

pm_j2o = [eul2rotm(j2_rpy,'ZYX'), j2_xyz'
    0,0,0,1];
j2_vso = Tv(pm_j2o)*[0;0;0;0;0;1];

pm_j3o = [eul2rotm(j3_rpy,'ZYX'), j3_xyz'
    0,0,0,1];
j3_vso = Tv(pm_j3o)*[0;0;0;0;0;1];

pm_j4o = [eul2rotm(j4_rpy,'ZYX'), j4_xyz'
    0,0,0,1];
j4_vso = Tv(pm_j4o)*[0;0;0;0;0;1];

pm_j5o = [eul2rotm(j5_rpy,'ZYX'), j5_xyz'
    0,0,0,1];
j5_vso = Tv(pm_j5o)*[0;0;0;0;0;1];

pm_j6o = [eul2rotm(j6_rpy,'ZYX'), j6_xyz'
    0,0,0,1];
j6_vso = Tv(pm_j6o)*[0;0;0;0;0;1];

pm_j7o = [eul2rotm(j7_rpy,'ZYX'), j7_xyz'
    0,0,0,1];
j7_vso = Tv(pm_j7o)*[0;0;0;0;0;1];

pm_j8o = [eul2rotm(j8_rpy,'ZYX'), j8_xyz'
    0,0,0,1];
j8_vso = Tv(pm_j8o)*[0;0;0;0;0;1];

pm_j9o = [eul2rotm(j6_rpy,'ZYX'), j6_xyz'
    0,0,0,1];
j9_vso = Tv(pm_j6o)*[0;0;0;0;0;1];

pm_j10o = [eul2rotm(j10_rpy,'ZYX'), j10_xyz'
    0,0,0,1];
j10_vso = Tv(pm_j10o)*[0;0;0;0;0;1];

% end effector（末端）
ee_rpy = [0.0 0.0 0.0];
ee_xyz = [0.0 -1131.9101101572 0.0];

pm_eeo = [eul2rotm(ee_rpy,'ZYX'), ee_xyz'
    0,0,0,1];

% 输入，仿真中q3, q5正方向与实际相反
if(~exist('q','var'))
    q = [0.4,0.5,0.4,-0.65,-0.4]';
end

%求正解
Q4 = q4toq4f(q(4),j3_xyz,j4_xyz,j5_xyz,j6_xyz);
Q5 = q5toq5f(q(5),j7_xyz,j8_xyz,j9_xyz,j10_xyz);

P0 = eye(4);
P1 = P(j1_vso*q(1));
P2 = P1*P(j2_vso*q(2));
P3 = P2*P(j3_vso*q(3));
P4 = P3*P(j4_vso*q(4));
P5 = P4*P(j5_vso*Q4);
P6 = P5*P(j8_vso*Q5);

ee = P6*pm_eeo;

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
EI = sqrt(40^2+85^2);
GH = norm(j10_xyz-j7_xyz);
HI = norm(j9_xyz-j10_xyz);
q50 = acosine(EG,GH,norm(j10_xyz-j8_xyz));
EH0 = lcosine(EG,GH,q50);
EH = lcosine(EG,GH,q50-q5);
Q50 = acosine(EG,EH0,GH)+acosine(EH0,EI,HI);
Q5f = acosine(EG,EH,GH)+acosine(EH,EI,HI);
Q5 = Q5f - Q50;
end