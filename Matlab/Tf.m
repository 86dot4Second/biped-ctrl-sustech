function Tf = Tf(pm)
%UNTITLED 此处显示有关此函数的摘要
%   力螺旋转换矩阵：第八讲

R = pm(1:3,1:3);
p = pm(1:3,4); 

Tf = [   
    R        zeros(3,3)  
    C3(p)*R  R          ];
end

