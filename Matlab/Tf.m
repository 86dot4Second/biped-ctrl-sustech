function Tf = Tf(pm)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   ������ת�����󣺵ڰ˽�

R = pm(1:3,1:3);
p = pm(1:3,4); 

Tf = [   
    R        zeros(3,3)  
    C3(p)*R  R          ];
end

