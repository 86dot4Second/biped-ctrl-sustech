function Tv = Tv(pm)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �ٶ�����ת�����󣺵ڰ˽�

R = pm(1:3,1:3);
p = pm(1:3,4); 

Tv = [   
    R           C3(p)*R  
    zeros(3,3)  R          ];
end

