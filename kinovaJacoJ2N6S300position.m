function temp = kinovaJacoJ2N6S300position(robot,q,index)
%KINOVAJACOJ2N6S300POSITION 此处显示有关此函数的摘要
%   此处显示详细说明
%q = [1.675;2.843;-3.216;4.187;-1.71;-2.65]; index=7;
if index == 1
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_link_1'));
elseif index == 2
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_link_2'));
elseif index == 3
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_link_3'));
elseif index == 4
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_link_4'));
elseif index == 5
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_link_5'));
elseif index == 6
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_link_6'));
else
    temp = tform2trvec(getTransform(robot,[q;0;0;0],'j2n6s300_end_effector'));
end

end

