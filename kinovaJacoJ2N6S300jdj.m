function [J,DJ] = kinovaJacoJ2N6S300jdj(robot,q,dq)
%KINOVAJACOJ2N6S300JDJ ´Ë´¦ÏÔÊ¾ÓÐ¹Ø´Ëº¯ÊýµÄÕªÒª
%   ´Ë´¦ÏÔÊ¾ÏêÏ¸ËµÃ÷
J = geometricJacobian(robot,[q;0;0;0],'j2n6s300_end_effector');
J = J(4:6,1:6);

delta_t = 1e-16;
q_new = q + dq * delta_t;
J_new = geometricJacobian(robot,[q_new;0;0;0],'j2n6s300_end_effector');
J_new = J_new(4:6,1:6);

DJ = (J_new - J)./delta_t;
end