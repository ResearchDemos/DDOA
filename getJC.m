function J = getJC(robot,l,C,jointPos,q)
    H = zeros(6,3);
    for i = 1:l
        H(i,:) = C - jointPos(i,:);
    end
    %for j = l+1:6
    %    H(j,:) = H(l,:);
    %end
    
    R = zeros(3,3,6);
    temp = getTransform(robot,[q;0;0;0],'j2n6s300_link_1');
    R(:,:,1) = temp(1:3,1:3);
    temp = getTransform(robot,[q;0;0;0],'j2n6s300_link_2');
    R(:,:,2) = temp(1:3,1:3);
    temp = getTransform(robot,[q;0;0;0],'j2n6s300_link_3');
    R(:,:,3) = temp(1:3,1:3);
    temp = getTransform(robot,[q;0;0;0],'j2n6s300_link_4');
    R(:,:,4) = temp(1:3,1:3);
    temp = getTransform(robot,[q;0;0;0],'j2n6s300_link_5');
    R(:,:,5) = temp(1:3,1:3);
    temp = getTransform(robot,[q;0;0;0],'j2n6s300_link_6');
    R(:,:,6) = temp(1:3,1:3);
    
    e = [0;0;1];
    J = zeros(3,6);
    for k = 1:l
        J(:,k) = cross(R(:,:,k)*e,H(k,:)');
%          J(:,k) = R(:,:,k)*e;
    end
    
end