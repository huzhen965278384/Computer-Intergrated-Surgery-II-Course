function M = quat2euler(q)
W  = q(:,1);
X  = q(:,2);
Y  = q(:,3);
Z  = q(:,4);

%M = [pitch, yaw, roll];
M = zeros(size(W,1),3);

for i = 1:size(W,1)
    M(i,1) = atan2(-2*X(i,1)*W(i,1) + 2*Y(i,1)*Z(i,1), -1+2*X(i,1)*X(i,1) + 2*Y(i,1)*Y(i,1))/pi*180; % pitch
    M(i,2) = asin(-2*W(i,1)*Y(i,1) - 2*Z(i,1)*X(i,1))/pi*180;  % raw
    M(i,3) = atan2(-2*Z(i,1)*W(i,1) + 2*X(i,1)*Y(i,1), -1+2*Y(i,1)*Y(i,1) + 2*Z(i,1)*Z(i,1))/pi*180; % roll
   
end

end

