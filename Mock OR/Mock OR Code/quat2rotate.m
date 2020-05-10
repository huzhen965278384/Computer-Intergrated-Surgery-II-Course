% transfer from quaternion to rotation matrix
function M = quat2rotate(x)
q0 = x(1,1);
q1 = x(1,2);
q2 = x(1,3);
q3 = x(1,4);
M = zeros(3,3);
M(1,:) = 2*[q0^2+q1^2-0.5, q1*q2-q0*q3 q0*q2+q1*q3];
M(2,:) = 2*[q0*q3+q1*q2 q0^2+q2^2-0.5 q2*q3-q0*q1];
M(3,:) = 2*[q1*q3-q0*q2, q0*q1+q2*q3, q0^2+q3^2-0.5];
end