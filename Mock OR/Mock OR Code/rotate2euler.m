function M = rotate2euler(x)
M(1,1) = atan2(x(3,2),x(3,3));
M(1,2) = asin(-x(3,1));
M(1,3) = atan2(x(2,1),x(1,1));
end

