function M = euler2rotate(euler)

M = cell(size(euler,1),1);

for i = 1: size(M,1)
    x = euler(i,1);
    y = euler(i,2);
    z = euler(i,3);
    N = zeros(3,3);
    N(:,1) = [cos(z)*cos(y),cos(z)*sin(y)*sin(x)-sin(z)*cos(x),cos(z)*sin(y)*cos(x)+sin(z)*sin(x)];
    N(:,2) = [sin(z)*cos(y),sin(z)*sin(y)*sin(x)+cos(z)*cos(x),sin(z)*sin(y)*cos(x)-cos(z)*sin(x)];
    N(:,3) = [-sin(y),cos(y)*sin(x),cos(y)*cos(x)];
    M{i} = {N};
end
end
