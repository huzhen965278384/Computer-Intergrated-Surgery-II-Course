start =  readtable('/Users/zhenhu/Documents/Second_semester/CIS/Mock OR/Mock OR Data/start.csv');
%%
start = sortrows(start,1);
start = [start(:,1:3) start(:,16:19)];
start = table2array(start);

A6 = start(start(:,1)==1,:);
A1 = start(start(:,1)==2,:);
%%
time_A6 = A6(131:465,2);
W_A6 = A6(131:465,4);
X_A6 = A6(131:465,5);
Y_A6 = A6(131:465,6);
Z_A6 = A6(131:465,7);
Quat_A6 = A6(131:465,4:7);

time_A1 = A1(131:465,2);
W_A1 = A1(131:465,4);
X_A1 = A1(131:465,5);
Y_A1 = A1(131:465,6);
Z_A1 = A1(131:465,7);
Quat_A1 = A1(131:465,4:7);

%%
eulZYX_A6 = quat2eul(Quat_A6);
quat_A6 = quaternion(eulZYX_A6,'euler','ZYX','frame');
start_A6 = -quat2eul(meanrot(quat_A6),'XYZ')*180/pi

eulZYX_A1 = quat2eul(Quat_A1);
quat_A1 = quaternion(eulZYX_A1,'euler','ZYX','frame');
start_A1 = -quat2eul(meanrot(quat_A1),'XYZ')*180/pi
%%
q_A6 =  [0.80241, -0.3799,0.13374,0.44037];
q_A1 =  [0.63627, -0.77083,-0.00032067,-0.031174];
z = quatmultiply(quatconj(q_A6),q_A1)
a = 2* atan2(norm(z(2:4)),z(1))/pi*180
%%
figure(1)
plot(time_A6,W_A6)
hold on
plot(time_A6,X_A6)
plot(time_A6,Y_A6)
plot(time_A6,Z_A6)

plot(time_A1,W_A1)
plot(time_A1,X_A1)
plot(time_A1,Y_A1)
plot(time_A1,Z_A1)

% 1.0411 * Measured_{A6} - 2.1207
% 
% Calibrated_{1A} = 1.0295 * Measured_{1A} - 1.7564