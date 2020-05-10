%% Read in the Rosbag file
bag = rosbag('/Users/zhenhu/Documents/Second_semester/CIS/Calibration/1AThird.bag');
bagselect = select(bag, 'Topic', '/imu_1A/imu'); 
msgStructs = readMessages(bagselect,'DataFormat','struct');
ts = timeseries(bagselect, 'Orientation.W', 'Orientation.X','Orientation.Y','Orientation.Z');
% % %% Plot the w,x,y,z of the quaternion in order to select the time region
% % plot(ts.Data(:,1));
% % hold on
% % plot(ts.Data(:,2));
% % plot(ts.Data(:,3));
% % plot(ts.Data(:,4));
% % legend('Orientation.W','Orientation.X','Orientation.Y','Orientation.Z')
%% take horizontal plane as reference (0 degree)
reference = ts.Data(36620:41870,:);
eulZYX = quat2eul(reference);
% ref = eul2quat(eulZYX);
quat = quaternion(eulZYX,'euler','ZYX','frame');
quatAverage = meanrot(quat);

%% next degree
first = ts.Data(34690:36270,:);
eulZYX_first = quat2eul(first);
quat_first = quaternion(eulZYX_first,'euler','ZYX','frame');
quatAverage_first = meanrot(quat_first);

%% derive the pitch, yaw, roll angles
euler = zeros(10,3);
i = 1;
%q1 =  [0.38673, -0.0086113,-0.00060235,-0.92215]; % original q1:(reference 0)
q1 = [0.036689 0.031016 -0.68204 -0.72974];
q2 =  [0.40267, 0.0040498,-0.029676,-0.91485];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
second = ts.Data(29710:34130,:);
eulZYX_second = quat2eul(second);
quat_second = quaternion(eulZYX_second,'euler','ZYX','frame');
quatAverage_second = meanrot(quat_second);

%%
q2 =  [0.39827, 0.047998,-0.12489,-0.90746];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
third = ts.Data(20380:21660,:);
eulZYX_third = quat2eul(third);
quat_third = quaternion(eulZYX_third,'euler','ZYX','frame');
quatAverage_third = meanrot(quat_third);

%% new reference 90 degree
ref = ts.Data(49:4084,:);
eulZYX_ref = quat2eul(ref);
quat_ref = quaternion(eulZYX_ref,'euler','ZYX','frame');
quatAverage_ref = meanrot(quat_ref);
%%
q1 = [0.036689 0.031016 -0.68204 -0.72974];
q2 =  [0.021725, -0.000021259,-0.20675,-0.97815];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
fourth = ts.Data(18320:20020,:);
eulZYX_fourth = quat2eul(fourth);
quat_fourth = quaternion(eulZYX_fourth,'euler','ZYX','frame');
quatAverage_fourth = meanrot(quat_fourth);
%%
q2 =  [0.024728, 0.0030814,-0.25528,-0.96654];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
fifth = ts.Data(16140:18070,:);
eulZYX_fifth = quat2eul(fifth);
quat_fifth = quaternion(eulZYX_fifth,'euler','ZYX','frame');
quatAverage_fifth = meanrot(quat_fifth);
%%
q2 =  [0.025744, 0.005043,-0.29814,-0.95416];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
sixth = ts.Data(14080:15680,:);
eulZYX_sixth = quat2eul(sixth);
quat_sixth = quaternion(eulZYX_sixth,'euler','ZYX','frame');
quatAverage_sixth = meanrot(quat_sixth);
%%
q2 =  [0.024613, 0.0059242,-0.3325,-0.94276];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
seventh = ts.Data(12040:13760,:);
eulZYX_seventh = quat2eul(seventh);
quat_seventh = quaternion(eulZYX_seventh,'euler','ZYX','frame');
quatAverage_seventh = meanrot(quat_seventh);
%%
q2 =  [0.026799, 0.0079971,-0.36,-0.93253];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
eighth = ts.Data(10360:11660,:);
eulZYX_eighth = quat2eul(eighth);
quat_eighth = quaternion(eulZYX_eighth,'euler','ZYX','frame');
quatAverage_eighth = meanrot(quat_eighth);
%%
q2 =  [0.028788, 0.0099906,-0.38121,-0.92399];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
nineth = ts.Data(8472:9802,:);
eulZYX_nineth = quat2eul(nineth);
quat_nineth = quaternion(eulZYX_nineth,'euler','ZYX','frame');
quatAverage_nineth = meanrot(quat_nineth);
%%
q2 =  [0.033298, 0.012309,-0.39528,-0.91787];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
tenth = ts.Data(5389:7777,:);
eulZYX_tenth = quat2eul(tenth);
quat_tenth = quaternion(eulZYX_tenth,'euler','ZYX','frame');
quatAverage_tenth = meanrot(quat_tenth);
%%
q2 =  [0.034823, 0.014511,-0.4187,-0.90734];
%z = quatmultiply(quatconj(q1),q2)
%a = 2* atan2(norm(z(2:4)),z(1))/pi*180
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);


%% Plot
figure(1)
subplot(3,1,1)
plot(euler(:,1)/pi*180)
title('Euler X: Pitch Angle')
xlabel('Time/s')
ylabel('Angle/degree')

subplot(3,1,2)
plot(euler(:,2)/pi*180)
title('Euler Y: Yaw Angle')
xlabel('Time/s')
ylabel('Angle/degree')

subplot(3,1,3)
plot(euler(:,3)/pi*180)
title('Euler Z: Roll Angle')
xlabel('Time/s')
ylabel('Angle/degree')
