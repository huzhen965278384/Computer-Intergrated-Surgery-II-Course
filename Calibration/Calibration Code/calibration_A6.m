%% Read in Rosbag Files
bag = rosbag('/Users/zhenhu/Documents/Second_semester/CIS/Calibration/A6Third.bag');
bagselect = select(bag, 'Topic', '/imu_A6/imu'); 
msgStructs = readMessages(bagselect,'DataFormat','struct');
ts = timeseries(bagselect, 'Orientation.W', 'Orientation.X','Orientation.Y','Orientation.Z');
% % %%
% % plot(ts.Data(:,1));
% % hold on
% % plot(ts.Data(:,2));
% % plot(ts.Data(:,3));
% % plot(ts.Data(:,4));
% % legend('Orientation.W','Orientation.X','Orientation.Y','Orientation.Z')
%% take horizontal plane as reference (0 degree)
reference = ts.Data(29610:end,:);
eulZYX = quat2eul(reference);
% ref = eul2quat(eulZYX);
quat = quaternion(eulZYX,'euler','ZYX','frame');
quatAverage = meanrot(quat);

%% next degree
first = ts.Data(27540:28990,:);
eulZYX_first = quat2eul(first);
quat_first = quaternion(eulZYX_first,'euler','ZYX','frame');
quatAverage_first = meanrot(quat_first);

%% angle
%q1 =  [0.041334, -0.99756,0.056274,-0.0003621];
q1 = [0.67421 -0.15721 0.71156 0.12006];
q2 =  [0.04337, -0.99776,0.041305,0.029929];
euler = zeros(10,3);
i = 1;
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
second = ts.Data(23240:27000,:);
eulZYX_second = quat2eul(second);
quat_second = quaternion(eulZYX_second,'euler','ZYX','frame');
quatAverage_second = meanrot(quat_second);

%%
q2 =  [0.048662, -0.98891,0.045159,0.13288];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
third = ts.Data(20630:21910,:);
eulZYX_third = quat2eul(third);
quat_third = quaternion(eulZYX_third,'euler','ZYX','frame');
quatAverage_third = meanrot(quat_third);

%% new reference 90 degree
ref = ts.Data(11:4459,:);
eulZYX_ref = quat2eul(ref);
quat_ref = quaternion(eulZYX_ref,'euler','ZYX','frame');
quatAverage_ref = meanrot(quat_ref);
%%
%q1 = [0.67421 -0.15721 0.71156 0.12006];
q2 =  [0.21247, -0.21314,0.95363,0.0032791];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
fourth = ts.Data(18650:20240,:);
eulZYX_fourth = quat2eul(fourth);
quat_fourth = quaternion(eulZYX_fourth,'euler','ZYX','frame');
quatAverage_fourth = meanrot(quat_fourth);
%%
q2 =  [0.25978, -0.21321,0.94172,0.014708];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
fifth = ts.Data(16360:18350,:);
eulZYX_fifth = quat2eul(fifth);
quat_fifth = quaternion(eulZYX_fifth,'euler','ZYX','frame');
quatAverage_fifth = meanrot(quat_fifth);
%%
q2 =  [0.30084, -0.20924,0.93014,0.023448];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
sixth = ts.Data(14280:15970,:);
eulZYX_sixth = quat2eul(sixth);
quat_sixth = quaternion(eulZYX_sixth,'euler','ZYX','frame');
quatAverage_sixth = meanrot(quat_sixth);
%%
q2 =  [0.33403, -0.20512,0.91946,0.030636];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
seventh = ts.Data(12360:14030,:);
eulZYX_seventh = quat2eul(seventh);
quat_seventh = quaternion(eulZYX_seventh,'euler','ZYX','frame');
quatAverage_seventh = meanrot(quat_seventh);
%%
q2 =  [0.36037, -0.20159,0.91003,0.036688];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
eighth = ts.Data(10680:12050,:);
eulZYX_eighth = quat2eul(eighth);
quat_eighth = quaternion(eulZYX_eighth,'euler','ZYX','frame');
quatAverage_eighth = meanrot(quat_eighth);
%%
q2 =  [0.38112, -0.20053,0.90153,0.042229];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
nineth = ts.Data(8840:10110,:);
eulZYX_nineth = quat2eul(nineth);
quat_nineth = quaternion(eulZYX_nineth,'euler','ZYX','frame');
quatAverage_nineth = meanrot(quat_nineth);
%%
q2 =  [0.39486, -0.20029,0.89547,0.045847];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

%% next degree
tenth = ts.Data(5688:8316,:);
eulZYX_tenth = quat2eul(tenth);
quat_tenth = quaternion(eulZYX_tenth,'euler','ZYX','frame');
quatAverage_tenth = meanrot(quat_tenth);

%%
q2 =  [0.41742, -0.19868,0.88519,0.052132];
q1_RM = RM_q(q1);
q2_RM = RM_q(q2);
D = q2_RM'*q1_RM;
euler(i,:) = RM_eul(D);
i = i+1;

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
