bag = rosbag('/Users/zhenhu/Documents/Second_semester/CIS/Calibration/NdiThird.bag');
%%
bagselect = select(bag, 'Topic', '/ndi_01_3B911400_610066___T6D0_S01764/position_cartesian_current'); 
msgStructs = readMessages(bagselect,'DataFormat','struct');
ts = timeseries(bagselect, 'Pose.Orientation.W', 'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z');
%%
plot(ts.Data(:,1));
hold on
plot(ts.Data(:,2));
plot(ts.Data(:,3));
plot(ts.Data(:,4));
legend('Orientation.W','Orientation.X','Orientation.Y','Orientation.Z')
title('The measurement result from EM tracker')
%% take horizontal plane as reference (0 degree)
reference = ts.Data(22830:end,:);
eulZYX = quat2eul(reference);
% ref = eul2quat(eulZYX);
quat = quaternion(eulZYX,'euler','ZYX','frame');
quatAverage = meanrot(quat)

%% next degree
first = ts.Data(21140:22380,:);
eulZYX_first = quat2eul(first);
quat_first = quaternion(eulZYX_first,'euler','ZYX','frame');
quatAverage_first = meanrot(quat_first)

%% angle
q1 =  [0.67018, -0.0022938,-0.0027044,0.74219];
q2 =  [0.68042, 0.019302,-0.024501,0.73216];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
second = ts.Data(14970:20800,:);
eulZYX_second = quat2eul(second);
quat_second = quaternion(eulZYX_second,'euler','ZYX','frame');
quatAverage_second = meanrot(quat_second)

%%
q2 =  [0.63692, 0.14869,-0.13058,0.7451];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
third = ts.Data(13270:14450,:);
eulZYX_third = quat2eul(third);
quat_third = quaternion(eulZYX_third,'euler','ZYX','frame');
quatAverage_third = meanrot(quat_third)

%% new reference 90 degree
ref = ts.Data(56:1466,:);
eulZYX_ref = quat2eul(ref);
quat_ref = quaternion(eulZYX_ref,'euler','ZYX','frame');
quatAverage_ref = meanrot(quat_ref)
%%
q1 = [0.48126 0.45063 -0.51143 0.55115];
q2 =  [0.63692, 0.14866,-0.13058,0.7451];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
fourth = ts.Data(11840:13070,:);
eulZYX_fourth = quat2eul(fourth);
quat_fourth = quaternion(eulZYX_fourth,'euler','ZYX','frame');
quatAverage_fourth = meanrot(quat_fourth)
%%
q2 =  [0.63051, 0.18297,-0.16824,0.73531];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
fifth = ts.Data(10110:11690,:);
eulZYX_fifth = quat2eul(fifth);
quat_fifth = quaternion(eulZYX_fifth,'euler','ZYX','frame');
quatAverage_fifth = meanrot(quat_fifth)
%%
q2 =  [0.61967, 0.21345,-0.20056,0.72816];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
sixth = ts.Data(8768:9965,:);
eulZYX_sixth = quat2eul(sixth);
quat_sixth = quaternion(eulZYX_sixth,'euler','ZYX','frame');
quatAverage_sixth = meanrot(quat_sixth)
%%
q2 =  [0.60936, 0.23815,-0.22693,0.72144];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
seventh = ts.Data(7162:8483,:);
eulZYX_seventh = quat2eul(seventh);
quat_seventh = quaternion(eulZYX_seventh,'euler','ZYX','frame');
quatAverage_seventh = meanrot(quat_seventh)
%%
q2 =  [0.59986, 0.25791,-0.24834,0.71553];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
eighth = ts.Data(5990:6992,:);
eulZYX_eighth = quat2eul(eighth);
quat_eighth = quaternion(eulZYX_eighth,'euler','ZYX','frame');
quatAverage_eighth = meanrot(quat_eighth)
%%
q2 =  [0.59188, 0.2728,-0.26543,0.71049];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
nineth = ts.Data(4473:5684,:);
eulZYX_nineth = quat2eul(nineth);
quat_nineth = quaternion(eulZYX_nineth,'euler','ZYX','frame');
quatAverage_nineth = meanrot(quat_nineth)
%%
q2 =  [0.58778, 0.28112,-0.27734,0.7061];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180

%% next degree
tenth = ts.Data(2107:4288,:);
eulZYX_tenth = quat2eul(tenth);
quat_tenth = quaternion(eulZYX_tenth,'euler','ZYX','frame');
quatAverage_tenth = meanrot(quat_tenth)
%%
q2 =  [0.58794, 0.28668,-0.29914,0.69473];
z = quatmultiply(quatconj(q1),q2)
%a = 2* acosd(z(1))
a = 2* atan2(norm(z(2:4)),z(1))/pi*180