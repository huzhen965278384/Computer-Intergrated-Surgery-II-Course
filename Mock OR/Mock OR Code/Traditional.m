%% Read in the CSV File
trad =  readtable('/Users/zhenhu/Documents/Second_semester/CIS/Mock OR/mocktraditional.csv');
%% Reference position's quaternion and eular
start_A6 =  [0.80241, -0.3799,0.13374,0.44037];
start_A1 =  [0.63627, -0.77083,-0.00032067,-0.031174];

start_A6_Euler = [47.1180    6.8894  -54.5092];
start_A1_Euler = [100.8701   -2.7313    2.3045];

%% Separate the A6 and A1 data
trad = sortrows(trad,1);
trad = [trad(:,1:3) trad(:,13:19)];
trad = table2array(trad);

A6 = trad(trad(:,1)==1,:);
A6 = A6(1:end-5,:);
A1 = trad(trad(:,1)==2,:);

%% Extract the time stamp and quaternion data of A6 and A1
time_A6 = A6(:,2);
Quat_A6 = A6(:,7:10);

time_A1 = A1(:,2);
Quat_A1 = A1(:,7:10);

Quat_A6_Euler = A6(:,4:6);
Quat_A1_Euler = A1(:,4:6);

%% Calibration of Pitch Angles for reference data
% for A6:
calibrated_start_A6_Euler = start_A6_Euler;
calibrated_start_A6_Euler(:,1) = 1.0411 * start_A6_Euler(:,1) - 2.1207;

% for A1:
calibrated_start_A1_Euler = start_A1_Euler;
calibrated_start_A1_Euler(:,1) = 1.0295 * start_A1_Euler(:,1) - 1.7564;
%% %% Calibration of Pitch Angles for other data
% for A6:
calibrated_Quat_A6_Euler = Quat_A6_Euler;
calibrated_Quat_A6_Euler(:,1) = 1.0411 * Quat_A6_Euler(:,1) - 2.1207;

% for A1:
calibrated_Quat_A1_Euler = Quat_A1_Euler;
calibrated_Quat_A1_Euler(:,1) = 1.0295 * Quat_A1_Euler(:,1) - 1.7564;

%% From Euler to Quaternion for reference data
QA1_start = eul2quat(-calibrated_start_A1_Euler*pi/180,'XYZ');
QA6_start = eul2quat(-calibrated_start_A6_Euler*pi/180,'XYZ');

%% From Quaternion to Rotation matrix for reference data
A6_start_RM = quat2rotm(QA6_start);
A1_start_RM = quat2rotm(QA1_start);

%% From Euler to Quaternion for other data
QA1 = eul2quat(-calibrated_Quat_A1_Euler*pi/180,'XYZ');
QA6 = eul2quat(-calibrated_Quat_A6_Euler*pi/180,'XYZ');

%% From Quaternion to Rotation matrix for other data
A6_RM0 = cell(size(QA6,1),1);
A1_RM0 = cell(size(QA1,1),1);

for i =1:size(QA6,1)
    A6_RM0(i,1) = {quat2rotm(QA6(i,:))};
    A1_RM0(i,1) = {quat2rotm(QA1(i,:))};
end
%% A6, A1 w.r.t their reference:
% others w.r.t reference, using reference.transpose * others
A6_RM = cell(size(QA6,1),1);
A1_RM = cell(size(QA1,1),1);

for i = 1:size(QA6,1)
    A6_RM(i,1) = {A6_start_RM'*cell2mat(A6_RM0(i,1))};
    A1_RM(i,1) = {A1_start_RM'*cell2mat(A1_RM0(i,1))};
%     A6_RM(i,1) = {cell2mat(A6_RM0{i,1})'*cell2mat(A6_start_RM{1,1})};
%     A1_RM(i,1) = {cell2mat(A1_RM0{i,1})'*cell2mat(A1_start_RM{1,1})};
end

%% Calculate the Difference Rotation matrix between the two IMUs
D = cell(size(Quat_A6,1),1);
for i = 1:size(Quat_A6,1)
    D(i,1) = {A6_RM{i,1}'*A1_RM{i,1}};
end

% calculate the three small angel matrix 
eul = zeros(size(Quat_A6,1),3);
for i = 1:size(Quat_A6,1)
    eul(i,:) = rotate2euler(cell2mat(D(i,1)));
end 

%% Calculating the mean and SD for pitch angles
M = mean(eul(:,1)/pi*180)
SD = std(eul(:,1)/pi*180)
%% Figures
figure(1)
subplot(3,1,1)
plot(time_A6,eul(:,1)/pi*180)
title('Euler X: Pitch Angle')
xlabel('Time/s')
ylabel('Angle/degree')

subplot(3,1,2)
plot(time_A6,eul(:,2)/pi*180)
title('Euler Y: Yaw Angle')
xlabel('Time/s')
ylabel('Angle/degree')

subplot(3,1,3)
plot(time_A6,eul(:,3)/pi*180)
title('Euler Z: Roll Angle')
xlabel('Time/s')
ylabel('Angle/degree')

figure(2)
histogram(eul(:,1)/pi*180)
hold on
histogram(eul(:,2)/pi*180)
hold on
histogram(eul(:,3)/pi*180)
xlabel('Angle/degree')
ylabel('Count/sample')
title('The histogram of Angles')
legend('Pitch','Yaw','Roll')
