clc

% Load MAVLink protocol dialect
dialect = mavlinkdialect("common.xml");

% Create mavlinkio and connect via UDP on port 14550 (match QGC)
mavlink = mavlinkio(dialect);
connect(mavlink, "UDP", "LocalPort", 14445);

%% List Topics
listTopics(mavlink)
%% Subscribe to scaled IMU message (ID 26)
imuSub = mavlinksub(mavlink, 26, "BufferSize", 10);

pause(1); % allow time to receive messages
j = 1;
tCorrection = 0;
%% Read latest IMU messages from subscriber
while(1)
imuMsgs = latestmsgs(imuSub, 1);

for i = 1:numel(imuMsgs)
    msg = imuMsgs(i);
    disp(msg.Payload)

    % Add points to animated lines
    tic;
    % addpoints(hAzimuth, tVector(i) + tCorrection,sensorVal(1));
    % addpoints(hPitch, tVector(i) + tCorrection,sensorVal(2));
    % addpoints(hRoll, tVector(i) + tCorrection,sensorVal(3));

    addpoints(hAx, tVector(j) + tCorrection, msg.Payload.xacc);
    addpoints(hAy, tVector(j) + tCorrection, msg.Payload.yacc);
    addpoints(hAz, tVector(j) + tCorrection, msg.Payload.zacc);

    addpoints(hVx, tVector(j) + tCorrection, msg.Payload.xgyro);
    addpoints(hVy, tVector(j) + tCorrection, msg.Payload.ygyro);
    addpoints(hVz, tVector(j) + tCorrection, msg.Payload.zgyro);

    addpoints(hMagx, tVector(j) + tCorrection, msg.Payload.xmag);
    addpoints(hMagy, tVector(j) + tCorrection, msg.Payload.ymag);
    addpoints(hMagz, tVector(j) + tCorrection, msg.Payload.zmag);

    j = j +1;
    tCorrection = toc;
end
pause(0.01)
end

%% Close connection once done
disconnect(mavlink);

