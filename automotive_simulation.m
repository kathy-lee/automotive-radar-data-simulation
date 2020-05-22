
clear;
rng(1)


for i = 1:500
    numTargets = randi([0,2], 1, 2);
    while(numTargets(1) == 0 & numTargets(2) == 0)
        numTargets = randi([0,2], 1, 2);
        continue;
    end
    fprintf('create no %d th scene: %d pedestrian, %d bicyclist.\n', i, numTargets(1), numTargets(2));
    egoVel = 0;
    fc = 24e9;
    bw = 250e6;
    OtherCar = 0; % flag0/1, if other driving car exists in the scene
    [xRecF, Tsamp, label] = automotive_simu(numTargets(1),numTargets(2),OtherCar,fc,bw,egoVel);
    %[xRecF, Tsamp, label] = automotive_simu(numTargets(1), numTargets(2), 0, 24e9, 250e6, 0);
    filename = strcat(num2str(i));
    save(filename, 'xRecF', 'Tsamp', 'label', 'fc','bw','-v7.3');

end

clf;
spectrogram(sum(xRecF),kaiser(128,10),120,256,1/Tsamp,'centered','yaxis');
clim = get(gca,'CLim');
set(gca,'CLim',clim(2)+[-50 0])


function [xRec, Tsamp, label] = automotive_simu(numPed, numBic, carFlag, fc, bw, egoVel)
% simulate scene

% pedestrian num:0-2
% bicylist num:0-2
% moving car num:0-2
% stationary targets num:

% egoCar:staionary/moving
% Rmax: m
% Vmax: km/h

    %rng(1)

    c = 3e8;

    fs = bw*2;

    maxbicSpeed = 10; % maximum speed of the Bicyclist
    lambda = c/fc;
    oversamplingFactor = 1.11; % oversampling factor in frequency domain
    fmax = (2*maxbicSpeed/lambda)*oversamplingFactor; % calculate the sampling frequency based on the maximum speed of a bicyclist
    Tsamp = 1/(2*fmax); % slow time sampling interval

    % radar operating related parameters
    tm = 1e-6; % waveform repetition time   
    timeDuration = 2; % simulation time duration in seconds
    npulse = floor(timeDuration/Tsamp); % number of pulses
    fprintf('echo dimension: (%d,%d)\n',fs*tm,npulse);
    
    % radar plat parameters
    radar_pos = [0;0;0]; % radar position
    if egoVel == 0
        radar_vel = [0;0;0]; % radar velocity
    else
        radar_vel = [egoVel;0;0];
    end
      
    radarplat = phased.Platform('InitialPosition',radar_pos,'Velocity',radar_vel,...
        'OrientationAxesOutputPort',true);
    
    % radar waveform
    wav = phased.FMCWWaveform('SampleRate',fs,'SweepTime',tm,'SweepBandwidth',bw);
    tx = phased.Transmitter('PeakPower',1,'Gain',25);
    txWave = tx(wav());
    channel_ped = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc,...
            'TwoWayPropagation',true,'SampleRate',fs); % channel
    channel_bic = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc,...
            'TwoWayPropagation',true,'SampleRate',fs); % channel
    channel_car = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc,...
            'TwoWayPropagation',true,'SampleRate',fs); % channel
    
    [posr,velr,~] = radarplat(Tsamp); % radar position and velocity
    
    numCar = 0;
    if carFlag == 1
        numCar = 1;
    end
    
    % signal initialization
    xRec = complex(zeros(round(fs*tm),npulse));
    xRecF = complex(zeros(round(fs*tm),npulse));
    
    xPedRec = complex(zeros(round(fs*tm),npulse));
    xBicRec = complex(zeros(round(fs*tm),npulse));
    xCarRec = complex(zeros(round(fs*tm),npulse));
    
    % area of interest 
    yLocLimit = [-10,10];
    xLocLimit = [5,45];
    
    % Generation of pedestrian signals    
    for i = 1:numPed
        % Pedestrian parameters
        ped_pos = [xLocLimit(1) + (xLocLimit(2)-xLocLimit(1))*rand;
            yLocLimit(1) + (yLocLimit(2)-yLocLimit(1))*rand;
            0]; % initial location
        ped_height = 1.5 + (2-1.5)*rand; % height in U[1.5,2] meters
        ped_speed = rand*ped_height*1.4; % speed in U[0,1.4*height] m/s
        ped_heading(i) = -180 + 360*rand; % heading in U[-180,180] degrees        
        pedestrian(i) = backscatterPedestrian('InitialPosition',ped_pos,...
            'InitialHeading',ped_heading(i),'PropagationSpeed',c,...
            'OperatingFrequency',fc,'Height',ped_height,...
            'WalkingSpeed',ped_speed); % pedestrian object
        
        label(i).type = 'pedestrian';
        label(i).pos = ped_pos;
        label(i).speed = ped_speed;
        label(i).heading = ped_heading(i);
              
    end
    
    % Generation of bicyclist signals
    for i = 1:numBic  
        % Bicyclist parameters
        bic_pos = [xLocLimit(1) + (xLocLimit(2)-xLocLimit(1))*rand;
            yLocLimit(1) + (yLocLimit(2)-yLocLimit(1))*rand;
            0]; % initial location
        bicyclistSpeed = 1 + (10-1)*rand; % Speed in U[1,10] meters
        bic_heading(i) = -180 + 360*rand; % heading in U[-180,180] degrees
        GearTransmissionRatio = 0.5 + (6-0.5)*rand; % in U[0.5,6],the ratio of wheel rotations to pedal rotations
        NumWheelSpokes = 36; % number of spokes
        Coast = rand<0.5; % 50% chance to be pedaling or coasting
        bicyclist(i) = backscatterBicyclist('InitialPosition',bic_pos,'InitialHeading',bic_heading(i),...
            'Speed',bicyclistSpeed,'PropagationSpeed',c,'OperatingFrequency',fc,...
            'GearTransmissionRatio',GearTransmissionRatio,'NumWheelSpokes',NumWheelSpokes,...
            'Coast',Coast); % bicyclist object
        
        label(i+numPed).type = 'bicyclist';
        label(i+numPed).pos = bic_pos;
        label(i+numPed).speed = bicyclistSpeed;
        label(i+numPed).heading = bic_heading(i);
    end
    
    maxCarSpeed = 10;
    % Generation of car signals
    for i = 1:numCar
        % Car parameters
        car_pos = [xLocLimit(1) + (xLocLimit(2)-xLocLimit(1))*rand;
            yLocLimit(1) + (yLocLimit(2)-yLocLimit(1))*rand;
            0]; % initial location
        car_vel = [-maxCarSpeed+(maxCarSpeed+maxCarSpeed)*rand;
            -maxCarSpeed+(maxCarSpeed+maxCarSpeed)*rand;
            0]; % car velocity
        car = phased.Platform('InitialPosition',car_pos,'Velocity',car_vel,...
            'OrientationAxesOutputPort',true); % car object
        carTgt = phased.RadarTarget('PropagationSpeed',c,'OperatingFrequency',fc,'MeanRCS',10);
        label(i+numPed+numBic).type = 'car';
        label(i+numPed+numBic).pos = car_pos;
        label(i+numPed+numBic).speed = car_vel;
    end
    
    for m = 1:npulse  
        for i = 1:numPed
            [posPed,velPed,axPed] = move(pedestrian(i),Tsamp,ped_heading(i)); % pedestrian moves
            [~,angrPed] = rangeangle(posr,posPed,axPed); % propagation path direction
            xPedCh = channel_ped(repmat(txWave,1,size(posPed,2)),posr,posPed,velr,velPed); % simulate channel
            xPed = reflect(pedestrian(i),xPedCh,angrPed); % signal reflection
            %xPedRec(:,m) = xPed; % received m-th pulse
            xRec(:,m) = xRec(:,m) + xPed;
        end
        
        for i = 1:numBic
            [posBic,velBic,axBic] = move(bicyclist(i),Tsamp,bic_heading(i)); % target moves
            [~,angrBic] = rangeangle(posr,posBic,axBic); % propagation path direction
            xBicCh = channel_bic(repmat(txWave,1,size(posBic,2)),posr,posBic,velr,velBic); % simulate channel
            xBic = reflect(bicyclist(i),xBicCh,angrBic); % signal reflection
            %xBicRec(:,m) = xBic; % received m-th pulse
            xRec(:,m) = xRec(:,m) + xBic;
        end
        
        for i = 1:numCar 
            [posCar,velCar,~] = car(Tsamp); % car moves
            xCarCh = channel_car(repmat(txWave,1,size(posCar,2)),posr,posCar,velr,velCar); % simulate channel
            xCar = carTgt(xCarCh); % detection
            %xCarRec(:,m) = xCar; % received m-th pulse
            xRec(:,m) = xRec(:,m) + xCar;
        end
        
%         if mod(m,100) == 0
%             fprintf('%d pulses created\n',m);
%         end
    end
         
    % Configure Gaussian noise level at the receiver
    rx = phased.ReceiverPreamp('Gain',25,'NoiseFigure',10);
    xRecF = rx(xRecF);
    % convert received signals to baseband  
    xRecF = conj(dechirp(xRec, txWave));    
    %xPedRecF(:,:,ii) = conj(dechirp(xPedRec,txWave));     
    
end
