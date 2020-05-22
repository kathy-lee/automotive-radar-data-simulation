clear;
bw = 250e6;
bw = 1e9;
fs = bw;
%fs = bw*2;
c = 3e8;
fc = 77e9;
tm = 1e-6;
wav = phased.FMCWWaveform('SampleRate',fs,'SweepTime',tm,...
    'SweepBandwidth',bw);

% FMCW radar is on the ego car
egocar_pos = [0;0;0];
egocar_vel = [10*1600/3600;0;0];
egocar_vel = [0;0;0];
egocar = phased.Platform('InitialPosition',egocar_pos,'Velocity',egocar_vel,...
    'OrientationAxesOutputPort',true);

parkedcar_pos = [39;-4;0];
parkedcar_vel = [0;0;0];
parkedcar = phased.Platform('InitialPosition',parkedcar_pos,'Velocity',parkedcar_vel,...
    'OrientationAxesOutputPort',true);
parkedcar_tgt = phased.RadarTarget('PropagationSpeed',c,'OperatingFrequency',fc,'MeanRCS',10);

ped_pos = [40;-3;0];
ped_vel = [0;1;0];
ped_heading = 90;
ped_height = 1.8;
rng(1)
randb=0.5;
% area of interest 
yLocLimit = [-10,10];
xLocLimit = [5,45];
ped_pos = [xLocLimit(1) + (xLocLimit(2)-xLocLimit(1))*rand;
    yLocLimit(1) + (yLocLimit(2)-yLocLimit(1))*rand;
    0]; % initial location
ped_height = 1.5 + (2-1.5)*rand; % height in U[1.5,2] meters
ped_heading = -180 + 360*randb; % heading in U[-180,180] degrees
ped_vel = rand*ped_height*1.4; % speed in U[0,1.4*height] m/s
        
ped = phased.BackscatterPedestrian('InitialPosition',ped_pos,'InitialHeading',ped_heading,...
    'PropagationSpeed',c,'OperatingFrequency',fc,'Height',ped_height,'WalkingSpeed',ped_vel);

chan_ped = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc,...
    'TwoWayPropagation',true,'SampleRate',fs);
chan_pcar = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc,...
    'TwoWayPropagation',true,'SampleRate',fs);

tx = phased.Transmitter('PeakPower',1,'Gain',25);
rx = phased.ReceiverPreamp('Gain',25,'NoiseFigure',10);


%Tsamp = 0.001;

maxbicSpeed = 10; % maximum speed of the Bicyclist
lambda = c/fc;
oversamplingFactor = 1.11; % oversampling factor in frequency domain
fmax = (2*maxbicSpeed/lambda)*oversamplingFactor; % calculate the sampling frequency based on the maximum speed of a bicyclist
Tsamp = 1/(2*fmax); % slow time sampling interval

npulse = 2500;
npulse = floor(2/Tsamp);

xr = complex(zeros(round(fs*tm),npulse));
xr_ped = complex(zeros(round(fs*tm),npulse));
xs_ped = complex(zeros(round(fs*tm),npulse));

for m = 1:npulse
    [pos_ego,vel_ego,ax_ego] = egocar(Tsamp);
    [pos_pcar,vel_pcar,ax_pcar] = parkedcar(Tsamp);
    [pos_ped,vel_ped,ax_ped] = move(ped,Tsamp,ped_heading);
    [~,angrt_ped] = rangeangle(pos_ego,pos_ped,ax_ped);
    [~,angrt_pcar] = rangeangle(pos_ego,pos_pcar,ax_pcar);

    x = tx(wav());
    xt_ped = chan_ped(repmat(x,1,size(pos_ped,2)),pos_ego,pos_ped,vel_ego,vel_ped);
    xt_pcar = chan_pcar(x,pos_ego,pos_pcar,vel_ego,vel_pcar);
    xt_ped = reflect(ped,xt_ped,angrt_ped);
    xt_pcar = parkedcar_tgt(xt_pcar);
    xr_ped(:,m) = rx(xt_ped);
    xs_ped(:,m) = xt_ped;
    xr(:,m) = rx(xt_ped+xt_pcar);
end

% only echo of pedestrian
xd_ped = conj(dechirp(xr_ped,x));
xd = conj(dechirp(xr,x));
clf;
spectrogram(sum(xd_ped),kaiser(128,10),120,256,1/Tsamp,'centered','yaxis');
clim = get(gca,'CLim');
set(gca,'CLim',clim(2)+[-50 0])

% feature engineering
M = 200; % FFT window length
beta = 6; % window parameter
w = kaiser(M,beta);
R = floor(1.7*(M-1)/(beta+1)); % ROUGH estimate
noverlap = M-R; % overlap length

[S,F,T] = stft(squeeze(sum(xd_ped,1)),1/Tsamp,'Window',w,'FFTLength',M*2,'OverlapLength',noverlap);
S = helperPreProcess(S); % preprocessing of the spectrogram
figure(2)
imagesc(T,F,S(:,:,1))

% echo of pedestrian and car
spectrogram(sum(xd),kaiser(128,10),120,256,1/Tsamp,'centered','yaxis');
clim = get(gca,'CLim');
set(gca,'CLim',clim(2)+[-50 0])


function S = helperPreProcess(S)
%helperPreProcess converts each spectrogram into log-scale and normalizes each log-scale spectrogram
%to [0,1].
    
    S = 10*log10(abs(S)); % logarithmic scaling to dB
    for ii = 1:size(S,3)
        zs = S(:,:,ii);
        zs = (zs - min(zs(:)))/(max(zs(:))-min(zs(:))); % normalize amplitudes of each map to [0,1]
        S(:,:,ii) = zs;
    end
end
