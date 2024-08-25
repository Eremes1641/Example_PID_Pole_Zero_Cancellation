clear; clc; close all;

%% declare
% sampling
Fs = 1000;   % sampling freq [Hz]
dt = 1/Fs;   % sampling period [sec]
time = 0:dt:2;  % time span
lenT = length(time);

% plant (continuous)
%    b
% -------
%  s + a
param.a = 5;
param.b = 7;
param.ctr_bw = 10;


%% sim
% simulation
IC = [0 0]';
equ = @(t,state)equ_1st_order_sys(t,state,param);
[t_sim,X_sim] = ode45(equ, time, IC);

% extract data
simData = extract_sim_data(equ, t_sim, X_sim);
t = simData.t;
x1 = simData.x1;
u = simData.u;
x1d = simData.x1d;


%% plot
% fig
figure;
ax = createSubplot(2,1);
ax_x1 = ax(1);
ax_u = ax(2);
hold(ax,'on')

% plot
plot(ax_x1,time,x1)
plot(ax_x1,time,x1d)
plot(ax_u,time,u)

% property
grid(ax,'on')
loose_ylim(ax)
title(ax_x1,'x1')
title(ax_u,'u')
legend(ax_x1,{'x1','x1d'})
xlabel(ax,'time (sec)')
linkaxes(ax,'x')



%%
function [stateDt,simOut] = equ_1st_order_sys(t,state,param)
    %% declare
    % plant
    a = param.a;
    b = param.b;
    % controller
    bw = param.ctr_bw;
    Kp = 1/b*bw;
    Ki = a/b*bw;

    %% substitute state
    % plant
    x1 = state(1);
    % controller
    Sedt = state(2);    % integral

    %% controller
    % error
    x1d = 1;
    e1 = x1d - x1;

    % integral
    SedtDt = e1;

    % PID
    u = Kp*e1 + Ki*Sedt;

    %% plant motor
    % plant
    x1Dt = -a*x1 + b*u;

    %% return
    stateDt = [x1Dt;
        SedtDt];
    simOut.t = t;       % time [sec]
    simOut.u = u;       % input
    simOut.x1 = x1;     % x1
    % controller
    simOut.x1d = x1d;   % desired x1
end


%% subfunction

function simOut = extract_sim_data(equ, t, state)
    %% extract_sim_data
    %
    % input: [equ, t, state]
    % equ       @(t,state)      simulation equation
    % t         1D double       time span
    % state     2D double       equation state
    %
    % output: simOut
    % simOut    sutructure      simulation data out
    %
    % update:2022/02/10
    % Author:Jim
    
    %% --------------------------------------
    lenT = length(t);
    for i = 1:lenT
        [~,simOutTemp(i)] = equ(t(i),state(i,:)');
    end
    
    simOut = ArrayStruc2StrucArray(simOutTemp);
end

function StrucArray = ArrayStruc2StrucArray(ArrayStruc)
    %% ArrayStruc2StrucArray
    % convert array of structure to structure of array
    %
    % input: ArrayStruc
    % ArrayStruc    array       array of structure
    %
    % output: StrucArray
    % StrucArray    structure   structure of array
    %
    % update:2022/02/01
    % Author:Jim
    
    %% --------------------------------------
    fieldName = fields(ArrayStruc(1));
    for i = 1:length(fieldName)
        StrucArray.(fieldName{i}) = [ArrayStruc.(fieldName{i})]';
    end
end

function ax = createSubplot(numRow,numCol)
    ax(numRow,numCol) = axes;
    for i = 1:numRow*numCol
        row = idivide(i-1,int16(numCol))+1;
        col = rem(i-1,int16(numCol))+1;
        ax(row,col) = subplot(numRow,numCol,i);
    end
end

function loose_ylim(axIn,scale)
    %% loose_ylim
    % loose y limit
    %
    % input: (axIn,scale)
    % ax        2D  axes    axes in
    % scale     double      0 ~ 1, default value is 0.2
    %
    % update:2021/08/20
    % Author:Jim
    
    %% --------------------------------------
    % default
    if nargin < 2
        scale = 0.2;
    end
    
    % access all axes
    for k = 1:size(axIn,1)
        for j = 1:size(axIn,2)
            % get line
            ax = axIn(k,j);
            Line = ax.Children;
            numLine = length(Line);
            y = [];
            for i = 1:numLine
                y = [y Line.YData];
            end
            
            % run
            y = reshape(y,[],1);
            minValue = min(y);
            maxValue = max(y);
            if minValue ~= maxValue
                extend = (maxValue-minValue)*scale;
                ylim(ax,[minValue-extend, maxValue+extend]);
            end
        end
    end
end