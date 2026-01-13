clc; clear; close all;

%% ================== USER INPUT ==================
word = upper(input('Enter word to write: ', 's'));
numSamples = 20;

%% ================== LOAD ROBOT ==================
sawyer = importrobot('sawyer.urdf');
sawyer.DataFormat = 'column';
sawyer.Gravity = [0 0 -9.81];

eeName = 'right_hand';

%% ================== GENERATE WORD PATH ==================
[pathSegments, letterIdx] = generateWordPaths(word, numSamples);

%% ================== VISUALIZE TASK-SPACE PATH ==================
figure; hold on; grid on;
for i = 1:numel(pathSegments)
    seg = pathSegments{i};
    plot3(seg(:,1), seg(:,2), seg(:,3), 'r', 'LineWidth', 2);
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title(['Task Space Path (XZ Plane) - "', word, '"']);
axis equal;
view(10,15);
hold off;

%% ================== SHOW ROBOT & PATH ==================
figure;
show(sawyer); hold on;
for i = 1:numel(pathSegments)
    seg = pathSegments{i};
    plot3(seg(:,1), seg(:,2), seg(:,3), 'k--', 'LineWidth', 1.5);
end
view(10,15);
axis([-1 1 -.6 .6 -0.8 0.8]);
hold off;

%% ================== INVERSE KINEMATICS ==================
ik = inverseKinematics('RigidBodyTree', sawyer);
ik.SolverParameters.AllowRandomRestarts = false;

weights = [1 1 1 0.2 0.2 0.2];
orientation = axang2rotm([0 1 0 pi]);

jointPath = [];
drawMask  = [];

%% ================== LETTER-BY-LETTER MOTION ==================
for L = 1:numel(letterIdx)

    initialGuess = sawyer.homeConfiguration;
    strokes = letterIdx{L};

    firstSeg = pathSegments{strokes(1)};
    pStart = firstSeg(1,:)';

    % Travel move off plane (+Y)
    Ttravel = [orientation [pStart(1); pStart(2)+0.1; pStart(3)]; 0 0 0 1];
    qTravel = ik(eeName, Ttravel, weights, initialGuess);

    jointPath(:,end+1) = qTravel;
    drawMask(end+1) = false;
    initialGuess = qTravel;

    % Drawing strokes
    for s = strokes
        seg = pathSegments{s};
        for k = 1:size(seg,1)
            T = [orientation seg(k,:)'; 0 0 0 1];
            q = ik(eeName, T, weights, initialGuess);

            jointPath(:,end+1) = q;
            drawMask(end+1) = true;
            initialGuess = q;
        end
    end
end

%% ================== GUARANTEED UPWARD LIFT ==================
Tend = getTransform(sawyer, jointPath(:,end), eeName);
pEnd = tform2trvec(Tend)';

liftHeight = 0.25;
nLiftPts   = 25;

zLift = linspace(pEnd(3), pEnd(3)+liftHeight, nLiftPts);
qPrev = jointPath(:,end);

for i = 2:nLiftPts
    pLift = [pEnd(1); pEnd(2); zLift(i)];
    Tup = [orientation pLift; 0 0 0 1];

    qUp = ik(eeName, Tup, weights, qPrev);

    jointPath(:,end+1) = qUp;
    drawMask(end+1) = false;
    qPrev = qUp;
end

%% ================== END-EFFECTOR TRAJECTORY ==================
xEE = []; yEE = []; zEE = [];

for k = 1:size(jointPath,2)
    T = getTransform(sawyer, jointPath(:,k), eeName);
    p = tform2trvec(T);

    if drawMask(k)
        xEE(end+1) = p(1);
        yEE(end+1) = p(2);
        zEE(end+1) = p(3);
    else
        xEE(end+1) = NaN;
        yEE(end+1) = NaN;
        zEE(end+1) = NaN;
    end
end

%% ================== ROBOT ANIMATION ==================
figure;
for k = 1:3:length(jointPath)
    show(sawyer, jointPath(:,k), 'PreservePlot', false, 'Frames','off');
    hold on
    plot3(xEE(1:k), yEE(1:k), zEE(1:k), 'r', 'LineWidth', 2)
    view(10,15)
    axis([-1 1 -.6 .6 -0.8 0.8]);
    drawnow
end
hold off;

%% ================== PATH GENERATOR (ALL LETTERS & NUMBERS) ==================
function [paths, letterIdx] = generateWordPaths(word, n)

yPlane  = -0.15;
scale   = 0.10;
spacing = 0.18;
xStart  = 0.45;
zBase   = 0.25;

paths = {};
letterIdx = {};
xOffset = 0;

L = @(x1,z1,x2,z2) ...
    [linspace(x1,x2,n)', yPlane*ones(n,1), linspace(z1,z2,n)'];

for ch = word
    switch ch

        % ===== ALPHABETS =====
        case 'A'
            letter = {L(0,0,0.5,1), L(1,0,0.5,1), L(0.25,0.5,0.75,0.5)};
        case 'B'
            letter = {L(0,0,0,1), L(0,1,0.7,0.75), L(0.7,0.75,0,0.5), ...
                      L(0,0.5,0.7,0.25), L(0.7,0.25,0,0)};
        case 'C'
            letter = {L(1,1,0,1), L(0,1,0,0), L(0,0,1,0)};
        case 'D'
            letter = {L(0,0,0,1), L(0,1,1,0.5), L(1,0.5,0,0)};
        case 'E'
            letter = {L(0,0,0,1), L(0,1,1,1), L(0,0.5,0.8,0.5), L(0,0,1,0)};
        case 'F'
            letter = {L(0,0,0,1), L(0,1,1,1), L(0,0.5,0.8,0.5)};
        case 'G'
            letter = {L(1,1,0,1), L(0,1,0,0), L(0,0,1,0), L(1,0,1,0.5)};
        case 'H'
            letter = {L(0,0,0,1), L(1,0,1,1), L(0,0.5,1,0.5)};
        case 'I'
            letter = {L(0.5,0,0.5,1)};
        case 'J'
            letter = {L(1,1,1,0.2), L(1,0.2,0.5,0), L(0.5,0,0,0.2)};
        case 'K'
            letter = {L(0,0,0,1), L(0,0.5,1,1), L(0,0.5,1,0)};
        case 'L'
            letter = {L(0,1,0,0), L(0,0,1,0)};
        case 'M'
            letter = {L(0,0,0,1), L(0,1,0.5,0), L(0.5,0,1,1), L(1,1,1,0)};
        case 'N'
            letter = {L(0,0,0,1), L(0,1,1,0), L(1,0,1,1)};
        case 'O'
            letter = {L(0,0,1,0), L(1,0,1,1), L(1,1,0,1), L(0,1,0,0)};
        case 'P'
            letter = {L(0,0,0,1), L(0,1,1,1), L(1,1,1,0.5), L(1,0.5,0,0.5)};
        case 'Q'
            letter = {L(0,0,1,0), L(1,0,1,1), L(1,1,0,1), L(0,1,0,0), L(0.6,0.2,1,0)};
        case 'R'
            letter = {L(0,0,0,1), L(0,1,1,1), L(1,1,1,0.5), ...
                      L(1,0.5,0,0.5), L(0,0.5,1,0)};
        case 'S'
            letter = {L(1,1,0,1), L(0,1,0,0.5), L(0,0.5,1,0.5), ...
                      L(1,0.5,1,0), L(1,0,0,0)};
        case 'T'
            letter = {L(0,1,1,1), L(0.5,1,0.5,0)};
        case 'U'
            letter = {L(0,1,0,0), L(0,0,1,0), L(1,0,1,1)};
        case 'V'
            letter = {L(0,1,0.5,0), L(0.5,0,1,1)};
        case 'W'
            letter = {L(0,1,0.25,0), L(0.25,0,0.5,1), ...
                      L(0.5,1,0.75,0), L(0.75,0,1,1)};
        case 'X'
            letter = {L(0,1,1,0), L(0,0,1,1)};
        case 'Y'
            letter = {L(0,1,0.5,0.5), L(1,1,0.5,0.5), L(0.5,0.5,0.5,0)};
        case 'Z'
            letter = {L(0,1,1,1), L(1,1,0,0), L(0,0,1,0)};

        % ===== NUMBERS =====
        case '0'
            letter = {L(0,0,1,0), L(1,0,1,1), L(1,1,0,1), L(0,1,0,0)};
        case '1'
            letter = {L(0.5,0,0.5,1)};
        case '2'
            letter = {L(0,1,1,1), L(1,1,1,0.5), L(1,0.5,0,0), L(0,0,1,0)};
        case '3'
            letter = {L(0,1,1,1), L(1,1,1,0), L(0,0.5,1,0.5), L(0,0,1,0)};
        case '4'
            letter = {L(0,1,0,0.5), L(0,0.5,1,0.5), L(1,1,1,0)};
        case '5'
            letter = {L(1,1,0,1), L(0,1,0,0.5), L(0,0.5,1,0.5), ...
                      L(1,0.5,1,0), L(1,0,0,0)};
        case '6'
            letter = {L(1,1,0,0.5), L(0,0.5,0,0), L(0,0,1,0), ...
                      L(1,0,1,0.5), L(1,0.5,0,0.5)};
        case '7'
            letter = {L(0,1,1,1), L(1,1,0.5,0)};
        case '8'
            letter = {L(0,0,1,0), L(1,0,1,1), L(1,1,0,1), ...
                      L(0,1,0,0), L(0,0.5,1,0.5)};
        case '9'
            letter = {L(1,0.5,1,1), L(1,1,0,1), L(0,1,0,0.5), L(0,0.5,1,0.5)};

        otherwise
            letter = {};
    end

    idx = [];
    for k = 1:numel(letter)
        seg = letter{k};
        seg(:,1) = seg(:,1)*scale + xStart + xOffset;
        seg(:,3) = seg(:,3)*scale + zBase;
        paths{end+1} = seg;
        idx(end+1) = numel(paths);
    end

    letterIdx{end+1} = idx;
    xOffset = xOffset + spacing;
end
end