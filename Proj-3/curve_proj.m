close all; clear; clc;
load S_letter_path.mat

%% Convert curve to YZ plane
Sls(3,:) = Sls(2,:)+2;
Sls(2,:) = Sls(1,:)-2;
Sls(1,:) = 0;

%% Projection Parameter
O = [0; 0; 2];
N = [-2; 0; 2];
%% Plot Sphere and Original curve
plot3(Sls(1,:),Sls(2,:),Sls(3,:), 'linewidth', 3);
hold on;
r = 2;
[X, Y, Z] = sphere;
X = X*r+O(1,1);
Y = Y*r+O(2,1);
Z = Z*r+O(3,1);
surf(X, Y, Z);
alpha 0.4;
view(120,10);
grid();
axis(r*[-1 1 -1 1 0 2]);axis('square');
xlabel('x');ylabel('y');zlabel('z');
title("Sterographic Projection of Letter S Path (Yuxin Hu)");
%%
Sls_p = [0; 0; 0];
for i = 1:length(Sls)
    a = 2*((N - O).')*(N - Sls(:, i))/(norm(N - Sls(:, i))^2);
    P = a*(Sls(:, i) - N) + N;
    Sls_p(:, i) = P;
end
plot3(Sls_p(1,:),Sls_p(2,:),Sls_p(3,:), 'rx', 'linewidth', 3);