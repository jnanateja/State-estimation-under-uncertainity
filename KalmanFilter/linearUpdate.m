function [x, P] = linearUpdate(x, P, y, H, R)
%LINEARPREDICTION calculates mean and covariance of predicted state
%   density using a linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   y           [m x 1] Measurement
%   H           [m x n] Measurement model matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%
S=H*P*transpose(H)+R; %predicted covariance
K=P*transpose(H)*(S)^-1; %Kalman Gain
V=y-H*x; %Innovation
x=x+K*V; %updated state mean
P=P-K*S*transpose(K); %updated state covariance

end