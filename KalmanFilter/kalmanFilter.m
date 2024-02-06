function [X, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R)
%KALMANFILTER Filters measurements sequence Y using a Kalman filter. 
%
%Input:
%   Y           [m x N] Measurement sequence
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   A           [n x n] State transition matrix
%   Q           [n x n] Process noise covariance
%   H           [m x n] Measurement model matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   x           [n x N] Estimated state vector sequence
%   P           [n x n x N] Filter error convariance
%

N = size(Y,2);

[X(:,1), P(:,:,1)] = linearPrediction(x_0, P_0, A, Q);
[X(:,1), P(:,:,1)] = linearUpdate(X(:,1), P(:,:,1), Y(:,1), H, R);

for i=2:N
    [X(:,i), P(:,:,i)] = linearPrediction(X(:,i-1), P(:,:,i-1), A, Q);
    [X(:,i), P(:,:,i)] = linearUpdate(X(:,i), P(:,:,i), Y(:,i), H, R);
end


end
