function [ q_out ] = normalizeQuat( q_in )
%NORMALIZEQUAT normalizes the quaternion
	q_out = q_in/norm(q_in);
end

