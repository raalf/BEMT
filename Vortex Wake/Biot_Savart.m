%---------------------------------------
% Biot_Savart Function (Semi-Vectorized)
%---------------------------------------
% Calculates the induced velocity on a series of points 
% caused by a corresponding series of vortex segment filament structure(s).
% Where P is an array of query points and s1,s2 represent a combination of
% vortex segments for which will be calculated for each P
%
%
% P: Query points array, [x,y,z]
% s1: Vortex segment starting points array, [x,y,z]
% s2: Vortex segment ending points array, [x,y,z]
% gamma: Circulation of vortex segment. [mx1], column vector

 
function [q12_P]= Biot_Savart(P,s1,s2,gamma)
% Loop over points P
    for i = 1:size(P,1)
            r1      =   P(i,:)-s1;                                        % Length between P and start of segment
            r2      =   P(i,:)-s2;                                        % Length between P and end of segment
            r0      =   r1-r2;
            
            d       =   sqrt(sum(abs(cross(r1,r2,2)).^2,2))./sqrt(sum(abs(r0).^2,2));
            
            cosB1   =   dot(r0,r1,2)./(sqrt(sum(abs(r0).^2,2)).*sqrt(sum(abs(r1).^2,2))); % B1 is the angle between r1 and line segment
            cosB2   =   dot(r0,r2,2)./(sqrt(sum(abs(r0).^2,2)).*sqrt(sum(abs(r2).^2,2))); % B2 is the angle between r2 and line segment

            qtheta  =   gamma./(4.*pi.*d).*(cosB1-cosB2);              %Velocity induced in P by vortex segment
            q12     =   bsxfun(@rdivide,bsxfun(@times,cross(r1,r2,2),qtheta),sqrt(sum(abs(cross(r1,r2,2)).^2,2))); %Velocity induced in P by vortex segment with direction
        
        q12_P(i,:) = sum(q12,1); % q12 wrt point P.
    end
end