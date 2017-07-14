function [Params, measurements, P_obj_i] = geolocation_generate()

%% Camera Parameters
% Lens: http://computar.com/product/565/H1214FICS
% Camera: https://en.ids-imaging.com/store/ui-1250ml.html
% Calculation: http://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/?answer=17180#post-id-17180
% Or, eq 13.5 in UAV book
P.focal_mm = 12;
P.sensor_width_mm = 7.2;
P.im_w_pix = 640;
P.im_h_pix = 480;
P.focal_pix = (P.focal_mm / P.sensor_width_mm) * P.im_w_pix;
P.ox = P.im_w_pix/2;    % offset from optical axis in x-direction
P.oy = P.im_h_pix/2;    % offset from optical axis in y-direction
P.sx = 90; % scale factor in x (e.g. pixels/mm) [guessed]
P.sy = 75; % scale factor in y (e.g. pixels/mm) [guessed]
P.fov_x = 2*atan(P.im_w_pix/(2*P.focal_mm*P.sx)); % horizontal field of view 
P.fov_y = 2*atan(P.im_h_pix/(2*P.focal_mm*P.sy)); % vertical field of view

%% Inertial Parameters
P.max_pn = 10;
P.min_pn = -10;
P.max_pe = 10;
P.min_pe = -10;
P.max_pd = 0;
P.min_pd = -10;

%% Input Parameters
% UAV position and orientation
pn = -5;
pe = 1;
pd = -10;
phi = deg2rad(0);
theta = deg2rad(0);
psi = deg2rad(0);

% Gimbal orientation
az = deg2rad(0);
el = deg2rad(-60);

% Output Params
Params.pn = pn;
Params.pe = pe;
Params.pd = pd;
Params.phi = phi;
Params.theta = theta;
Params.psi = psi;
Params.az = az;
Params.el = el;
Params.f = P.focal_pix;

% Generate measurements in the camera frame
N = 10;
pts_x = P.im_w_pix.*rand(N,1);
pts_y = P.im_h_pix.*rand(N,1);

figure(1);
scatter(pts_x,pts_y);
title('Measurements on image plane');
axis([0 P.im_w_pix 0 P.im_h_pix]);
set(gca,'YDir','Reverse'); set(gca, 'XAxisLocation', 'top');
a = (1:N)'; b = num2str(a); c = cellstr(b);
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
text(pts_x+dx, pts_y+dy, c);

measurements = [pts_x pts_y];

%% Geolocate
P_obj_i = geolocate(P, measurements, pn, pe, pd, phi, theta, psi, az, el);

%% Draw targets in inertial frame
fov_x = 50;
fov_y = 50;
figure(2), clf;
drawFov(pn, pe, pd, phi, theta, psi, az, el, [], P.fov_x, P.fov_y, 0);
title('UAV Camera Frustum'), grid on, hold on;
axis([P.min_pn P.max_pn P.min_pe P.max_pe P.min_pd P.max_pd])
xlabel('North');
ylabel('East');
zlabel('Down');
view(51,65);
set(gca,'ZDir','Reverse'); set(gca,'YDir','Reverse')
X = P_obj_i(1,:); Y = P_obj_i(2,:); Z = P_obj_i(3,:);
scatter3(X, Y, Z);
a = (1:N)'; b = num2str(a); c = cellstr(b);
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
text(X+dx, Y+dy, c);
end

%% Geolocation Algorithm
function P_obj_i = geolocate(P, meas, pn, pe, pd, phi, theta, psi, az, el)

% How many samples are there?
N = size(meas,1);

% Convert camera measurements to the image plane by removing the ox and oy
% offsets. This puts the optical axis through the center of the camera.
eps_x = meas(:,1) - P.ox;
eps_y = meas(:,2) - P.oy;

% -------------------------------------------------------------------------
% Equation 13.9
% -------------------------------------------------------------------------
F = sqrt(P.focal_pix^2 + eps_x.^2 + eps_y.^2);
ell_c_unit = [eps_x eps_y repelem(P.focal_pix,N)'] ./ repmat(F,1,3);
% =========================================================================

% compute rotation from vehicle to camera coordinates
R_v_to_c = R_g_to_c() * R_b_to_g(az,el) * R_v_to_b(phi,theta,psi);

% rotate all unit vectors into the vehicle frame
ell_i_unit = R_v_to_c' * ell_c_unit';

% compute distance from UAV to each target
h = -pd;
L = h ./ ([0;0;1]'*ell_i_unit);

% scale target unit vectors by their distance
ell_i = repmat(L,[3,1]) .* ell_i_unit;

% add UAV vector to target vectors to get target inertial coordinates
P_obj_i = repmat([pn;pe;pd],[1,size(ell_i,2)]) + ell_i;

end

  
%% Rotation Matrices
% rotation from vehicle to body coordinates
function R = R_v_to_b(phi,theta,psi)

    R_v_to_v1 = [ cos(psi)  sin(psi)  0
                 -sin(psi)  cos(psi)  0
                     0         0      1];
    
    R_v1_to_v2 = [ cos(theta)  0 -sin(theta)
                      0        1     0
                   sin(theta)  0  cos(theta)];
    
    R_v2_to_b = [ 1     0         0
                  0  cos(phi)  sin(phi)
                  0 -sin(phi)  cos(phi)];
    
    R = R_v2_to_b * R_v1_to_v2 * R_v_to_v1;
end


% rotation from body to gimbal coordinates
function R = R_b_to_g(az,el)

    R_b_to_g1 = [ cos(az)  sin(az)  0
                 -sin(az)  cos(az)  0
                     0        0     1];

    R_g1_to_g = [ cos(el)  0 -sin(el)
                     0     1     0
                  sin(el)  0  cos(el)];

    R = R_g1_to_g * R_b_to_g1;
end

% transform from gimbal to camera coordinates
function R = R_g_to_c()
    R = [0 1 0
         0 0 1
         1 0 0];
end

% function to draw camera fov
function handle = drawFov(pn, pe, pd, phi, theta, psi, az, el, handle, fov_x, fov_y, t)

    % rotation from gimbal to camera
    R_g2c = [ 0  1  0
              0  0  1
              1  0  0];
                           
    % define unit vectors along fov in the camera frame
    % here R_b_to_g is not actually rotating anything from gimbal to body,
    % it is just a convenient rotation for rotating the optical axis vector
    % to the corners of the FOV
    pts = [ (R_b_to_g( fov_x/2, fov_y/2)'*R_g2c'*[0;0;1])'     % top-right
            (R_b_to_g(-fov_x/2, fov_y/2)'*R_g2c'*[0;0;1])'     % top-left
            (R_b_to_g(-fov_x/2,-fov_y/2)'*R_g2c'*[0;0;1])'     % bot-left
            (R_b_to_g( fov_x/2,-fov_y/2)'*R_g2c'*[0;0;1])' ]'; % bot-right
        
    % transform from gimbal coordinates to the vehicle coordinates
    pts = R_v_to_b(phi,theta,psi)'*R_b_to_g(az,el)'*pts;

    % first vertex is at center of MAV vehicle frame
    Vert = [pn, pe, pd]; 
    
    % project field of view lines onto ground plane and make correction
    % when the projection is above the horizon
    for i = 1:4
        
        % alpha is the angle that the field-of-view line makes with horizon
        alpha = atan2(pts(3,i),norm(pts(1:2,i)));
        
        if alpha > 0
            
            % fov line is below horizon and intersects ground plane
            Vert = [...
                Vert
                [pn-pd*pts(1,i)/pts(3,i), pe-pd*pts(2,i)/pts(3,i), 0]
                ];
            
        elseif alpha < 0
            
            % fov line is above horizon and intersects some high plane
            Vert = [...
                Vert
                [pn+pd*pts(1,i)/pts(3,i), pe+pd*pts(2,i)/pts(3,i), pd*2]
                ];
            
        else

            % fov line exactly on horizon and intersects no plane
            Vert = [...
                Vert
                [pn+999*cos(fov_x), pe+999*sin(fov_x), pd]
                ];
            
        end
    end

    Faces = [ 1  1  2  3    % top face
              1  1  2  5    % right face
              1  1  5  4    % bottom face
              1  1  4  3    % left face
              2  3  4  5 ]; % footprint face

    colors = [[1 1 1]; [1 1 1]; [1 1 1]; [1 1 1]; [0 1 0]];

  if t == 0
    handle = patch('Vertices', Vert, 'Faces', Faces,...
                 'FaceVertexCData',colors,'FaceColor','flat',...
                 'FaceAlpha',0.05);
  else
    set(handle,'Vertices',Vert,'Faces',Faces);
  end
  
end