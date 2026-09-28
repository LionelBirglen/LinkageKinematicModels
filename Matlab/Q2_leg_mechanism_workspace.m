function W = Q2_leg_mechanism_workspace(params, thetaA_lim, rho_lim, n)
%Q2_LEG_MECHANISM_WORKSPACE  Workspace of point P of the leg mechanism
%
%   W = Q2_LEG_MECHANISM_WORKSPACE(PARAMS, THETAA_LIM, RHO_LIM)
%   W = Q2_LEG_MECHANISM_WORKSPACE(PARAMS, THETAA_LIM, RHO_LIM, N)
%
%   Computes the set of positions of point P (on body I-J, defined by
%   PARAMS.IP and PARAMS.JIP) reachable for all inputs
%       THETAA_LIM(1) <= thetaA <= THETAA_LIM(2)   (rad)
%       RHO_LIM(1)    <= rho    <= RHO_LIM(2)
%   for each of the 8 assembly branches of Q2_leg_mechanism_direct_kinematics.
%
%   The input box is sampled on a regular N(1) x N(2) grid (default
%   [91 51]) and Q2_leg_mechanism_direct_kinematics is evaluated at every node, so
%   the result is exactly consistent with the direct kinematics used
%   for drawing. Branch k of W is branch k of Q2_leg_mechanism_direct_kinematics.
%
%   PARAMS : the 17-parameter struct of Q2_leg_mechanism_direct_kinematics
%            (IP = 0 gives the workspace of I).
%
%   W : struct with fields
%     .thetaA_lim, .rho_lim, .n - the requested limits and grid size
%     .thetaA   - 1 x n(1) sampled thetaA values (rad)
%     .rho      - 1 x n(2) sampled rho values
%     .X, .Y    - n(1) x n(2) x 8 coordinates of P (NaN where the branch
%                 does not assemble)
%     .valid    - n(1) x n(2) x 8 logical, branch assembles at node
%     .patch    - 1 x 8 struct array with fields .Vertices (M x 2) and
%                 .Faces (nF x 4): every grid cell whose 4 corners are
%                 valid, mapped to the plane. Ready for
%                 patch('Vertices',..,'Faces',..) to draw the reachable
%                 region of each branch as a filled area.
%     .bbox     - 1 x 8 cell, [xmin xmax ymin ymax] of each branch's
%                 workspace ([] if empty)
%     .nValid   - 1 x 8, number of valid grid nodes per branch
%     .params   - copy of PARAMS (lets a caller check whether a cached
%                 workspace is still up to date)
%     .time     - computation time (s)
%
%   Example:
%       p = struct('AB',150,'AC',90,'BD',120,'CD',100,'FG',80,'CF',90, ...
%                  'BE',50,'GK',50,'GJ',80,'IF',80,'IJ',100,'IP',50, ...
%                  'DCF',deg2rad(80),'GFC',deg2rad(110),'DBE',deg2rad(45), ...
%                  'JGK',deg2rad(30),'JIP',deg2rad(30));
%       W = Q2_leg_mechanism_workspace(p, deg2rad([-180 180]), [50 400]);
%       Q2_leg_mechanism_plot(p,'direct',[deg2rad(-80) 220], ...
%                          struct('workspace',W));
%
%   See also Q2_LEG_MECHANISM_DIRECT_KINEMATICS, Q2_LEG_MECHANISM_PLOT.

if nargin < 4 || isempty(n), n = [91 51]; end
if nargin < 3
    error('Q2_leg_mechanism_workspace:NotEnoughInputs', ...
        'Need at least: params, thetaA_lim, rho_lim.');
end
if numel(thetaA_lim) ~= 2 || numel(rho_lim) ~= 2 || ...
        any(~isfinite([thetaA_lim(:); rho_lim(:)]))
    error('Q2_leg_mechanism_workspace:BadLimits', ...
        'thetaA_lim and rho_lim must be finite 2-element vectors.');
end
n = max(round(n(:).'), 2);
if numel(n) == 1, n = [n n]; end

t0 = tic;
thetaA = linspace(min(thetaA_lim), max(thetaA_lim), n(1));
rho    = linspace(min(rho_lim),    max(rho_lim),    n(2));

nb = 8;
X = nan(n(1), n(2), nb);
Y = nan(n(1), n(2), nb);
V = false(n(1), n(2), nb);

for i = 1:n(1)
    for j = 1:n(2)
        s = Q2_leg_mechanism_direct_kinematics(thetaA(i), rho(j), params);
        for k = 1:min(nb, numel(s))
            if s(k).valid && all(isfinite(s(k).P))
                X(i,j,k) = s(k).P(1);
                Y(i,j,k) = s(k).P(2);
                V(i,j,k) = true;
            end
        end
    end
end

% --- Patch data: grid cells with 4 valid corners, per branch ---------
% Node (i,j) has linear index i + (j-1)*n(1) in each branch slice.
[I, J] = ndgrid(1:n(1)-1, 1:n(2)-1);
c1 = I(:)   + (J(:)-1)*n(1);    % (i  , j  )
c2 = I(:)+1 + (J(:)-1)*n(1);    % (i+1, j  )
c3 = I(:)+1 +  J(:)   *n(1);    % (i+1, j+1)
c4 = I(:)   +  J(:)   *n(1);    % (i  , j+1)

patchData = repmat(struct('Vertices', zeros(0,2), 'Faces', zeros(0,4)), 1, nb);
bbox   = cell(1, nb);
nValid = zeros(1, nb);
for k = 1:nb
    Vk = V(:,:,k);
    nValid(k) = nnz(Vk);
    if nValid(k) == 0, continue; end
    Xk = X(:,:,k);  Yk = Y(:,:,k);
    ok = Vk(c1) & Vk(c2) & Vk(c3) & Vk(c4);
    patchData(k).Vertices = [Xk(:) Yk(:)];
    patchData(k).Faces    = [c1(ok) c2(ok) c3(ok) c4(ok)];
    bbox{k} = [min(Xk(Vk)) max(Xk(Vk)) min(Yk(Vk)) max(Yk(Vk))];
end

W = struct();
W.thetaA_lim = [min(thetaA_lim) max(thetaA_lim)];
W.rho_lim    = [min(rho_lim) max(rho_lim)];
W.n          = n;
W.thetaA     = thetaA;
W.rho        = rho;
W.X          = X;
W.Y          = Y;
W.valid      = V;
W.patch      = patchData;
W.bbox       = bbox;
W.nValid     = nValid;
W.params     = params;
W.time       = toc(t0);
end
