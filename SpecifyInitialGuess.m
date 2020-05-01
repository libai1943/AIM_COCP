function [x, y, theta, vv, aa, pp, ww, xxff, yyff, xxrr, yyrr] = SpecifyInitialGuess(x, y, theta)
[Nv, Nfe] = size(x);
vv = zeros(Nv, Nfe);
aa = zeros(Nv, Nfe);
pp = zeros(Nv, Nfe);
ww = zeros(Nv, Nfe);
xxff = zeros(Nv, Nfe);
yyff = zeros(Nv, Nfe);
xxrr = zeros(Nv, Nfe);
yyrr = zeros(Nv, Nfe);
global vehicle_geometrics_
for ii = 1 : Nv
    [vv(ii,1:Nfe), aa(ii,1:Nfe), pp(ii,1:Nfe), ww(ii,1:Nfe)] = FulfillProfiles(x(ii,1:Nfe), y(ii,1:Nfe), theta(ii,1:Nfe));
    xxff(ii,1:Nfe) = x(ii,1:Nfe) + vehicle_geometrics_.f2p .* cos(theta(ii,1:Nfe));
    yyff(ii,1:Nfe) = y(ii,1:Nfe) + vehicle_geometrics_.f2p .* sin(theta(ii,1:Nfe));
    xxrr(ii,1:Nfe) = x(ii,1:Nfe) + vehicle_geometrics_.r2p .* cos(theta(ii,1:Nfe));
    yyrr(ii,1:Nfe) = y(ii,1:Nfe) + vehicle_geometrics_.r2p .* sin(theta(ii,1:Nfe));
end
WriteInitialGuessForNLP(x, y, theta, vv, aa, pp, ww, xxff, yyff, xxrr, yyrr);
end

function [v, a, phy, w] = FulfillProfiles(x, y, theta)
global xyt_graph_search_
Nfe = length(x);
dt = xyt_graph_search_.t_max / Nfe;
% Judge velocity direction
vdr = zeros(1, Nfe);
for ii = 2 : (Nfe - 1)
    addtion = (x(ii+1) - x(ii)) * cos(theta(ii)) + (y(ii+1) - y(ii)) * sin(theta(ii));
    if (addtion > 0)
        vdr(ii) = 1;
    else
        vdr(ii) = -1;
    end
end
v = zeros(1, Nfe);
a = zeros(1, Nfe);
for ii = 2 : Nfe
    v(ii) = vdr(ii) * sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
end
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end
phy = zeros(1, Nfe);
w = zeros(1, Nfe);
global vehicle_kinematics_ vehicle_geometrics_
phy_max = vehicle_kinematics_.phy_max;
w_max = vehicle_kinematics_.w_max;
for ii = 2 : (Nfe-1)
    phy(ii) = atan((theta(ii+1) - theta(ii)) * vehicle_geometrics_.wheelbase / (dt * v(ii)));
    if (phy(ii) > phy_max)
        phy(ii) = phy_max;
    elseif (phy(ii) < -phy_max)
        phy(ii) = -phy_max;
    end
end
for ii = 2 : (Nfe-1)
    w(ii) = (phy(ii+1) - phy(ii)) / dt;
    if (w(ii) > w_max)
        w(ii) = w_max;
    elseif (w(ii) < -w_max)
        w(ii) = -w_max;
    end
end
end

function WriteInitialGuessForNLP(x, y, theta, v, a, phy, w, xf, yf, xr, yr)
[Nv, Nfe] = size(x);
delete('IG.INIVAL');
fid = fopen('IG.INIVAL', 'w');
for ii = 1 : Nv
    for jj = 1 : Nfe
        fprintf(fid, 'let x[%g,%g] := %f;\r\n', ii, jj, x(ii,jj));
        fprintf(fid, 'let y[%g,%g] := %f;\r\n', ii, jj, y(ii,jj));
        fprintf(fid, 'let theta[%g,%g] := %f;\r\n', ii, jj, theta(ii,jj));
        fprintf(fid, 'let v[%g,%g] := %f;\r\n', ii, jj, v(ii,jj));
        fprintf(fid, 'let phy[%g,%g] := %f;\r\n', ii, jj, phy(ii,jj));

        fprintf(fid, 'let xf[%g,%g] := %f;\r\n', ii, jj, xf(ii,jj));
        fprintf(fid, 'let yf[%g,%g] := %f;\r\n', ii, jj, yf(ii,jj));
        fprintf(fid, 'let xr[%g,%g] := %f;\r\n', ii, jj, xr(ii,jj));
        fprintf(fid, 'let yr[%g,%g] := %f;\r\n', ii, jj, yr(ii,jj));
        
        fprintf(fid, 'let a[%g,%g] := %f;\r\n', ii, jj, a(ii,jj));
        fprintf(fid, 'let w[%g,%g] := %f;\r\n', ii, jj, w(ii,jj));
    end
end
fclose(fid);
end