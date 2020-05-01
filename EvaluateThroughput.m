function val = EvaluateThroughput()
global NLP_
Nfe = NLP_.Nfe;
load x.txt; load y.txt; load theta.txt;
Nv = length(x) / Nfe; x = reshape(x',Nfe,Nv)'; y = reshape(y',Nfe,Nv)'; theta = reshape(theta',Nfe,Nv)';
global vehicle_geometrics_ intersection xyt_graph_search_
RW = intersection.road_width;
r2p = vehicle_geometrics_.r2p;
f2p = vehicle_geometrics_.f2p;
R = vehicle_geometrics_.radius;
ind_min = Nfe;
ind_max = 1;
for ii = 1 : Nfe
    for jj = 1 : Nv
        xr = x(jj,ii) + r2p * cos(theta(jj,ii));
        yr = y(jj,ii) + r2p * sin(theta(jj,ii));
        xf = x(jj,ii) + f2p * cos(theta(jj,ii));
        yf = y(jj,ii) + f2p * sin(theta(jj,ii));
        if (IsVehicleTouchingIntersection(xr, yr, xf, yf, RW, R))
            ind_min = min(ii, ind_min);
            ind_max = max(ii, ind_max);
        end
    end
end
val = xyt_graph_search_.t_max / Nfe * (ind_max - ind_min);
end

function is_touching = IsVehicleTouchingIntersection(xr, yr, xf, yf, RW, R)
unit_length = RW - R;
if (((abs(xr) < unit_length)&&(abs(yr) < unit_length))||((abs(xf) < unit_length)&&(abs(yf) < unit_length)))
    is_touching = 1;
else
    is_touching = 0;
end
end