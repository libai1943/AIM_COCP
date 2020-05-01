function UpdateObstacleLayers(x, y, theta)
global original_obstacle_layers_ vehicle_geometrics_ xyt_graph_search_
xr = x + vehicle_geometrics_.r2p * cos(theta);
yr = y + vehicle_geometrics_.r2p * sin(theta);
xf = x + vehicle_geometrics_.f2p * cos(theta);
yf = y + vehicle_geometrics_.f2p * sin(theta);

for ii = 1 : length(x)
    x = xr(ii) + xyt_graph_search_.Rcos_ang;
    y = yr(ii) + xyt_graph_search_.Rsin_ang;
    x = [x, xf(ii) + xyt_graph_search_.Rcos_ang];
    y = [y, yf(ii) + xyt_graph_search_.Rsin_ang];
    original_obstacle_layers_{1,ii}.x = [original_obstacle_layers_{1,ii}.x, x];
    original_obstacle_layers_{1,ii}.y = [original_obstacle_layers_{1,ii}.y, y];
end
end