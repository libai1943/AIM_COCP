%  MATLAB Source Codes for article "Autonomous Intersection Management over
%  Continuous Space: A Microscopic and Precise Solution via Computational
%  Optimal Control", IFAC-PapersOnline, 2020.
%  Copyright (C) 2020 Bai Li
%  2020.04.30
% ==============================================================================
%  Users of the source codes are requested to cite the following papers.
%  a) Bai Li, Youmin Zhang, Ning Jia, and Xiaoyan Peng, "Autonomous
%  Intersection Management over Continuous Space: A Microscopic and Precise
%  Solution by Computational Optimal Control", IFAC 2020, accepted, 2020.
%  b) Bai Li, and Youmin Zhang, "Fault-tolerant cooperative motion planning
%  of connected and automated vehicles at a signal-free and lane-free
%  intersection." IFAC-PapersOnLine 51(24), 60-67, 2018.
%  c) Bai Li, et al., Incrementally constrained dynamic optimization: A
%  computational framework for lane change motion planning of connected and
%  automated vehicles. Journal of Intelligent Transportation Systems,
%  23(6), 557-568, 2019.
% ==============================================================================
clear; close all; clc;
for case_id = 1 : 50
    tic
    % % Parameters
    global vehicle_geometrics_
    vehicle_geometrics_.wheelbase = 2.8;
    vehicle_geometrics_.front_hang = 0.96;
    vehicle_geometrics_.rear_hang = 0.929;
    vehicle_geometrics_.width = 1.942;
    vehicle_geometrics_.length = vehicle_geometrics_.wheelbase + vehicle_geometrics_.front_hang + vehicle_geometrics_.rear_hang;
    vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.length, 0.5 * vehicle_geometrics_.width);
    vehicle_geometrics_.r2p = 0.25 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;
    vehicle_geometrics_.f2p = 0.75 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;
    global vehicle_kinematics_
    vehicle_kinematics_.v_max = 25;
    vehicle_kinematics_.v_common = 20;
    vehicle_kinematics_.a_max = 2.0;
    vehicle_kinematics_.phy_max = 0.7;
    vehicle_kinematics_.w_max = 0.3;
    global environment_scale_
    environment_scale_.x_min = -250;
    environment_scale_.x_max = 250;
    environment_scale_.y_min = -250;
    environment_scale_.y_max = 250;
    environment_scale_.x_scale = environment_scale_.x_max - environment_scale_.x_min;
    environment_scale_.y_scale = environment_scale_.y_max - environment_scale_.y_min;
    global intersection
    intersection.road_width = 12;
    intersection.num_lanes = 3;
    global xyt_graph_search_
    xyt_graph_search_.t_max = 10.0;
    xyt_graph_search_.num_nodes_t = 200;
    xyt_graph_search_.resolution_t = xyt_graph_search_.t_max / xyt_graph_search_.num_nodes_t;
    xyt_graph_search_.resolution_x = 1; xyt_graph_search_.resolution_y = 1;
    xyt_graph_search_.num_nodes_x = ceil(environment_scale_.x_scale / xyt_graph_search_.resolution_x) + 1;
    xyt_graph_search_.num_nodes_y = ceil(environment_scale_.y_scale / xyt_graph_search_.resolution_y) + 1;
    xyt_graph_search_.multiplier_for_H = 2.0;
    xyt_graph_search_.weight_for_time = 5;
    xyt_graph_search_.max_cycle = 2000;
    xyt_graph_search_.pelnaty_for_halt = 20;
    ang = linspace(0, 2 * pi, 200);
    xyt_graph_search_.Rcos_ang = vehicle_geometrics_.radius .* cos(ang);
    xyt_graph_search_.Rsin_ang = vehicle_geometrics_.radius .* sin(ang);
    global NLP_
    NLP_.Nfe = 100;
    NLP_.tf = xyt_graph_search_.t_max;
    global boundary_configs_ obstacles_
    Nv = 24;
    global A1 A2 A3 B1 B2 B3 C1 C2 C3 D1 D2 D3
    
    load(['Benchmarks\', num2str(case_id)]);
    [index_flag, ranklist] = SpecifyRanklist(Nv);
    
    global original_obstacle_layers_
    original_obstacle_layers_ = GenerateOriginalObstacleLayers();
    backup_original_obstacle_layers = original_obstacle_layers_;
    
    decision_x = zeros(Nv, NLP_.Nfe);
    decision_y = zeros(Nv, NLP_.Nfe);
    decision_theta = zeros(Nv, NLP_.Nfe);
    for ii = 1 : Nv
        iv = ranklist(ii);
        [x, y, theta] = SearchTrajectoryInXYTGraph(boundary_configs_{1,iv});
        decision_x(iv, 1 : NLP_.Nfe) = ResampleProfile(x, NLP_.Nfe);
        decision_y(iv, 1 : NLP_.Nfe) = ResampleProfile(y, NLP_.Nfe);
        decision_theta(iv, 1 : NLP_.Nfe) = ResampleProfile(theta, NLP_.Nfe);
        UpdateObstacleLayers(x, y, theta);
    end
    
    WriteBoundaryValuesAndBasicParams(Nv, NLP_.Nfe);
    SpecifyInitialGuess(decision_x, decision_y, decision_theta);
    !ampl rr0.run
    load opti_flag.txt
    
    successive_failure_times = 0;
    for ii = 1 : 2: NLP_.Nfe
        delete('TimeLine');
        fid = fopen('TimeLine', 'w');
        fprintf(fid, '1 %g\r\n', ii);
        fclose(fid);
        !ampl rr.run
        load opti_flag.txt
        if (opti_flag == 1)
            successive_failure_times = 0;
        else
            successive_failure_times = successive_failure_times + 1;
        end
        
        if (opti_flag && (~CheckCollisions()))
            break;
        end
        if ((opti_flag == -1)||(toc >= 3600))
            break;
        end
        if (successive_failure_times >= 5)
            break;
        end
    end
    if (opti_flag && (~CheckCollisions()))
        ProduceVideo(case_id);
        disp('AIM Solution Succeeded.');
        disp(['Throughput = ', num2str(EvaluateThroughput())]);
    else
        disp('AIM Solution Failed!');
    end
    disp(['CPU Time = ', num2str(toc)]);
end