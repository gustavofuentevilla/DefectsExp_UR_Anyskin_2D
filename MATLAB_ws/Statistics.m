close all
clear
clc

%% Statistics

% For TESTS1

% N100_times = [15.363; 18.116; 16.481; 16.9; 17.405; 23.21; 13.825;...
%               18.114; 21.89; 24.737; 19.672; 13.986; 18.822; 14.695;...
%               16.224; 19.02; 28.544; 11.208; 11.431; 20.240; 11.591;...
%               16.037; 20.529; 21.438; 16.284; 22.417; 24.833; 15.329;...
%               12.656; 14.304; 16.597; 25.580; 17.771; 16.617; 21.9;...
%               25.171; 14.911; 20.208; 21.418; 16.224; 14.940; 11.949;...
%               14.318; 17.561; 14.516; 13.625; 14.172; 13.857; 12.043;...
%               19.140; 18.829; 10.709; 18.222; 13.276; 17.768; 14.957;...
%               17.168; 15.051; 15.279; 13.302; 14.856; 15.562; 26.231;...
%               19.774; 13.161; 15.526; 15.717; 15.094; 14.55; 23.234;...
%               24.36; 25.644; 24.966; 13.403; 16.56; 11.378; 13.594;...
%               15.723; 20.126; 18.908; 19.139; 12.703; 24.047; 14.71;...
%               21.72; 18.686; 17.868; 16.117; 21.716; 13.345; 14.946;...
%               11.824; 14.804; 22.114; 16.937; 11.061; 22.009; 17.003;...
%               25.82; 22.897];
% 
% N150_times = [42.351; 34.117; 45.876; 55.419; 34.857; 25.501; 32.264;...
%               31.444; 37.323; 27.238; 31.137; 45.544; 56.037; 40.11;...
%               44.287; 45.091; 33.231; 40.982; 39.944; 38.139; 27.195;...
%               45.457; 61.421; 34.742; 32.035; 29.583; 35.354; 35.694;...
%               42.371; 28.27; 27.638; 28.795; 72.203; 38.53; 38.556;...
%               44.486; 33.676; 29.622; 37.053; 40.511; 34.661; 40.352;...
%               40.69; 28.859; 35.634; 27.024; 29.754; 41.226; 30.027;...
%               40.946; 51.628; 34.597; 40.087; 65.184; 62.026; 45.708;...
%               28.251; 29.522; 53.310; 48.643; 46.672; 32.729; 41.227;...
%               38.256; 33.986; 42.54; 37.28; 50.377; 28.012; 31.19;...
%               43.528; 29.028; 45.698];

% Calculate the mean and standard deviation for both datasets
% meanN100 = mean(N100_times);
% stdN100 = std(N100_times);
% meanN150 = mean(N150_times);
% stdN150 = std(N150_times);

%% Mean error distance between estimated and real defect locations
% Solo hay que cambiar el path (N100 o N150 / 1, 2 o 3 Def)

close all
clear
clc

% N100 = 10 sec; N150 = 15 sec; N200 = 20 sec
planner = "N100";

% Number of defects
def = 1;

% Number of tests per case
totalOut = 10;

% Buffers for:
nbIter = zeros(totalOut, 1);        % Number of iterations
nbMu = zeros(totalOut, 1);          % Number of found defects
T_traj = cell(totalOut, 1);         % Time spend computing ergodic trajectory
T_PDF = cell(totalOut, 1);          % Time spend computing the PDF
T_iter = cell(totalOut, 1);         % Times per iteration
Mu_buffer = cell(totalOut, 1);
Mu_found_buffer = cell(totalOut, 1);

dblCheck = zeros(totalOut, 1);

for out_i = 1:totalOut
    % Loading file and extract variables
    filename = "Results2/" + planner + "/" + def +...
                "Def/output_" + out_i + ".mat";
    load(filename, "n_iter", "Mu_found", "Estim_sol",...
         "T_ErgC_i", "T_PDF_i", "Mu", "Par_PDF")

    % Save data into a vector
    nbIter(out_i) = n_iter;
    nbMu(out_i) = height(Mu_found);

    % double check
    dblCheck(out_i) = Par_PDF.NoDataIterCounter;

    % Times
    T_traj{out_i} = T_ErgC_i;
    T_PDF{out_i} = T_PDF_i;
    T_iter{out_i} = T_ErgC_i + T_PDF_i;

    % Defect locations
    Mu_buffer{out_i} = Mu;
    Mu_found_buffer{out_i} = Mu_found;
end

% Success rate (Number of tests in which all the defects are found)
successRate = sum(nbMu == def, 'all') / numel(nbMu) * 100;

% Calculate the average values and standard deviations of iterations
avgIter = mean(nbIter, "all");
stdIter = std(nbIter, 0, "all");

% Time per iteration
TimePerIter = [];
for out_i = 1:totalOut
    TimePerIter = [TimePerIter;
                   T_iter{out_i}(1:nbIter(out_i))];
end

avgTimePerIter = mean(TimePerIter);
stdTimePerIter = std(TimePerIter);

% Mean error distance between estimated and real defect locations
e_centers = [];
for out_i = 1:totalOut
    % If found defects buffer its not empty
    if ~isempty(Mu_found_buffer{out_i})
        k_idx = dsearchn(Mu_buffer{out_i},...
                         Mu_found_buffer{out_i});
        Mu_tmp = Mu_buffer{out_i}(k_idx, :);
        e_centers = [e_centers;...
                     sum((Mu_tmp - Mu_found_buffer{out_i}).^2, 2).^0.5];
    end
end

avgLocationError = mean(e_centers);
stdLocationError = std(e_centers);


