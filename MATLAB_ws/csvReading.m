function Data = csvReading(FileFolderPath)
% Read data from csv and save variables

data_read = readmatrix(FileFolderPath);

Time = data_read(:, 1);
Time = Time - Time(1);
Poses_x = data_read(:, 2);
Poses_y = data_read(:, 3);
Poses_z = data_read(:, 4);
Mediciones = data_read(:, end);


Data = [Time, Poses_x, Poses_y, Poses_z, Mediciones];

end