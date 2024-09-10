close all;

% Get a list of all folders that start with 'bundle_'
folders = dir('async_*');

for i = 1:length(folders)
    % Construct the folder path
    folderPath = fullfile(folders(i).folder, folders(i).name);
    
    % Define the file names
    fileX = fullfile(folderPath, 'localizability_x.csv');
    fileY = fullfile(folderPath, 'localizability_y.csv');
    fileZ = fullfile(folderPath, 'localizability_z.csv');
    
    % Check if all files exist
    if exist(fileX, 'file') == 2 && exist(fileY, 'file') == 2 && exist(fileZ, 'file') == 2
        % Read the CSV files with original column headers
        dataX = readtable(fileX, 'VariableNamingRule', 'preserve');
        dataY = readtable(fileY, 'VariableNamingRule', 'preserve');
        dataZ = readtable(fileZ, 'VariableNamingRule', 'preserve');
        
        % Extract the time and localizability data
        time = dataX.Time - dataX.Time(1);
        localizabilityX = dataX.("Localizability X");
        localizabilityY = dataY.("Localizability Y");
        localizabilityZ = dataZ.("Localizability Z");
        
        % Calculate the norm of the localizability vector
        localizabilityNorm = sqrt(localizabilityX.^2 + localizabilityY.^2 + localizabilityZ.^2);
        
        % Calculate the minimum and maximum values among X, Y, Z
        localizabilityMin = min([localizabilityX, localizabilityY, localizabilityZ], [], 2);
        localizabilityMax = max([localizabilityX, localizabilityY, localizabilityZ], [], 2);
        
        % Plot the data
        figure;
        % plot(time, localizabilityX, 'r', 'LineWidth', 1.5, 'DisplayName', 'Localizability X');
        hold on;
        % plot(time, localizabilityY, 'g', 'LineWidth', 1.5, 'DisplayName', 'Localizability Y');
        % plot(time, localizabilityZ, 'b', 'LineWidth', 1.5, 'DisplayName', 'Localizability Z');
        % plot(time, localizabilityNorm, 'k--', 'LineWidth', 2, 'DisplayName', 'Localizability Norm');
        plot(time, localizabilityMin, 'm:', 'LineWidth', 2, 'DisplayName', 'Localizability Min');
        % plot(time, localizabilityMax, 'c:', 'LineWidth', 2, 'DisplayName', 'Localizability Max');
        hold off;
        
        % Add title and labels
        folderNameEscaped = strrep(folders(i).name, '_', '\_'); % Escape underscores in the folder name for the title
        title(['Localizability in Folder: ', folderNameEscaped], 'Interpreter', 'tex', 'FontSize', 14);
        xlabel('Time', 'FontSize', 12);
        ylabel('Localizability', 'FontSize', 12);
        legend('show', 'Location', 'best');
        grid on;
        
        % Save the figure if needed
        % saveas(gcf, fullfile(folderPath, 'localizability_plot.png'));
    else
        warning('Some localizability files are missing in folder: %s', folders(i).name);
    end
end
