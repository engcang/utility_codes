% 현재 디렉토리 내 모든 폴더들 검색
folders = dir('*'); % 'prefix'로 시작하는 폴더들 검색
n_folders = length(folders);

% 각 폴더에 대해 반복
for i = 1:n_folders
    folder_name = folders(i).name;
    velocity_file = fullfile(folder_name, 'velocity.csv');
    acc_bias_file = fullfile(folder_name, 'acc_bias.csv');
    
    if isfile(velocity_file) && isfile(acc_bias_file)
        % velocity.csv 파일 읽기
        velocity_data = readtable(velocity_file);
        time_vel = velocity_data.Time - velocity_data.Time(1);
        vel_x = velocity_data.vel_x;
        vel_y = velocity_data.vel_y;
        vel_z = velocity_data.vel_z;
        
        % acc_bias.csv 파일 읽기
        acc_bias_data = readtable(acc_bias_file);
        time_acc = acc_bias_data.Time - acc_bias_data.Time(1);
        acc_x = acc_bias_data.acc_bias_x;
        acc_y = acc_bias_data.acc_bias_y;
        acc_z = acc_bias_data.acc_bias_z;
        
        % 서브플롯 생성
        figure;
        
        % Velocity plot
        subplot(2,1,1);
        plot(time_vel, vel_x, 'r', 'DisplayName', 'vel_x'); hold on;
        plot(time_vel, vel_y, 'g', 'DisplayName', 'vel_y');
        plot(time_vel, vel_z, 'b', 'DisplayName', 'vel_z');
        title(['Velocity - ' folder_name]);
        xlabel('Time');
        ylabel('Velocity');
        legend show;
        hold off;
        
        % Acceleration bias plot
        subplot(2,1,2);
        plot(time_acc, acc_x, 'r', 'DisplayName', 'acc_x'); hold on;
        plot(time_acc, acc_y, 'g', 'DisplayName', 'acc_y');
        plot(time_acc, acc_z, 'b', 'DisplayName', 'acc_z');
        title(['Acceleration Bias - ' folder_name]);
        xlabel('Time');
        ylabel('Acceleration Bias');
        legend show;
        hold off;
        
    else
        disp(['파일이 없습니다: ' folder_name]);
    end
end
