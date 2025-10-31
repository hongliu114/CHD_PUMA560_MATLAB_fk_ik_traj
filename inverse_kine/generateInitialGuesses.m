function initial_guesses = generateInitialGuesses(target_pos, qlims)
% 生成智能的初始猜测值
    
    px = target_pos(1);
    py = target_pos(2);
    pz = target_pos(3);
    
    initial_guesses = [];
    
    % 基于几何的初始猜测
    theta1_candidates = [atan2(py, px), atan2(py, px) + pi, atan2(py, px) - pi];
    
    % 过滤theta1候选值
    valid_theta1 = [];
    for theta1 = theta1_candidates
        if theta1 >= qlims(1,1) && theta1 <= qlims(1,2)
            valid_theta1 = [valid_theta1, theta1];
        end
    end
    
    % 为每个有效的theta1生成初始猜测
    for theta1 = valid_theta1
        % theta2的几个候选值
        theta2_candidates = [-pi/4, 0, pi/4, pi/2];
        
        for theta2 = theta2_candidates
            if theta2 >= qlims(2,1) && theta2 <= qlims(2,2)
                % theta3的几个候选值
                theta3_candidates = [-pi/2, -pi/4, 0, pi/4, pi/2];
                
                for theta3 = theta3_candidates
                    if theta3 >= qlims(3,1) && theta3 <= qlims(3,2)
                        % theta4, theta5, theta6的组合
                        theta456_combinations = [
                            [0, 0, 0];
                            [pi/2, 0, 0];
                            [-pi/2, 0, 0];
                            [0, pi/2, 0];
                            [0, -pi/2, 0];
                            [0, 0, pi/2];
                            [0, 0, -pi/2];
                            [pi/4, pi/4, 0];
                            [-pi/4, -pi/4, 0];
                        ];
                        
                        for k = 1:size(theta456_combinations, 1)
                            theta4 = theta456_combinations(k, 1);
                            theta5 = theta456_combinations(k, 2);
                            theta6 = theta456_combinations(k, 3);
                            
                            if theta4 >= qlims(4,1) && theta4 <= qlims(4,2) && ...
                               theta5 >= qlims(5,1) && theta5 <= qlims(5,2) && ...
                               theta6 >= qlims(6,1) && theta6 <= qlims(6,2)
                                
                                initial_guesses = [initial_guesses; 
                                                 theta1, theta2, theta3, theta4, theta5, theta6];
                            end
                        end
                    end
                end
            end
        end
    end
    
    % 添加一些随机的初始猜测
    n_random = 50;
    for i = 1:n_random
        theta_random = generateRandomInitialGuess(qlims);
        initial_guesses = [initial_guesses; theta_random];
    end
end