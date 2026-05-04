function [path, costMap, pathLength] = wavefront_planner(mapSize, obs, start, goal, connectedness)
% WAVEFRONT_PLANNER Generates a shortest path, plots results, and calculates cost.
%
% OUTPUTS:
%   pathLength - The total number of steps in the final path.

    % 1. Initialization and Map Building
    rows = mapSize(1);
    cols = mapSize(2);
    costMap = zeros(rows, cols);
    
    % Apply obstacles
    linearIdx = sub2ind([rows, cols], obs(:,1), obs(:,2));
    costMap(linearIdx) = 1;

    % --- SAFETY CHECK ---
    if costMap(start(1), start(2)) == 1 || costMap(goal(1), goal(2)) == 1
        error('SAFETY ERROR: Start or Goal position is on an obstacle.');
    end

    % 2. Visual Setup
    f1 = figure('Name','Wavefront Planner');
    set(f1, 'Position', [200, 200, 400, 400]);
    h = imagesc(costMap); 
    title({'Wavefront'}, {'Phase 1 (Propagation)'});
    axis square; colormap([0.5 0.5 0.5; 0 0 0; 0 0 1]); clim([0 2]);
    hold on;
    
    txt_handles = gobjects(rows, cols); 
    for r = 1:rows
        for c = 1:cols
            txt_handles(r,c) = text(c, r, num2str(costMap(r,c)), ...
                'HorizontalAlignment', 'center', 'Color', 'white', 'FontWeight', 'bold');
        end
    end

    % 3. Propagation Setup
    currentValue = 2; 
    costMap(goal(1), goal(2)) = currentValue;
    txt_handles(goal(1), goal(2)).String = num2str(currentValue);
    
    queue = goal; 
    foundStart = false;

    if connectedness == 4
        offsets = [-1 0; 1 0; 0 -1; 0 1]; 
    else
        offsets = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    end

    % Continue propagation while cells remain to explore and Start is not reached.
    % ~isempty(queue) handles blocked paths; ~foundStart enables early exit for speed.
    while ~isempty(queue) && ~foundStart
        currentCell = queue(1,:);
        queue(1,:) = []; 
        
        currentVal = costMap(currentCell(1), currentCell(2));
        allNeighbors = currentCell + offsets;
        
        for j = 1:size(allNeighbors, 1)
            nr = allNeighbors(j, 1);
            nc = allNeighbors(j, 2);

            if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                if costMap(nr, nc) == 0 
                    costMap(nr, nc) = currentVal + 1;
                    txt_handles(nr, nc).String = num2str(costMap(nr, nc));
                    queue = [queue; nr, nc]; 
                    
                    if nr == start(1) && nc == start(2)
                        foundStart = true;
                        break;
                    end
                end
            end
        end
        set(h, 'CData', costMap);
        drawnow;
    end

    % 4. Backtracking and Cost Calculation
    path = [];
    pathLength = 0;
    
    if foundStart
        title({'Wavefront'}, {'Phase 2 (Backtracking)'});
        currentCell = start;
        path = currentCell;
        
        while costMap(currentCell(1), currentCell(2)) > 2
            allNeighbors = currentCell + offsets;
            for j = 1:size(allNeighbors, 1)
                nr = allNeighbors(j, 1);
                nc = allNeighbors(j, 2);
                if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                    if costMap(nr, nc) == costMap(currentCell(1), currentCell(2)) - 1
                        currentCell = [nr nc];
                        path = [path; currentCell];
                        break;
                    end
                end
            end
        end
        
        % Calculate Path Length (number of edges/steps)
        pathLength = size(path, 1) - 1; 

        % Plotting the final path
        plot(path(:,2), path(:,1), 'r-', 'LineWidth', 3); 
        plot(path(1,2), path(1,1), 'go', 'MarkerSize', 12, 'MarkerFaceColor','g'); 
        plot(path(end,2), path(end,1), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r'); 
        uistack(txt_handles(:), 'top');
        title({'Wavefront Complete'}, {['Path Found : ', num2str(pathLength), ' steps']});
    else
        title({'Wavefront Complete'}, {'No Path Possible'});
    end
    hold off;
end