%%
% data_parser_new;
before_timeout_trials = ~(trajectory_time == 11.9); %all trials that did not time out. 
%% Trajectory Time Analysis. Almost Normal Distribution. Therefore use ANOVA. 
title_list = {'R^2', 'R^3', 'SE(2)', 'SE(3)'};
bins = 20;
sps = cell(4,1);
% for i=1:length(spaceList)
%     data = trajectory_time(:, :, i);
%     fprintf('*****************\n');
%     fprintf('THE SPACE IS %s\n', spaceList{i});
% %     close all;
%      x = []; cond = {};
%     for j=1:length(condList)
%         fprintf('THE COND IS %s\n', condList{j});
%         col = data(before_timeout_trials(:,j,i) == 1, j); %only those trials that did not timeout.
%         x = [x; col(:)]; cond = cat(1, cond, cellstr(repmat(condList{j}, length(col), 1 )));
% %         histogram(col, bins); hold on;
% %         pause;
%     end
%     [p, tbl, stats] = anova1(x, cond, 'off');
%     c = multcompare(stats, 'Display', 'off');
%     sps{i} = subplot(2,2,i);
%     bh1 = boxplot(x, cond); 
%     set(bh1(:,1),'linewidth',2); set(bh1(:,2),'linewidth',2); set(bh1(:,3),'linewidth',2); set(bh1(:,4),'linewidth',2);
%     title(sps{i}, title_list{i}); 
%     grid on; xlabel('Mode Switch Assistance'); ylabel('Trial Completion Time');
%     pvalues = c(:, 6);
% %     [p,tbl,stats] = kruskalwallis(x, cond);
%     
%     
% end

%% Kruskal Wallis on Initial Alpha Distribution (R2 has no difference...R3, SE2, SE3 showing some difference). 
% R2
for i=1:length(spaceList) %R2, R3, SE2, SE3
    data = initial_alpha(:, :, i);
    fprintf('*****************\n');
    fprintf('THE SPACE IS %s\n', spaceList{i});
%     close all;
    x = []; cond = {}; maxx = zeros(length(condList), 1);
    for j=1:length(condList)
        fprintf('THE COND IS %s\n', condList{j});
        col = data(before_timeout_trials(:,j,i) == 1, j); %only those trials that did not timeout. 
        col(col == -999) = [];
        maxx(j) = max(col(:));
        x = [x; col(:)]; cond = cat(1, cond, cellstr(repmat(condList{j}, length(col), 1 )));
%          histogram(col, bins); hold on;
%         pause;
    end
    [p,tbl,stats] = kruskalwallis(x, cond, 'off');
    c = multcompare(stats, 'Display', 'off');
    sps{i} = subplot(2,2,i);
    bh1 = boxplot(x, cond, 'whisker', 5); 
    set(bh1(:,1),'linewidth',2); set(bh1(:,2),'linewidth',2); set(bh1(:,3),'linewidth',2); set(bh1(:,4),'linewidth',2);
    title(sps{i}, strcat('\bf \fontsize{15}',title_list{i})); 
    grid on; xlabel('\bf Mode Switch Assistance Type'); ylabel('\bf Onset Time (Normalized)');
    pvalues = c(:, 6);
    %%significance plotting
    xt = get(gca, 'XTick');
    yt = get(gca, 'YTick');
    axis([xlim    0  ceil(max(yt)*1.2)]); hold on;
%     plot(xt([1 3]), [1 1]*max(yt)*1.1, '-k',  [0.9, 1,1.1]*mean(xt([1 3])), [1,1,1]*max(yt)*1.15, '*k')
    for j=1:length(pvalues)
        if pvalues(j) < 0.001
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', [mean([xtl, xth]) - 0.07, mean([xtl, xth]), mean([xtl, xth]) + 0.07], [1,1,1]*yval*(1.04 + j*0.04), '*k');
        elseif pvalues(j) < 0.05
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', [mean([xtl, xth]) - 0.02, mean([xtl, xth]) + 0.02], [1,1]*yval*(1.04+ j*0.04), '*k');
        elseif pvalues(j) < 0.01
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', mean([xtl, xth]), yval*(1.04 + j*0.04), '*k');
        end
    end
end
%% Number of mode switches Analysis
bins = 25;
% for i=1:length(spaceList)
%     data = num_mode_switches(:, :, i);
%     fprintf('*****************\n');
%     fprintf('THE SPACE IS %s\n', spaceList{i});
% %     close all;
%      x = []; cond = {}; maxx = zeros(length(condList), 1);
%     for j=1:length(condList)
%         fprintf('THE COND IS %s\n', condList{j});
%         col = data(before_timeout_trials(:,j,i) == 1, j); %only those trials that did not timeout.
%         maxx(j) = max(col(:));
%         x = [x; col(:)]; cond = cat(1, cond, cellstr(repmat(condList{j}, length(col), 1 )));
% %         histogram(col, bins); hold on;
% %         pause;
%     end
%     [p,tbl,stats] = kruskalwallis(x, cond, 'off');
%     c = multcompare(stats, 'Display', 'off');
%     sps{i} = subplot(2,2,i);
%     bh1 = boxplot(x, cond, 'whisker', 3); 
%     set(bh1(:,1),'linewidth',2); set(bh1(:,2),'linewidth',2); set(bh1(:,3),'linewidth',2); set(bh1(:,4),'linewidth',2);
%     title(sps{i}, title_list{i}); 
%     grid on; xlabel('\bf Mode Switch Assistance Type'); ylabel('\bf Number of Mode Switches');
%     pvalues = c(:, 6);
%     %%significance plotting
%     xt = get(sps{i}, 'XTick');
%     yt = get(sps{i}, 'YTick');
%     axis([xlim    0  ceil(max(yt)*1.3)]); hold on;
% %     plot(xt([1 3]), [1 1]*max(yt)*1.1, '-k',  [0.9, 1,1.1]*mean(xt([1 3])), [1,1,1]*max(yt)*1.15, '*k')
%     for j=1:length(pvalues)
%         if pvalues(j) < 0.001
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.01 + j*0.023), '-k', [mean([xtl, xth]) - 0.05, mean([xtl, xth]), mean([xtl, xth]) + 0.05], [1,1,1]*yval*(1.02 + j*0.023), '*k');
%         elseif pvalues(j) < 0.05
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.01 + j*0.023), '-k', [mean([xtl, xth]) - 0.02, mean([xtl, xth]) + 0.02], [1,1]*yval*(1.02+ j*0.023), '*k');
%         elseif pvalues(j) < 0.01
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.01 + j*0.023), '-k', mean([xtl, xth]), yval*(1.02 + j*0.023), '*k');
%         end
%     end
% end

%% Amount of time assistance is present when inference is correct and task is completed. 
bins = 25;
for i=1:length(spaceList)
    data = percentage_alpha(:, :, i);
    fprintf('*****************\n');
    fprintf('THE SPACE IS %s\n', spaceList{i});
%     close all;
     x = []; cond = {}; maxx = zeros(length(condList), 1);
    for j=1:length(condList)
        fprintf('THE COND IS %s\n', condList{j});
        col = data(before_timeout_trials(:,j,i) == 1, j); %only those trials that did not timeout.
        maxx(j) = max(col(:));
        x = [x; col(:)]; cond = cat(1, cond, cellstr(repmat(condList{j}, length(col), 1 )));
%         histogram(col, bins); hold on;
%         pause;
    end
    [p,tbl,stats] = kruskalwallis(x, cond, 'off');
    c = multcompare(stats, 'Display', 'off');
    sps{i} = subplot(2,2,i);
    bh1 = boxplot(x, cond, 'whisker', 1.5); 
    set(bh1(:,1),'linewidth',2); set(bh1(:,2),'linewidth',2); set(bh1(:,3),'linewidth',2); set(bh1(:,4),'linewidth',2);
    title(sps{i}, strcat('\bf \fontsize{15}',title_list{i})); 
    grid on; xlabel('\bf Mode Switch Assistance Type'); ylabel('\bf \fontsize{10} Total Assistance (Normalized)');
    pvalues = c(:, 6);
    %%significance plotting
    xt = get(sps{i}, 'XTick');
    yt = get(sps{i}, 'YTick');
    axis([xlim    0  ceil(max(yt)*1.3)]); hold on;
%     plot(xt([1 3]), [1 1]*max(yt)*1.1, '-k',  [0.9, 1,1.1]*mean(xt([1 3])), [1,1,1]*max(yt)*1.15, '*k')
%     for j=1:length(pvalues)
%         if pvalues(j) < 0.001
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.02 + j*0.2), '-k', [mean([xtl, xth]) - 0.05, mean([xtl, xth]), mean([xtl, xth]) + 0.05], [1,1,1]*yval*(1.15 + j*0.2), '*k');
%         elseif pvalues(j) < 0.05
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.1 + j*0.2), '-k', [mean([xtl, xth]) - 0.02, mean([xtl, xth]) + 0.02], [1,1]*yval*(1.15+ j*0.2), '*k');
%         elseif pvalues(j) < 0.01
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.1 + j*0.2), '-k', mean([xtl, xth]), yval*(1.15 + j*0.2), '*k');
%         end
%     end
    for j=1:length(pvalues)
        if pvalues(j) < 0.001
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', [mean([xtl, xth]) - 0.07, mean([xtl, xth]), mean([xtl, xth]) + 0.07], [1,1,1]*yval*(1.04 + j*0.04), '*k');
        elseif pvalues(j) < 0.05
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', [mean([xtl, xth]) - 0.02, mean([xtl, xth]) + 0.02], [1,1]*yval*(1.04+ j*0.04), '*k');
        elseif pvalues(j) < 0.01
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', mean([xtl, xth]), yval*(1.04 + j*0.04), '*k');
        end
    end
end

%% Correct inference of goal when task is completed. 

bins = 25;
for i=1:length(spaceList)
    data = percentage_correct_inference(:, :, i);
    fprintf('*****************\n');
    fprintf('THE SPACE IS %s\n', spaceList{i});
%     close all;
     x = []; cond = {}; maxx = zeros(length(condList), 1);
    for j=1:length(condList)
        fprintf('THE COND IS %s\n', condList{j});
        col = data(before_timeout_trials(:,j,i) == 1, j); %only those trials that did not timeout.
        x = [x; col(:)]; cond = cat(1, cond, cellstr(repmat(condList{j}, length(col), 1 )));
         maxx(j) = max(col(:));
%         histogram(col, bins); hold on;
%         pause;
    end
    [p,tbl,stats] = kruskalwallis(x, cond, 'off');
    c = multcompare(stats, 'Display', 'off');
    sps{i} = subplot(2,2,i);
    bh1 = boxplot(x, cond, 'whisker', 1.5); 
    set(bh1(:,1),'linewidth',2); set(bh1(:,2),'linewidth',2); set(bh1(:,3),'linewidth',2); set(bh1(:,4),'linewidth',2);
    title(sps{i}, strcat('\bf \fontsize{15}',title_list{i})); 
    grid on; xlabel('\bf Mode Switch Assistance Type'); ylabel('\bf \fontsize{8} Total Correct Inference (Normalized)');
    pvalues = c(:, 6);
    %significance plotting
    xt = get(sps{i}, 'XTick');
    yt = get(sps{i}, 'YTick');
    axis([xlim    0  ceil(max(yt)*1.3)]); hold on;
    plot(xt([1 3]), [1 1]*max(yt)*1.1, '-k',  [0.9, 1,1.1]*mean(xt([1 3])), [1,1,1]*max(yt)*1.15, '*k')
    for j=1:length(pvalues)
        if pvalues(j) < 0.001
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.05 + j*0.1), '-k', [mean([xtl, xth]) - 0.05, mean([xtl, xth]), mean([xtl, xth]) + 0.05], [1,1,1]*yval*(1.1 + j*0.1), '*k');
        elseif pvalues(j) < 0.05
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.05 + j*0.1), '-k', [mean([xtl, xth]) - 0.02, mean([xtl, xth]) + 0.02], [1,1]*yval*(1.1+ j*0.1), '*k');
        elseif pvalues(j) < 0.01
            xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
            plot([xtl, xth], [1 1]*yval*(1.05 + j*0.1), '-k', mean([xtl, xth]), yval*(1.1 + j*0.1), '*k');
        end
    end
%     for j=1:length(pvalues)
%         if pvalues(j) < 0.001
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', [mean([xtl, xth]) - 0.07, mean([xtl, xth]), mean([xtl, xth]) + 0.07], [1,1,1]*yval*(1.04 + j*0.04), '*k');
%         elseif pvalues(j) < 0.05
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', [mean([xtl, xth]) - 0.02, mean([xtl, xth]) + 0.02], [1,1]*yval*(1.04+ j*0.04), '*k');
%         elseif pvalues(j) < 0.01
%             xtl = c(j, 1); xth = c(j, 2); yval = max(maxx(xtl), maxx(xth));
%             plot([xtl, xth], [1 1]*yval*(1.02 + j*0.04), '-k', mean([xtl, xth]), yval*(1.04 + j*0.04), '*k');
%         end
%     end
end