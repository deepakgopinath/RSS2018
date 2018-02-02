close all;
%% percentage alpha
parse_data_SIM;
interfaces = {'j','h'};
interface_cells = {'pa_re_jkld', 'pa_po_jkld', 'pa_re_jpot', 'pa_po_jpot';
                    'pa_re_hkld', 'pa_po_hkld', 'pa_re_hpot', 'pa_po_hpot'};
interface_cells = {'pc_re_jkld', 'pc_po_jkld', 'pc_re_jpot', 'pc_po_jpot';
                    'pc_re_hkld', 'pc_po_hkld', 'pc_re_hpot', 'pc_po_hpot'};

interface_cells = {'ia_re_jkld', 'ia_po_jkld', 'ia_re_jpot', 'ia_po_jpot';
                    'ia_re_hkld', 'ia_po_hkld', 'ia_re_hpot', 'ia_po_hpot'};
%                 
% %only headarray data; 
ha_cells = {'pa_re_hkld', 'pa_po_hkld', 'pa_re_hpot', 'pa_po_hpot'; %plot 1 -  percent assistance
            'pc_re_hkld', 'pc_po_hkld', 'pc_re_hpot', 'pc_po_hpot';% plot 2 - percent correct inference
            'ia_re_hkld', 'ia_po_hkld', 'ia_re_hpot', 'ia_po_hpot'}; %plot 3 - initial asistance
%                
y1 = 1.0; y2 = 2.0;
ylims = 1.5*[1.0, 1.0, 1.0];

%group tasks and interfaces. only across assistance conditions

%% 
v1 = [eval(interface_cells{1, 1}); eval(interface_cells{1, 2}); eval(interface_cells{2, 1}); eval(interface_cells{2, 1})]; %kld group, group tasks and interfaces
v2 = [eval(interface_cells{1, 3}); eval(interface_cells{1, 4}); eval(interface_cells{2, 3}); eval(interface_cells{2, 4})]; %pot group, group tasks and interfaces
figure; hold on; grid on;
bh1 = boxplot(v1, 'positions', y1,'whisker', 50,'Widths', 0.3);set(bh1(:,1),'linewidth',2);
bh2 = boxplot(v2, 'positions', y2,'whisker', 50,'Widths', 0.3);set(bh2(:,1),'linewidth',2);
axis([0,3,0,1.0]);
grid on;
set(gca,  'fontWeight', 'normal', 'YTick', 0:0.2:1.0);
title('\bf \fontsize{16} Initial Alpha  ');
set(gca,'fontWeight','bold','Xtick',1:1:2);
set(gca, 'XTickLabel', {'KLD','POT'});
ylabel('\bf \fontsize{11} % Correct Inference');
[p,h] = ranksum(v1, v2);
disp(p);
%% 

for kk=1:length(interfaces)
    v1 = eval(interface_cells{kk, 1}); 
    v2 = eval(interface_cells{kk, 2}); 
    v3 = eval(interface_cells{kk, 3}); 
    v4 = eval(interface_cells{kk, 4}); 
    
    v1 = [v1;v2]; %group tasks. kld
    v3 = [v3;v4]; %group tasks. pot
    
    if kk == 1
        subplot(2,2,1); hold on; grid on;
    else
        subplot(2,2,2); hold on; grid on;
    end
    if kk==1
%     bh1 = boxplot([v1,v2], 'whisker', 50);
        bh1 = boxplot(v1, 'positions', y1,'whisker', 50,'Widths', 0.3);set(bh1(:,1),'linewidth',2);
        bh2 = boxplot(v3, 'positions', y2,'whisker', 50,'Widths', 0.3);set(bh2(:,1),'linewidth',2);
    else
        bh3 = boxplot(v1, 'positions', y1,'whisker', 50,'Widths', 0.3);set(bh3(:,1),'linewidth',2);
        bh4 = boxplot(v3, 'positions', y2,'whisker', 50,'Widths', 0.3);set(bh4(:,1),'linewidth',2);
    end
    axis([0,3,0,ylims(kk)]);
    grid on;
    if kk == 1
        set(gca,  'fontWeight', 'normal', 'YTick', 0:0.2:ylims(kk));
        title('\bf \fontsize{16} Joystick  ');
        set(gca,'fontWeight','bold','Xtick',1:1:2);
%         set(gca,'DefaultTextInterpreter', 'tex')
        set(gca, 'XTickLabel', {'KLD','POT'});
        ylabel('\bf \fontsize{11} % Correct Inference');
    else
        set(gca, 'fontWeight', 'normal',  'YTick', 0:0.2:ylims(kk));
        title('\bf \fontsize{16} Headarray');
%         set(gca,'DefaultTextInterpreter', 'tex')
        set(gca,'fontWeight','bold','Xtick',1:1:2);
        set(gca, 'XTickLabel', {'KLD','POT'});
    end
    [p,h] = ranksum(v1, v3);
    disp(p);
end
%%

for kk=1:3
    v1 = eval(ha_cells{kk, 1}); 
    v2 = eval(ha_cells{kk, 2}); 
    v3 = eval(ha_cells{kk, 3}); 
    v4 = eval(ha_cells{kk, 4}); 
    
    v1 = [v1;v2]; %group tasks. kld
    v3 = [v3;v4]; %group tasks. pot
    
    if kk == 1
        subplot(1,3,1); hold on; grid on;
    elseif kk==2
        subplot(1,3,2); hold on; grid on;
    else
        subplot(1,3,3); hold on; grid on;
    end
    if kk==1
%     bh1 = boxplot([v1,v2], 'whisker', 50);
        bh1 = boxplot(v3, 'positions', y1,'whisker', 50,'Widths', 0.3);set(bh1(:,1),'linewidth',2);
        bh2 = boxplot(v1, 'positions', y2,'whisker', 50,'Widths', 0.3);set(bh2(:,1),'linewidth',2);
    elseif kk==2
        bh3 = boxplot(v3, 'positions', y1,'whisker', 50,'Widths', 0.3);set(bh3(:,1),'linewidth',2);
        bh4 = boxplot(v1, 'positions', y2,'whisker', 50,'Widths', 0.3);set(bh4(:,1),'linewidth',2);
    else
        bh5 = boxplot(v3, 'positions', y1,'whisker', 50,'Widths', 0.3);set(bh5(:,1),'linewidth',2);
        bh6 = boxplot(v1, 'positions', y2,'whisker', 50,'Widths', 0.3);set(bh6(:,1),'linewidth',2);
    end
    axis([0,3,0,ylims(kk)]);
    grid on;
    if kk == 1
        set(gca,  'fontWeight', 'normal', 'YTick', 0:0.2:1.0);
        title('\bf \fontsize{8} Total Assistance');
        set(gca,'fontWeight','bold','Xtick',1:1:2);
%         set(gca,'DefaultTextInterpreter', 'tex')
        set(gca, 'XTickLabel', {'GRD','KL'});
        ylabel('\bf \fontsize{8} Normalized Time');
    elseif kk==2
        set(gca, 'fontWeight', 'normal',  'YTick', 0:0.2:1.0);
        title('\bf \fontsize{8} Percentage Correct Inference');
%         set(gca,'DefaultTextInterpreter', 'tex')
        set(gca,'fontWeight','bold','Xtick',1:1:2);
        set(gca, 'XTickLabel', {'GRD','KL'});
        ylabel('\bf \fontsize{8} Normalized Time');
    else
        set(gca, 'fontWeight', 'normal',  'YTick', 0:0.2:1.0);
        title('\bf \fontsize{8} Onset of Initial Assistance');
%         set(gca,'DefaultTextInterpreter', 'tex')
        set(gca,'fontWeight','bold','Xtick',1:1:2);
        set(gca, 'XTickLabel', {'KLD','POT'});
        ylabel('\bf \fontsize{8} Normalized Time');
    end
    [p,h] = ranksum(v1, v3);
    disp(p);
    ypos = max(max(v1), max(v3)); yoffset = 0.02;
    if p <= 0.05
        line([y1, y1], [ypos + yoffset, ypos+2*yoffset], 'LineWidth', 2.0);
        line([y2, y2], [ypos + yoffset, ypos+2*yoffset], 'LineWidth', 2.0);
        line([y1, y2], [ypos + 2*yoffset, ypos+2*yoffset], 'LineWidth', 2.0);
    end
    if p <= 0.001
        text(0.5*(y1+y2), ypos+3*yoffset, '***', 'HorizontalAlignment', 'Center', 'BackGroundcolor', 'none', 'FontSize', 15);
    elseif p <= 0.01
        text(0.5*(y1+y2), ypos+3*yoffset, '**', 'HorizontalAlignment', 'Center', 'BackGroundcolor', 'none', 'FontSize', 15);
    elseif p <= 0.05
        text(0.5*(y1+y2), ypos+3*yoffset, '*', 'HorizontalAlignment', 'Center', 'BackGroundcolor', 'none', 'FontSize', 15);
    end
end

