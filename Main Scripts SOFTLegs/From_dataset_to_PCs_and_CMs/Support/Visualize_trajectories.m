
joint_names = {'Ankle','Hip','Knee'};

figure('visible',flag_display_figures)

for i_sims=1:length(vect_sims)
    if vect_sims(i_sims) < 1
    else
        if vect_sims(i_sims) > 150
        else
            for i_joint=1:3
                
                %hip 
                subplot(1,3,i_joint),plot(MAT_SIMS{1,(i_sims)}(i_joint,1:NumSamples),'Color',[192 192 192]/256,'LineWidth',2);
                xlim([1 NumSamples])
                xticks([1 20 40 60]);
                xticklabels({'0','33','66','100'});

                if i_joint==1
                ylabel('Position [rad]')

                end
                xlabel('Time [% cycle]')
                grid on
                set(gca,'FontSize',30)
                title(joint_names{mod(i_joint,3)+1})
                hold on
            end
        end
    end
end

if flag_display_figures
mkdir('Dataset_Figures')
name='Dataset';
savefig(gcf,['Dataset_Figures/',Q_or_TH,'_',name]);
end

