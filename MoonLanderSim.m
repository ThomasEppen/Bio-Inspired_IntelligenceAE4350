classdef MoonLanderSim < rl.env.viz.AbstractFigureVisualizer
    methods
        function this = MoonLanderSim(env)
            this = this@rl.env.viz.AbstractFigureVisualizer(env)
        end
    end
        methods (Access =protected)
            function f = buildFigure(this)
                f = figure('Toolbar','none',...
                'Visible','on',...
                'HandleVisibility','off', ...
                'NumberTitle','off',...
                'Name','Moon Lander',... 
                'CloseRequestFcn',@(~,~)delete(this));
                if ~strcmp(f.WindowStyle,'docked')
                f.Position = [200 100 800 500];    
                end
            f.MenuBar = 'none';
            
            ha = gca(f);
            ha.XLimMode = 'manual';
            ha.YLimMode = 'manual';
            ha.ZLimMode = 'manual';
            ha.DataAspectRatioMode = 'manual';
            ha.PlotBoxAspectRatioMode = 'manual';
            ha.XLim = [-1 1]*100;
            ha.YLim = [-10 120];
            hold(ha,'on');
            end
            
            function updatePlot(this)
                env             = this.Environment;

                f               = this.Figure;

                ha              = gca(f);

                action          = env.PreviousAction;
                state           = env.State;

                Radius          = env.Diameter*0.5;
                L               = env.L;

                length          = L*0.25;
                x               = state(1);
                y               = state(2);
                theta           = state(3);
                dx              = state(4);
                dy              = state(5);

                col             = (y - Radius) < 0;
                roughCol        = col && (dy < -0.2 || abs(dx) > 0.2);
            
                if col
                    y = Radius;
                end

                c = cos(theta); 
                s = sin(theta);

                R = [c,-s;s,c];

                bodyplot = findobj(ha,'Tag','bodyplot');
                groundplot = findobj(ha,'Tag','groundplot');

                ltthrusterbaseplot = findobj(ha,'Tag','ltthrusterbaseplot');
                rtthrusterbaseplot = findobj(ha,'Tag','rtthrusterbaseplot');
                mtthrusterbaseplot = findobj(ha, 'Tag','mtthrusterbaseplot');
                
                ltthrusterplot = findobj(ha,'Tag','ltthrusterplot');
                rtthrusterplot = findobj(ha,'Tag','rtthrusterplot');
                mtthrusterplot = findobj(ha,'Tag','mtthrusterplot');
                
                textplot = findobj(ha,'Tag','textplot');
                
                if isempty(bodyplot) || ~isvalid(bodyplot) || ...
                        isempty(ltthrusterbaseplot) || ~isvalid(ltthrusterbaseplot) || ...
                        isempty(rtthrusterbaseplot) || ~isvalid(rtthrusterbaseplot) || ...
                        isempty(ltthrusterplot) || ~isvalid(ltthrusterplot) || ...
                        isempty(rtthrusterplot) || ~isvalid(rtthrusterplot) || ...
                        isempty(mtthrusterbaseplot) || ~isvalid(mtthrusterbaseplot) || ...
                        isempty(mtthrusterplot) || ~isvalid(mtthrusterplot) || ...
                        isempty(textplot) || ~isvalid(textplot)
                        
                    bodyplot = rectangle(ha,'Position', [x-Radius y-Radius 2*Radius 2*Radius],...
                        'Curvature',[1 1],'FaceColor','y','Tag','bodyplot');
                    groundplot = line(ha,ha.XLim,[0 0],'LineWidth',2,'Color','k','Tag','groundplot'); 
                    ltthrusterbaseplot = line(ha,[0 0],[0 0],'LineWidth',1,'Color','k','Tag','ltthrusterbaseplot');
                    rtthrusterbaseplot = line(ha,[0 0],[0 0],'LineWidth',1,'Color','k','Tag','rtthrusterbaseplot');
                    mtthrusterbaseplot = line(ha,[0 0],[0 0],'LineWidth',1,'Color','k','Tag','mtthrusterbaseplot');
                    
                    ltthrusterplot = patch(ha,[0 0 0],[0 0 0],'r','Tag','ltthrusterplot');
                    rtthrusterplot = patch(ha,[0 0 0],[0 0 0],'r','Tag','rtthrusterplot');
                    mtthrusterplot = patch(ha,[0 0 0],[0 0 0],'r','Tag','mtthrusterplot');
                    
                    textplot = text(ha,0,0,'','Color','r','Tag','textplot');
                end

                bodyplot.Position = [x-Radius y-Radius 2*Radius 2*Radius];

                LL1 = [-L-length;0];
                LL2 = [-L+length;0];
                LR1 = [+L-length;0];
                LR2 = [+L+length;0];

                LM1 = [-length; -Radius/2];
                RM2 = [+length; -Radius/2];
                
                TL1 = [-L-length;0];
                TL2 = [-L+length;0];
                TL3 = [-L   ;-action(2)*5];
                TR1 = [+L-length;0];
                TR2 = [+L+length;0];
                TR3 = [+L   ;-action(3)*5];

                TM1 = [-length; -Radius/2];
                TM2 = [+length; -Radius/2];
                TM3 = [0; (-Radius/2-action(1)*10)];
                
                in = [LL1 LL2 LR1 LR2 TL1 TL2 TL3 TR1 TR2 TR3 LM1 RM2 TM1 TM2 TM3];
                out = R*in + [x;y];
                
                ltthrusterbaseplot.XData = out(1, 1:2);
                ltthrusterbaseplot.YData = out(2, 1:2);
                rtthrusterbaseplot.XData = out(1, 3:4);
                rtthrusterbaseplot.YData = out(2, 3:4);
                
                mtthrusterbaseplot.XData = out(1, 11:12);
                mtthrusterbaseplot.YData = out(2, 11:12);
                
                ltthrusterplot.XData = out(1, 5:7);
                ltthrusterplot.YData = out(2, 5:7);
                rtthrusterplot.XData = out(1, 8:10);
                rtthrusterplot.YData = out(2, 8:10);

                mtthrusterplot.XData = out(1, 13:15);
                mtthrusterplot.YData = out(2, 13:15);

                
                if roughCol
                    textplot.String = 'Hard Landing';
                    textplot.Position = [x,-5];
                else
                    textplot.String = '';
                end
                
                drawnow();


            end            
    end
end
