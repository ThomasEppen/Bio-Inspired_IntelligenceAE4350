classdef MoonLander < rl.env.MATLABEnvironment
    properties
        % Set the mass of the Moon Lander
        m               = 1000;
        % Moon's gravity
        gravity         = 1.62;     
        % Lander Diameter
        Diameter        = 15;
        % Placement of small thrusters
        L               = 8;
        % Max thrust of big and small thrusters
        Thrust           = 1750;
        ThurstSmall      = 150;
        % Time step
        dt              = 0.1;
        % Init empty arrays for current state, last action and the shaping
        % for the reward
        State           = zeros(6,1);
        PreviousAction  = zeros(3,1);
        %PreviousHeight  = 0;
        Shaping         = 0;

        Time            = 0;        
        loggedSignals   = cell(2,1);
        Sim             = true;
    end

    properties (Transient, Access = private)
        Visualizer = []
    end
    
    methods
        function this = MoonLander(ActionInfo)
            ObservationInfo(1)  = rlNumericSpec([7,1]);
            ObservationInfo(1).Name = 'states';
            
            % Define action info
            ActionInfo(1) = rlNumericSpec([3 1]);
            ActionInfo(1).Name = 'thrusts';
            
            % Create environment
            this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
            
            % Update action info and initialize states and logs
            updateActionInfo(this);
            this.State = [0 this.Diameter 0 0 0 0]';
            this.loggedSignals{1} = this.State;
            this.loggedSignals{2} = [0 0 0]';
        end

        function set.State(this,state)
            this.State = state(:);
            notifyEnvUpdated(this);
        end
        
        function set.m(this,val)
            this.m = val;
        end
        
        function set.gravity(this,val)
            this.gravity = val;
        end
        
        function set.Diameter(this,val)
            this.Diameter = val;
        end
        
        function set.L(this,val)
            this.L = sort(val);
        end
        
        function set.Thrust(this,val)
            this.Thrust = val;
        end

        function set.ThurstSmall(this,val)
            this.ThurstSmall = val;
            updateActionInfo(this);
        end
        
        function set.dt(this,val)
            this.dt = val;
        end

        function varargout = plot(this)
            if isempty(this.Visualizer) || ~isvalid(this.Visualizer)
                this.Visualizer = MoonLanderSim(this);
            else
                bringToFront(this.Visualizer);
            end
            if nargout
                varargout{1} = this.Visualizer;
            end
            this.Sim = true;
        end

        function [nextObs, reward, isDone, loggedSignals] = step(this, Action)
            loggedSignals   = [];
            bounds          = [100 120-this.Diameter pi 40 40 pi/2];
            timeStep        = this.dt;
            currentState    = this.State(1:6);

            % Unpack the action of the agent
            ThrustBig       = Action(1)*this.Thrust;
            ThustSmall      = Action(2:3).*this.ThurstSmall;              

            % Execute the action of the agent and calculate the next states
            [nextState, action] = dynamics(this, currentState, [ThrustBig ThustSmall(1) ThustSmall(2)]');

            [next, action] = dynamics(this, currentState + nextState*timeStep, action);
            
            % Calculate the state after the action
            st                      = timeStep/2*(nextState + next) + currentState;
            this.State              = st;
            this.PreviousAction     = Action(:);

            % Calculate the angle of the Moon Lander
            st(3)       = atan2(sin(st(3)), cos(st(3)));
            
            % Update the time step
            this.Time = this.Time + timeStep;

            x           = st(1);
            y           = st(2) - this.Diameter/2;
            theta       = st(3); 
            vx          = st(4);
            vy          = st(5);
            dtheta      = st(6);            

            outOfBounds = any(abs(st) > bounds(:));
    
            % Check if the agent has landed and if it is horizontal
            landing         = y <=0;

            roughLanding    = landing && (vy < -0.1 || abs(vx) > 0.1);
            softLanding     = landing && (vy >= -0.1 && abs(vx) <= 0.1);

            %quickLanding    = landing && (this.Time <= 20);
            %rightUp         = level && (abs(dtheta) <= 0.002 ) ;

            
            % Calculate the reward
            distance_scaling = 1 / sqrt((120^2 + 100^2));
            velocity_scaling = 1 / sqrt((40^2 + 40^2));
            
            distance = sqrt(x^2 + y^2) * distance_scaling;
            velocity = sqrt(vx^2 + vy^2) * velocity_scaling;
            
            s = 1 - sqrt(distance) - 0.5 * sqrt(velocity);
            shaping = s - this.Shaping; 

            % Going up penatly
%             up     = y - 100;

            reward = shaping - 0.1 * theta^2; %- 0.01*up;
            
            soft_landing_bonus = 500 * softLanding;
            reward = reward + soft_landing_bonus;
            
%             rough_landing_reward = 50 * roughLanding;
%             reward = reward + rough_landing_reward;
            
      
            if roughLanding
                landingSensor = -1;
            elseif softLanding
                landingSensor = 1;
            else
                landingSensor = 0;
            end
            nextObs                 = [x(:)./bounds(:); landingSensor];
            % Log states and actions
            this.loggedSignals(1)   = {[this.loggedSignals{1}, this.State(:)]};
            this.loggedSignals(2)   = {[this.loggedSignals{2}, action(:)]};
            this.Shaping            = shaping;
            %this.PreviousHeight     = y;
            % When the agent is outside of the bounds or has landed terminate it
            isDone = outOfBounds || landing;
        end
        
        function Obs = reset(this)
            % Reset the initial parameters and let the new agent start at a
            % height of 100 meters
            x0 = 0;
            y0 = 100;
            t0 = 0;
            if rand
                x0 = -20 + 40*rand;             
                t0 = pi/180 * (-45 + 90*rand); 
            end
            x = [x0; y0; t0; 0; 0; 0];

            Obs = [x; 0];
            % Log the states 
            this.State = x;            
            this.Time = 0;
            this.Shaping = 0;
            %this.PreviousHeight = y0;
            this.loggedSignals{1} = this.State;
            this.loggedSignals{2} = [0 0 0]';   
        end
    end
        methods (Access = private)
            function updateActionInfo(this)
                % Define all the possible actions the agent can make
                actns = {...           
                    [0; 0; 0],...
                    [0; 0.5; 0],... 
                    [0; 0.5; 0.5],...
                    [0; 0.5; 1],... 
                    [0; 1; 0],...
                    [0; 1; 0.5],...
                    [0; 1; 1],... 
                    [0; 0; 0.5],...
                    [0; 0; 1],...                     
                    [0.5; 0; 0],...
                    [0.5; 0.5; 0],... 
                    [0.5; 0.5; 0.5],...
                    [0.5; 0.5; 1],... 
                    [0.5; 1; 0],...
                    [0.5; 1; 0.5],...
                    [0.5; 1; 1],... 
                    [0.5; 0; 0.5],...
                    [0.5; 0; 1],... 
                    [1; 0; 0],...
                    [1; 0.5; 0],... 
                    [1; 0.5; 0.5],...
                    [1; 0.5; 1],... 
                    [1; 1; 0],...
                    [1; 1; 0.5],...
                    [1; 1; 1],... 
                    [1; 0; 0.5],...
                    [1; 0; 1],... 
                    }';
                this.ActionInfo = rlFiniteSetSpec(actns);
                this.ActionInfo(1).Name = 'Thrusts';
            end


            function [update, action] = dynamics(this, current, action)
            % Get all the constants
            D       = this.Diameter;
            Lhor    = this.L;
            g       = this.gravity;
            Mass    = this.m;            

            % Unpack the states
            x       = current(1); 
            y       = current(2); 
            theta   = current(3);
            vx      = current(4);
            vy      = current(5);
            dtheta  = current(6);

            % Calculate the thrust based on the action chosen by the agent
            fwd         = max(0, min(this.Thrust, action(1)));
            ThurstLeft  = max(0, min(this.ThurstSmall, action(2)));
            ThrustRight = max(0, min(this.ThurstSmall, action(3)));
            
            Tfwd        = fwd + ThurstLeft + ThrustRight;
            Ttwist      = ThurstLeft - ThrustRight;   

            Thrustx = -sin(theta) * Tfwd;
            Thursty =  cos(theta) * Tfwd;
            Torque  =  Ttwist * Lhor;

            % Model the Moon Lander as a thin spherical shell with inertia
            I       = 2/3*Mass*(D/2)^2;   
            
            update = zeros(6,1);
            
            % Check for a collision with the ground
            yhat = y - D/2;
            
            if yhat < 0
                % If the agent hits the ground then it will bounce back
                k = 2000;
                c = 15;

                update(1) = vx;
                update(2) = vy;
                update(3) = -x/(D/2);
                update(4) = (Thrustx*(D/2)^2 - Torque*D/2)/(I + Mass*(D/2)^2);
                update(5) = Thursty/Mass - g - k*yhat - c*vy;
                update(6) = -vx/(D/2);
                
            else
                % If there is no contact with the ground then it is a
                % freefalling mass
                update(1) = vx;
                update(2) = vy;
                update(3) = dtheta;
                update(4) = Thrustx/Mass;
                update(5) = Thursty/Mass - g;
                update(6) = Torque/I;
                
            end
            
        end

        end
end
    