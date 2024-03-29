function[] = test( )
% This main function will execute the entirety of the Truck and Drone 
% routing algorithm. 

%%%
% When it comes to calculating the "travel time between two customers" we will 
% use the distance between the two nodes for trucks. When calculating the 
% travel time between two customers by UAV, we will use the distance but 
% divided by a speed factor of 1.5.
%%%

    % Variables 
    %      C0            Set of customer locations with depot as the starting
    %                     location where C0.x is the x location of the
    %                     customers and C0.y is the y locations of the
    %                     customers. 
    %       c            Number of customers
    %       U            Set of drones
    %       u            Number of drones
    % aafDistancesTruck  Travel times between two customers i & j by truck
    %     soln           Our representation of the solution where:
    %                      ansoln.Part1 represents the sequence of customers
    %                          visited by the truck
    %                      ansoln.Part2 represents the assignment of
    %                          customers to UAVs & also the sequence of
    %                          visits by each UAV
    %                      ansoln.Part3 represents the launch locations of
    %                          each UAV for every flight
    %                      ansoln.Part4 represents the reconvene locations of
    %                          each UAV for every flight
    %
    
    
    % Generate customer locations
%     x = (50 - -50)*rand(1, 4) + -50;
%     y = (50 - -50)*rand(1, 4) + -50;
%     C0.x = [ 0 -10 0 0 ]; % we treat the first slot as the depot;
%     C0.y = [ 0 0 -10 -5 ];  % we treat the first slot as the depot;
%     C0.x = [ 0 randi([-10, 10], 1, 10) 0 ];
%     C0.y = [ 0 randi([-10, 10], 1, 10) 0 ];
    
%     C0.x = [ 0 -4 -7 -6 4 6 2 9 8 7 6 0 ];
%     C0.y = [ 0 1 -3 -8 -9 -3 10 9 -2 -8 -8 0];
    
% %            0  1  2  3  4  0   Customer ID
% %            1  2  3  4  5  6   Indices
%     C0.x = [ 0  8  6 -7  1  0 ];
%     C0.y = [ 0 -2 -3 -3  2  0 ];

%            0  1  2  3  4  5  6  0  Customer ID
%            1  2  3  4  5  6  7  8  Indices
    C0.x = [ 0 -2 -3  2  2  7 -4  0 ]; 
    C0.y = [ 0  4  1  4  3  1  3  0 ]; 

    
    % Plot the customer locations
%     hold on
%     figure( 1 );
%     plot(0,0,'b*', 'MarkerSize', 12)
%     plot_2D(C0.x, C0.y)
%     hold off

    
    % Numbers of customers (excluding the depot) 
    c = length(C0.x) - 2;
    
    % Number of drones
    U = [1, 2];
    u = length(U);
    
    % Initial and final temperature (Inputs for SA)
    T0 = 90;
    TFinal = 0.01; 
    Beta_SA = 0.99;
    Imax = 20;
    
    % Calculate the distances between each all points
    % Here, our first row of the columns is how far our depot is from each 
    % of the customers. 
    aafDistances = calculateDistances(C0.x, C0.y);
    
    
    % Initialize solution data structure
    soln.anPart1 = zeros(1, length(C0.x));  % the Truck Route
    soln.anPart2 = [-1];  % Assignment of customers to UAVs
    soln.anPart3 = [-1];  % Launch locations of the UAVs
    soln.anPart4 = [-1];  % Reconvene locations of the UAVs
    
    
    % Generate initial solution s, to the STRPD based on SA
%     s = simulatedAnnealing(c, aafDistances, soln, T0, TFinal, Beta_SA, Imax, u);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Test plotting our path with drones
    % THis is correct btw
%     s.anPart1 = [ 0 4 2 3 0 ];
%     s.anPart3 = [ 2 -1 ];   % These indices refer to the cell number in part 1, so this is wrong. 
%     s.anPart2 = [ 1 -1 ];   % visits customer 1
%     s.anPart4 = [ 3 -1 ];   % returns to index 2 of s.anPart1 (which is customer 3)

    s.anPart1 = [ 0 4 3 1 2 0 ];
    s.anPart3 = [ 1 -1 4 ];
    s.anPart2 = [ 5 -1 6 ];
    s.anPart4 = [ 3 -1 5 ]; 
    
%     s.anPart1 = [ 0 4 2 0 ];
%     s.anPart3 = [ 2 3 -1 ];   % These indices refer to the cell number in part 1, so this is wrong. 
%     s.anPart2 = [ 1 3 -1 ];   % visits customer 1
%     s.anPart4 = [ 3 4 -1 ];   % returns to index 2 of s.anPart1 (which is customer 3)

%     s.anPart1 = [ 0 4 2 0 ];
%     s.anPart3 = [ 2 -1 3 ];   % These indices refer to the cell number in part 1, so this is wrong. 
%     s.anPart2 = [ 1 -1 3 ];   % visits customer 1
%     s.anPart4 = [ 3 -1 4 ];   % returns to index 2 of s.anPart1 (which is customer 3)

%     s.anPart1 = [ 0 4 2 0 ];
%     s.anPart3 = [ 2 -1 3 ];   % These indices refer to the cell number in part 1, so this is wrong. 
%     s.anPart2 = [ 1 -1 3 ];   % visits customer 1
%     s.anPart4 = [ 3 -1 4 ];   % returns to index 2 of s.anPart1 (which is customer 3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Plot the truck and drone route
    plot_route( C0, s);
    
    
    % Initialize best solution, the rsult of the best solution, the 
    % iteration number, and the temperature
    s_best = s;
    C0
    fs_best = f(aafDistances, s_best, u)
    
    
    % 
    
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function[ ] = plot_2D( x, y )
% plot_2D will plot the given x and y coordinates 
% INPUT
%   x
%   y
% OUTPUT

    % Variables
    %
    
    % Plot the graph
    plot(x, y, "r.", 'MarkerSize', 12)
    xline(0)
    yline(0)
    
end

function[ ] = plot_route( C0, s) 
% plot_route will take in the customer locations & the initial solution
% produced via the Simulated annealing method. It will then plot the route 
% the truck will take. 
% Input 
%   C0      The locations of all the customers
%   s       The route we are plotting 
%   u       The number of drones 
% Output
%   

    % Variables
    %    a, b     Placeholder variables for the coordinates 
    %  nCustInd   counter to keep track of the customer indices 
    
    % Plot the truck route
    nCustInd = 2;
    a = [ 0 ]; 
    b = [ 0 ]; 
    
    % Fill in these a and b arrays
%     for i = s.anPart1( 2 : length(C0.x) - 1 )
    for i = s.anPart1( 2 : length(s.anPart1) - 1)
        a(nCustInd) = C0.x(i+1);
        b(nCustInd) = C0.y(i+1);
        nCustInd = nCustInd + 1; 
        a;
        b;
    end

    a(end + 1) = 0;
    b(end + 1) = 0;


        
    % Initialize the coords visited by the drones
    aanDrones = zeros(length(s.anPart2)-1, 6);
    
    C0;
    s; 
    % Fill in this matrix
    for iCustomer = 1 : length(s.anPart2)        
        if s.anPart2(iCustomer) == -1
            aanDrones( iCustomer, : ) = -1*ones(1, 6);
%             aanDrones = [ aanDrones ; -1*ones(1, 6) ]
            
        else
            aanDrones(iCustomer, 1) = C0.x(s.anPart1(s.anPart3(iCustomer)) + 1);
            aanDrones(iCustomer, 4) = C0.y(s.anPart1(s.anPart3(iCustomer)) + 1);

            aanDrones(iCustomer, 3) = C0.x(s.anPart1(s.anPart4(iCustomer)) + 1);
            aanDrones(iCustomer, 6) = C0.y(s.anPart1(s.anPart4(iCustomer)) + 1);

            aanDrones(iCustomer, 2) = C0.x(s.anPart2(iCustomer) + 1);
            aanDrones(iCustomer, 5) = C0.y(s.anPart2(iCustomer) + 1);
        end
    end
    
    
    
    % Plot the drone route
    aColors = ["r--", "b--", "g--", "k--", "m--"];
        
    % 
    aDimensions = size(aanDrones);
    
    % Plot this stuff
    figure(2)
    hold on; 
    plot(0,0,'b*', 'MarkerSize', 12)
    plot(a, b, 'k-')
    plot(a, b, "ro")
    
    iDrone = 1; 
    for iRow = 1 : aDimensions 
        x = [ 0 ];
        y = [ 0 ]; 
        
        if aanDrones(iRow, 1) == -1
            iDrone = iDrone + 1; 
        else
            for iCol = 1 : 3
                x(iCol) = aanDrones(iRow, iCol);
                y(iCol) = aanDrones(iRow, iCol+3);
            end
            
            plot(x, y, aColors(iDrone));
        end
    end
    hold off; 
end

function[ aafDistances ] = calculateDistances( x, y )
% calculateDistances will calculate the distances between all the points 
%   on the graph
% INPUT
%   x, y   the x and y coordinates of the customers
% OUTPUT
%   aafDistances   An array of distances where D_ij is the distance between 
%                    house i and house j; this matrix should be symmetric

    % Variables
    %
    
    % Initialize the distances array
    aafDistances = zeros(length(x));
    
    % Calculate the distances
    for i = 1 : length(x)
        for j = 1 : length(x)
            aafDistances( i, j ) = sqrt( (x(i) - x(j))^2 + (y(i) - y(j))^2 );
        end
    end
    
end

function[ s_best ] = ...
    simulatedAnnealing( c, aafDistances, soln, T0, TFinal, Beta_SA, Imax, u)
% simulatedAnnealing function will take in an array of distances 
% and will generate a random TRP (traveling repairman problem).
% INPUT
%   c              The length of the soln
%  aafdistances    Array of distances between customers
%  soln            Data structure of solution
%   T0             The initial temperature
%   TFinal         The final temperature we want to approach
%  Beta_SA         The cooling schedule for our simulated annealing
%   Imax           The maximum number of iterations
%   u              The numebr of drones
% OUTPUT
%  s_best          Returns the best solution to the TRP 


    % Variables
    %  soln       Data structure with solutions parts 1, 2, 3, and 4 
    %     s       Solution to the TRP problem
    %  s_best     Best solution
    % fs_best     The result of the best solution7
    %     T       The current temperature of our simulated annealing
    

    % Initialize our current and final temperatures
    T = T0;     
    
    % Generate a ranomd TRP (Traveling Repairman Problem) 
    % randomly insert all customers into vector    
    soln.anPart1 = [ 0 randperm(c) 0 ];
    s = soln; % create duplicate of soln for comparison
    
    % Initilaize the best solution and the best result 
    s_best = s;
    fs_best = f(aafDistances, s, u); 
    
    sPrime = soln;
    
    % Run the simulated annealing process
    while T > TFinal
        for i = 1 : Imax 
            % Find neighbor to s
            sPrime.anPart1 = neighbor(s);
            

            
            % Check if neighbor is better than current solution 
            diff = fs_best - f(aafDistances, sPrime, u);
            
            % Check if neighbor is better
            if f(aafDistances, sPrime, u) < f(aafDistances, s_best, u)
                s_best.anPart1 = sPrime.anPart1;
                fs_best = f(aafDistances, sPrime, u); 
            end
            
            % Boltzmann probability time babbbyyyy
            if rand() < exp(-abs(diff) / T)
                s.anPart1 = sPrime.anPart1;
            end
            
        end
        
        % Cooling schedule
        T = Beta_SA * T; 
    end
    
    
    
end

function[ sPrime ] = neighbor( soln )
% The neighbor function will implement the well-known 2-opt method for
% creating neighbors for our current solution s. Here, we must note that
% our 2-opt method will switch two customers within the vector of Parts 1
% and 2. The customers are selected randomly. Our 2-opt is a local search
% heuristic.

% INPUT
%   soln       Current solution data structure.
% OUTPUT
%  sPrime      A neighbor to input solution soln

    % Variables
    %    sPrime    The soln.anPart1 which is the neighbor to our original 
    %                soln.anPart1  
    %  randIndex1  The index of the customer that will be switched out from
    %                part 1 of the soln data structure
    %  randIndex2  The index of the customer that will be switched out from
    %                part 2 of the soln data structure. 
    %    nTemp     A temporary variable that holds the customer at index 
    %                randIndex1
    
    
    % Randomly pick two customer indices from anPart1 
    randIndex1 = -1;
    randIndex2 = -1;
    
    % Make sure that we don't pick the same two indices
    while randIndex1 == randIndex2
        randIndex1 = randi([2, length(soln.anPart1)-1]);
        randIndex2 = randi([2, length(soln.anPart1)-1]); 
    end
    
%     fprintf("Before switching: \n")
%     soln.anPart1
    
    % Switch their positions in soln.anPart1
    nTemp = soln.anPart1(randIndex1);
    soln.anPart1(randIndex1) = soln.anPart1(randIndex2);
    soln.anPart1(randIndex2) = nTemp; 
    
% %     fprintf("after switching: \n")
%     soln.anPart1
    
    % Return the new neighbor to the input solution
    sPrime = soln.anPart1;
    
end

function[ fTotalWaitTime ] = f( aafDistances, soln, k)
% This function f is our wait time function. In this case, we treat
%  the time = distance for our trucks and we calculate the time for the 
%  drones using a speed factor of 1.5

% INPUT
%   aafDistances    Array of floats containing the distances between custs.
%   soln            Data structure of solutions in parts 1, 2, 3 and 4 
%   k               Number of drones
% OUTPUT
%   fTotalWaitTime  Float of total wait time 
    

    % Variables
    %   alpha         Our speed factor for the drones. 
    %   aSoln         Current solution
    
    
    % Initialize constants
    alpha = 1.5;
    
    
    % Create temporary list of customers
    anCustomers = soln.anPart1;
    
    
    % Initialize total wait time
    fTotalWaitTime = 0;
    
    
    % Create vector arrival and departure times for drones and trucks
    aafDroneArrivalTime = zeros(k, length(aafDistances));
    aafTruckArrivalTime = zeros(1, length(aafDistances));
    aafTruckDepartureTime = zeros(1, length(aafDistances));
    
    
    soln;
    aafDistances;
    
    % Calculate the total wait time for the truck customers
    for iCustomerIndex = 2 : length(soln.anPart1) % Here iCustomerIndex = 1 & = length(soln.anPart1) are the nodes 
        iDrone = 1; % Initialize drone counter
        
        % Calculate the arrival time of the truck to this node        
        aafTruckArrivalTime(anCustomers(iCustomerIndex) + 1) = ...
            aafDistances( anCustomers(iCustomerIndex - 1) + 1, anCustomers(iCustomerIndex) + 1 );
        
%         "Put this arrival times in aafTruckDepartureTimes and vector and 
%          then update them if they have a drone approaching that same
%          node. So that if there is no drone going towards customer at
%          iCustomerIndex, then they already have something in their truck
%          arrival time vector. That is, we are assuming that no customer
%          has a drone delivering to them. 
        aafTruckDepartureTime(anCustomers(iCustomerIndex) + 1) = ...
            aafTruckArrivalTime(anCustomers(iCustomerIndex) + 1);
        

        
        for iReconveneIndex = 1 : length(soln.anPart4)
            
            % Check if we have an "X" there
            if soln.anPart4(iReconveneIndex) == -1
                iDrone = iDrone + 1;
            end
            
            % Get the arrival time of the drones
            if iCustomerIndex == soln.anPart4(iReconveneIndex) % This means there is a drone flying to this point
                a = aafDistances( anCustomers(soln.anPart3(iReconveneIndex)) + 1, soln.anPart2(iReconveneIndex) + 1);
                b = aafDistances( soln.anPart2(iReconveneIndex) + 1, anCustomers(soln.anPart4(iReconveneIndex)) + 1);
                
                aafDroneArrivalTime(iDrone, anCustomers(soln.anPart4(iReconveneIndex)) + 1) = (a + b)/alpha;
                
                %%%% WORKS TILL HERE %%%%
                
                % if Truck is there before the drone, it must wait
                if aafTruckArrivalTime(anCustomers(iCustomerIndex) + 1) < max (aafDroneArrivalTime( :, anCustomers(iCustomerIndex) + 1 ) )
                    aafTruckDepartureTime(anCustomers(iCustomerIndex) + 1) = ...
                        max (aafDroneArrivalTime( :, anCustomers(iCustomerIndex) + 1) );
                end
                
                % If the drone is there before the truck
                if aafTruckArrivalTime(anCustomers(iCustomerIndex) + 1) > max( aafDroneArrivalTime( :, iCustomerIndex) )
                    aafTruckDepartureTime(anCustomers(iCustomerIndex) + 1) = aafTruckArrivalTime( anCustomers(iCustomerIndex) + 1);
                end
                    
            end
            
        end
        
    end
    
    for fDepartureTime = aafTruckDepartureTime
        fTotalWaitTime = fTotalWaitTime + fDepartureTime;
    end
    
    aafTruckArrivalTime
    aafTruckDepartureTime
    aafDroneArrivalTime
    
    
        
end
