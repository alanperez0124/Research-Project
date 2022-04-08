function[] = TDRA( )
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
%     C0.x = (50 - -50)*rand(1, 4) + -50;
%     C0.y = (50 - -50)*rand(1, 4) + -50;
%     C0.x = [ 0 -10 0 0 ]; % we treat the first slot as the depot;
%     C0.y = [ 0 0 -10 -5 ];  % we treat the first slot as the depot;
%     C0.x = [ 0 randi([-10, 10], 1, 10) 0 ];
%     C0.y = [ 0 randi([-10, 10], 1, 10) 0 ];
    
%     C0.x = [ 0 -4 -7 -6 4 6 2 9 8 7 6 0 ];
%     C0.y = [ 0 1 -3 -8 -9 -3 10 9 -2 -8 -8 0];
    
% %          0  1  2  3  4  0   Customer ID
% %          1  2  3  4  5  6   Indices
%     C0.x = [ 0  8  6 -7  1  0 ];
%     C0.y = [ 0 -2 -3 -3  2  0 ];

%              0  1  2  3  4  5  6  0  Customer ID
%              1  2  3  4  5  6  7  8  Indices
    C0.x = [ 0 -2 -3  2  2  7 -4  0 ]; 
    C0.y = [ 0  4  1  4  3  1  3  0 ]; 

%%%%%%%%%%%%%%%%%%%%%%%%Testing ellipse fitting %%%%%%%%%%%%%%%%%%%%%%%%
% Slanted ellipse
%     C0.x = [ 1, 2, 3, 4, 4.5, 5, 5.5, 6, 6, 5.7, 5.5, 5, 4, 3, 1, ...
%              -1, -3, -6, -7, -7.5, -7, -6, ...
%              -3, -2, -1, -4, -5];
%     C0.y = [ 7, 6, 5, 4, 3, 2, 0.5, -1, -1.5, -3.5, -4, -5, -5.3, -5.2, -4.5,...
%              8, 8, 7, 6, 4, 2, 0, ...
%              -2.5, -3, -3.5, -1.5, -1];
    

% Ellipse along x axis; This works well 
%     C0.x = [ -3, -2.5, -2, -1.5, -1, -.5, 0, .5, 1, 1.5, 2, 2.5, 3, -3, -2.5, -2, -1.5, -1, -.5, 0, .5, 1, 1.5, 2, 2.5, 3 ];
%     C0.y = [ 0.54, 1.06, 1.34, 1.52, 1.64, 1.71, 1.73, 1.71, 1.64, 1.54, 1.34, 1.06, .54, -0.54, -1.06, -1.34, -1.52, -1.64, -1.71, -1.73, -1.71, -1.64, -1.54, -1.34, -1.06, -.54 ];

% Ellipse along y axis
%     C0.x = [ 0 [ -2 : 0.5 : 2 ] [ -2 : 0.5 : 2 ] 0]
%     C0.y = [0 sqrt( 1 - (C0.x(1 : 9).^2)/4 ) [ -1*sqrt( 1 - (C0.x(10 : 18).^2)/4 ) ] 0 ]
    



    % Plot the customer locations
%     hold on
%     figure( 1 );
%     plot(0,0,'b*', 'MarkerSize', 12)
%     plot(C0.x, C0.y, 'bo')
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
    s = simulatedAnnealing(c, aafDistances, soln, T0, TFinal, Beta_SA, Imax, u);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Test plotting our path with drones
    % THis is correct btw
%     s.anPart1 = [ 0 4 2 3 0 ];
%     s.anPart3 = [ 2 -1 ];   % These indices refer to the cell number in part 1, so this is wrong. 
%     s.anPart2 = [ 1 -1 ];   % visits customer 1
%     s.anPart4 = [ 3 -1 ];   % returns to index 2 of s.anPart1 (which is customer 3)
% 
%     s.anPart1 = [ 0 4 3 1 2 0 ];    % MOST RECENT TEST
%     s.anPart3 = [ 1 -1 4 ];
%     s.anPart2 = [ 5 -1 6 ];
%     s.anPart4 = [ 3 -1 5 ]; 
    
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
    fs_best = f(aafDistances, s_best, u);
    
    
    % Elliptical stuff
    
    solnOut = ellipticalCustomerAssignment( s_best, C0, u);
    plot_route( C0, solnOut)
    
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

function[ solnOut ] = ellipticalCustomerAssignment( solnIn, C0, k )
% The ellipticalCustomerAssignment heuristic function will take in a soln
% and then will return another solution. 

% INPUT
%  solnIn      This is the input solution
%  C0          The locations (coordinates) of the customers
%  k           Number of drones
% OUTPUT
%  solnOut     This is the output solution

    % VARIABLES
    %    D        The design matrix
    %    C        The constraint matrix
    %    S        Scatter matrix
    %    V        Matrix with columns being the eigenvectors
    %    D        Matrix where diagonals are eigenvalues
    %    g        Anonymous function for distance from point to conic
    %    h        Anonymous function for distance between two points
    % aVectorScaled  The parameters that create our ellipse
    % anSortedTruckCustomers   Vector of truck customers sorted by distance
    %                          to ellipse
    % aafDistances  Get the distances between all the points
    
    
    % Create matrix of distances
    aafDistances = calculateDistances(C0.x, C0.y);
    
    % Assign initial variables 
    solnOut = solnIn; 
    fSolnOut = f( aafDistances, solnIn, k);
    
    % Fit an ellipse to C0 
    % First create our design matrix
    D = zeros(length(C0.x) - 1, 6);
    for iRow = 1 : length(C0.x) - 1
        D( iRow, 1 ) = (C0.x(iRow))^2;
        D( iRow, 2 ) = C0.x(iRow) * C0.y(iRow);
        D( iRow, 3 ) = (C0.y(iRow))^2;
        D( iRow, 4 ) = C0.x(iRow);
        D( iRow, 5 ) = C0.y(iRow);
        D( iRow, 6 ) = 1;
        

    end
    
    
    % Next our constraint matrix
    C = zeros(6); 
    C(1, 3) = 2;
    C(3, 1) = 2;
    C(2, 2) = -1; 
    
    % Create scatter matrix
    S = D' * D;
    
    % Find our eigenvalues and eigenvectors
    [ eigenvector, eigenvalue ] = eig( S, C );
    
    % Find the smallest eigenvalue
    fMinDiag = inf;
    for iDiagonal = 1 : 6
        if eigenvalue(iDiagonal, iDiagonal) > 1e-6 && eigenvalue(iDiagonal, iDiagonal) < fMinDiag
            iMinDiag = iDiagonal;
            fMinDiag = eigenvalue(iDiagonal, iDiagonal);
        end
    end
    
    
    % Scale the parameters so that a'C*a = 1
    aVector = eigenvector( :, iMinDiag );
    fScaling = aVector' * C * aVector;
    aVectorScaled = aVector / sqrt(fScaling);
        
    
    % This function gives us the algebraic distance of any point (x, y)
    % from the conic. 
    g = @( a, x, y) (a(1)*x.^2 + a(2).*x.*y + a(3)*y.^2 + a(4)*x + a(5)*y + a(6));
    
    
    % Create functions from the quadratic formulas    
    fplus  = @(a, x) ( -(a(2)*x + a(5)) + sqrt( (a(2)*x + a(5)).^2 - 4*a(3)*(a(1)*x.^2 + a(4)*x + a(6)) ) ) / (2*a(3));
    fminus = @(a, x) ( -(a(2)*x + a(5)) - sqrt( (a(2)*x + a(5)).^2 - 4*a(3)*(a(1)*x.^2 + a(4)*x + a(6)) ) ) / (2*a(3));
    
    
    % Create array of points
    afXData = [ min(C0.x) - 10 : 0.01 : max(C0.x) + 10 ];
    % Check to see if the x and y are imaginary
    counter = 1;
    for x = afXData
        if isreal(fplus( aVectorScaled, x )) && fplus( aVectorScaled, x) ~= 0
            afXGoodData(counter) = x;
            afYDataPositive(counter) = fplus( aVectorScaled, x ); 
        end
        
        if isreal(fminus( aVectorScaled, x )) && fminus( aVectorScaled, x) ~= 0
            afYDataNegative(counter) = fminus( aVectorScaled, x );
        end
        
        if isreal(fminus( aVectorScaled, x )) && isreal(fplus( aVectorScaled, x ))
            counter = counter + 1; 
        end
        
        
    end
               
    
    % Plot the points
    figure(12345)
    hold on; 
    plot(C0.x, C0.y, 'bo')
    plot(afXGoodData, afYDataPositive, 'r-')
    plot(afXGoodData, afYDataNegative, 'r-')

    
    % Given a customer point (xi, yi), find the point (x, y) on an ellipse
    % that minimizes the distance between (xi, yi) and (x, y)
    
    % Set the max number of iterations, the threshold
    % for convergence
    nMax = 30; 
    fThreshold = 10^(-6);
    
    % Hard code the function and its jacobian
    h = @(x, y, lambda, xi, yi, a) ...
        [ 2*x - 2*xi + 2*a(1)*x*lambda + a(2)*lambda*y + a(4)*lambda; 
          2*y - 2*yi + a(2)*lambda*x + 2*a(3)*lambda*y + a(5)*lambda; 
          a(1)*x^2 + a(2)*x*y + a(3)*y^2 + a(4)*x + a(5)*y + a(6) ];

    h_jacobian = @(x, y, lambda, xi, yi, a) ...
        [ 2 + 2*a(1)*lambda, a(2)*lambda, 2*a(1)*x + a(2)*y + a(4); 
          a(2)*lambda, 2 + 2*a(3)*lambda, a(2)*x + 2*a(3)*y + a(5); 
          2*a(1)*x + a(2)*y + a(4), a(2)*x + 2*a(3)*y + a(5), 0 ];
      
   % Initialize ellipse points
   afEllipse.x = [];
   afEllipse.y = [];
   
   % Initialize vector of distances
   afDistanceFromEllipse = zeros(1, length(C0.x)-2); 
    
   %Instead of iterating through the length of the arrays, iterate through   
   for nCustomer = solnIn.anPart1(2 : end - 1)
       % Set initial guess
       %                  x                y        lambda
       afGuess = [ C0.x(nCustomer + 1); C0.y(nCustomer + 1); 0 ]; 
       
       
       % Get the unknowns
       [ ae_vals, j, aUnknowns ] = ... 
           newtons_method( nMax, afGuess, fThreshold, h, h_jacobian, aVectorScaled, C0, nCustomer+1);
       
       % Plot the point on the ellipse
       plot(aUnknowns(1, j-1), aUnknowns(2, j-1), 'ko')
       
       % Store the ellipse point
       afEllipse.x = aUnknowns(1, j-1);
       afEllipse.y = aUnknowns(2, j-1); 
       
       % Calculate the distance between customers and closest pt on ellipse
       afDistanceFromEllipse( nCustomer ) = sqrt( (afEllipse.x(end) - C0.x(nCustomer+1))^2 + ...
           (afEllipse.y(end) - C0.y(nCustomer+1))^2 );
       
       % Plot those thangs 
       plot( [ C0.x(nCustomer+1); afEllipse.x ], [C0.y(nCustomer+1); afEllipse.y], 'b-' )
%        [ C0.x(nCustomer); afEllipse.x ]
% %        for i = 1 : length(C0.x)
% %            plot( [ C0.x(nCustomer+1); afEllipse.x(nCustomer) ], [C0.y(nCustomer+1); afEllipse.y(nCustomer)], 'b-' )
% %        end
% %        hold off; 
             
              % Final Output Lines
%        fprintf("\n")
%         fprintf("Final Output Line for Part (a)\n")
%         fprintf("Current k    |      e^(k+1)     |      x^(k+1)\n" );
%         fprintf("  %4d       |   %10.10f   |   [ %2.6f ; %2.6f; %2.6f ]   \n", k-2, ae_vals(k-1), aUnknowns(1, k-1), aUnknowns(2, k-1), aUnknowns(3, k-1))
       
   end
    
   hold off;
   
   
   % Sort the customer in descending order from distance to the ellipse
   afDistanceFromEllipse;
   [ afSortedDistances, anSortedTruckCustomers ] = sort( afDistanceFromEllipse, 2, "descend" );

   afSortedDistances;
   anSortedTruckCustomers;
   
   
   % Get the distance matrix (the distances between all the points)
   aafDistances = calculateDistances(C0.x, C0.y); 
   
   
    % Now we start looping until no feasible position is available for
    % customers in the list for re-insertion in UAV routes
%     feasible = 1;  %
    temp_counter = 0;     % this temporary counter will eventually be removed, 
                          % it is taking place of the feasibility thing
           
    bDone = 0;


    
% if for one iteration of the repeat loop, doesn't improve. 


    while ~bDone 
        % Initialize the "previous" best waiting time
        fTotalWaitingTimePrev = fSolnOut; 


        for i = 1 : length(anSortedTruckCustomers)
            % Initialize feasibility for truck insertion
            bFeasible = 1; 
            
            % Store the index of customer i in anSortedTruckCustomers
            jCust = anSortedTruckCustomers(i);  % customer index at index i in anSortedTruckCustomers
            
            % Select customer c_j in the list & remove it from truck route
            solnRemovedCustomer = soln_remove_truck_customer(solnIn, jCust);


%             %%%%% TEMPORARY STUFF REMOVE AFTER TESTING %%%%%
%             solnIn.anPart1 = [0 10 9 8 7 3 5 6 0];
%             solnIn.anPart3 = [ 1 6 -1 2 3 6];
%             solnIn.anPart2 = [11 1 -1 12 4 2];
%             solnIn.anPart4 = [3 7 -1 3 6 7];
%             solnIn
%             jCust = 8  % This is the customer we will be removing
%             %%%%% TEMPORARY STUFF REMOVE AFTER TESTING %%%%%
            
            % Check all potential positions in truck route
            for iTruckStop = 2 : length(solnIn.anPart1)
                % Insert the customer in the truck route
                insertedTruckSoln = ...
                    soln_insert_truck_customer(solnRemovedCustomer, jCust, iTruckStop);
                
                
                % Check feasibility 
                % Totally checking feasibility, yep looks super great
                %%%%% REMOVE AFTER TESTING %%%%%
                bFeasible = 1; 
                %%%%% REMOVE AFTER TESTING %%%%%              
                
                % Calculate the total waiting time
                fTruckInsertionWaitingTime = f( aafDistances, insertedTruckSoln, k ); 

                % Compare this s to our original s_out
                if bFeasible && (fTruckInsertionWaitingTime < fSolnOut)
                    solnOut = insertedTruckSoln; 
                    fSolnOut = fTruckInsertionWaitingTime; 
                end
            end
            

            % Intialize feasibility for inserting customer into drone
            bFeasible = 1; 
            
            % Check all potential positions in drone route
            for iDrone = 1 : k
                for iStopLeave = 1 : length(solnRemovedCustomer.anPart1)
                    for iStopReturn = iStopLeave : length(solnRemovedCustomer.anPart1)
                        % Insert the customer in the truck route
                        insertedDroneSoln = ... 
                            soln_add_drone_customer(solnRemovedCustomer,...
                            jCust, iDrone, iStopLeave, iStopReturn);

                        % Check feasibility
                        % Wow still super feasible 

                        % Calculate the total waiting time
                        fDroneInsertionWaitingTime = f( aafDistances, insertedDroneSoln, k); 

                        % Compare this to the solnOut
                        if bFeasible && (fDroneInsertionWaitingTime < fSolnOut)
                            solnOut = insertedDroneSoln; 
                            fSolnOut = fDroneInsertionWaitingTime; 
                        end

                        
                    end
                end
            end

        end 


        % Remove the customer from the list
        anSortedTruckCustomers(1) = []; 


        % right here
        if fTotalWaitingTimePrev <= fSolnOut 
            bDone = 1; 
        end
    end

end

function[ ae_vals, k, aUnknowns ] = newtons_method( nMax, afGuess, fThreshold, h, h_jacobian, a, C0, iCustomer )
% Newton's method will approximate the roots of our function h. For this
% particular problem, h is the gradient of our distance formula. 
% Input
%   nMax               The maximum number of iterations
%   afGuess            Our initial guess
%   fThreshold         The threshold for convergence
%   h                  The function we are trying to find the root of 
%   h_jacobian         The jacobian of function h
%   aVectorScaled      The paramters we found to create our ellipse
%   C0                 Our set of customers
%   iCustomer          The current customer we are at
% Output
%   ae_vals            Array of error values
%   k                  Number of iterations
%   aUnknowns          Array of unknowns we are trying to find

    % Local Variables
    %  k           Number of iterations it took to converge
    %  e           The error value
    %  xi          x coordinate of the customers
    %  yi          y coordinate of the customers
    
    % Initialize k values and error value e
    k = 1; 
    e = 99;   % absurd error value
    
    % Create variables for the x and y coordinates of the customers
    xi = C0.x(iCustomer); 
    yi = C0.y(iCustomer); 
    
    % Initialize vector to hold the updating x, y, and lambda values and
    % vector for e values
    aUnknowns = zeros(3, nMax); 
    ae_vals = zeros(1, nMax); 
    aUnknowns(:, 1) = afGuess; 
    ae_vals(1) = e; 
    
    % Run Newton's Method for the vector case
    while k < nMax + 2 && ae_vals(k) > fThreshold
        aUnknowns(:, k+1) = aUnknowns(:, k) - ... 
            h_jacobian(aUnknowns(1, k), aUnknowns(2, k), aUnknowns(3, k), xi, yi, a) \ ...
            h(aUnknowns(1, k), aUnknowns(2, k), aUnknowns(3, k), xi, yi, a ); 
        
        % Calculate the error
        ae_vals(k+1) = norm( aUnknowns(:, k+1) - aUnknowns(:, k) );
        
        % Print k, the error and the value
%         fprintf("Current k    |      e^(k+1)     |      x^(k+1)\n" );
%         fprintf("  %4d       |   %10.10f   |   [ %2.6f ; %2.6f; %2.6f ]   \n", k-1, ae_vals(k+1), aUnknowns(1, k+1), aUnknowns(2, k+1), aUnknowns(3, k+1))
        
        % Update k 
        k = k + 1; 
    end
end

function[ solnOut ] = soln_remove_truck_customer(solnIn, jCust)
% The soln_remove_truck_Customer function takes in a solution structure and
% the customer j that we will be removing from the truck route. It will
% first loop over soln.anPart1 to find jCust. Then it will "remove him" by
% esentially shifting everyone in solnIn.anPart1 after him down by 1. 
% Input
%  solnIn     The solution we are currently working with
%  jCust      The customer we will be removing from part 1 of solnIn
% Output
%  solnOut    The resulting solution

    % Local variables
    %  iCustomer     counter variable to keep track of the customer index
    %  
    
    % Loop over soln.anPart1 to find jCust
    for iCustomer = 1 : length(solnIn.anPart1)
        if solnIn.anPart1(iCustomer) == jCust
            jCustIndex = iCustomer; 
        end
    end
    
    % Remove jCust and shift everyone down
    solnIn.anPart1(jCustIndex) = [];
    solnIn.anPart1;

    % Shift things in part 3 and part 4 that are after the removal slot
    % down by 1
    nDroneCounter = 1; 
    for nIndex = 1 : length(solnIn.anPart3)
        if solnIn.anPart3(nIndex) < 0
            nDroneCounter = nDroneCounter + 1;
            
        else
            if solnIn.anPart3(nIndex) > jCustIndex  % "if it comes after"
                solnIn.anPart3(nIndex) = solnIn.anPart3(nIndex) - 1;  
            end
            
            if solnIn.anPart4(nIndex) > jCustIndex
                solnIn.anPart4(nIndex) = solnIn.anPart4(nIndex) - 1; 
            end
        end
    end
    
    % Return the solnOut
    solnOut = solnIn; 
  
end

function[ solnOut ] = soln_insert_truck_customer(solnIn, jCust, iStop)
% The soln_insert_truck_customer method will take in the solution, the
% jCustomer we will be inserting, and the place where the truck will be
% inserted. 
% Inputer
%    solnIn    The input solution
%    jCust     the customer that will be inserted
%    iStop     Where the truck will be placed

    % Local Variables
    %
    
    % Initialize the out solution 
    solnOut = solnIn;
    
    % First insert values 1 to iStop - 1
    solnOut.anPart1 = solnIn.anPart1(1 : iStop - 1);
    
    % Next insert jCust
    solnOut.anPart1(iStop) = jCust; 
    
    % Next insert the remaining values iStop to the end of the array
    solnOut.anPart1 = [ solnOut.anPart1 solnIn.anPart1(iStop : end)]; 
    
    % Update anPart3 and anPart4
    nDroneCounter = 1; 
    for nIndex = 1 : length(solnIn.anPart3)
        if solnIn.anPart3(nIndex) < 0
            nDroneCounter = nDroneCounter + 1; 
        else
            if solnIn.anPart3(nIndex) >= iStop
                solnOut.anPart3(nIndex) = solnIn.anPart3(nIndex) + 1; 
            end
            
            if solnIn.anPart4(nIndex) >= iStop
                solnOut.anPart4(nIndex) = solnIn.anPart4(nIndex) + 1; 
            end
        end
    end             
end

function[ solnOut ] = soln_add_drone_customer(solnIn, jCust, iDrone, iStopLeave, iStopReturn)
% soln_add_drone_customer will take in the current solution data structure,
% the customer we will be including, the drone that will be delivering to
% it as well as where the drone will be departing from and arriving to. 
% Input
%  solnIn       Current solution
%  jCust        Customer we are inserting
%  iDrone       The drone that will be delivering to that customer
%  iStopLeave   The index from which the drone will be leaving (based off
%                of the indices from soln.anPart1) 
%  iStopReturn  The index where the drone will be returning (also based off
%                of the indices from soln.anPart1)

% Output
%  solnOut      The resutling solution. 

    % Local variable
    % iPart3Drone     The drone we are currently looking at in part 3
    
    
    % Find the drone we are looking for in solnIn.anPart3
    iPart3Drone = 1;                   
    nDroneCounter = 1;
    
    % Initialize the solnOut
    solnOut = solnIn; 
    
    solnIn;
    while (nDroneCounter ~= iDrone)
        % If we hit a separator (-1) increment the drone
        if (solnIn.anPart3(iPart3Drone) == -1)
            nDroneCounter = nDroneCounter + 1; 
        end
        iPart3Drone = iPart3Drone + 1; 
    end
    
    
    % Look for where iStopLeave fits in solnIn.anPart3 (start from
    % iPart3Drone)
    bPlaced = 0;
    iLeavePlacement = iPart3Drone; 
    while (~bPlaced)        
        % In the case that we want to assign drone n a customer, but there
        % are currently no drone n customers
        if iLeavePlacement > length(solnIn.anPart3)
            % Insert placeholder slot
            solnOut.anPart3(end + 1) = 0;
            solnOut.anPart2(end + 1) = 0; 
            solnOut.anPart4(end + 1) = 0; 
        end
                    
        if (solnOut.anPart3(iLeavePlacement) < iStopLeave)
            % Then we place it in index iLeavePlacement
            bPlaced = 1; 
            
        else
            % We iterate through
            iLeavePlacement = iLeavePlacement + 1; 

        end
    end
    
    
    % Shift oclumns in Part 3 up by 1
    % Shift the columns at & abova where it goes to the right by 1
    solnOut.anPart3 = solnIn.anPart3(1 : iLeavePlacement - 1); 
    
    % Insert the index it will be leaving from
    solnOut.anPart3(iLeavePlacement) = iStopLeave;
    
    % Insert the remaining values iStopLeave to the end of the array
    solnOut.anPart3 = [ solnOut.anPart3 solnIn.anPart3(iLeavePlacement : end) ];
    
    % ----
    % Shift columns in Part 2 up by 1
    % Shift the columns at & abova where it goes to the right by 1
    solnOut.anPart2 = solnIn.anPart2(1 : iLeavePlacement - 1);
    
    % Insert the customer jCust
    solnOut.anPart2(iLeavePlacement) = jCust; 
    
    % Insert the remaining values iStopLeave to the end of the array
    solnOut.anPart2 = [ solnOut.anPart2 solnIn.anPart2(iLeavePlacement : end) ]; 
    
    
    % ----
    % Shift the columns in Part 4 up by 1
    % Shift the columns at & above where it goes to the right by 1
    solnOut.anPart4 = solnIn.anPart4(1 : iLeavePlacement - 1);

    % Insert the customer return index
    solnOut.anPart4( iLeavePlacement ) = iStopReturn; 
    
    % Insert the remaining values iStopLeave to the end of the array
    solnOut.anPart4 = [ solnOut.anPart4 solnIn.anPart4(iLeavePlacement : end) ]; 
    
    
    solnOut;      
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
    iPrevious = 1; 
    for iCustomerIndex = 2 : length(soln.anPart1) % Here iCustomerIndex = 1 & = length(soln.anPart1) are the nodes 
        iDrone = 1; % Initialize drone counter
        
        % Calculate the arrival time of the truck to this node        
        aafTruckArrivalTime(anCustomers(iCustomerIndex) + 1) = ...
            aafDistances( anCustomers(iCustomerIndex - 1) + 1, anCustomers(iCustomerIndex) + 1 ) ...
          + aafTruckArrivalTime( anCustomers(iCustomerIndex - 1) + 1);
        
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
                elseif aafTruckArrivalTime(anCustomers(iCustomerIndex) + 1) > max( aafDroneArrivalTime( :, iCustomerIndex) )
                    % If the drone is there before the truck
                    aafTruckDepartureTime(anCustomers(iCustomerIndex) + 1) = aafTruckArrivalTime( anCustomers(iCustomerIndex) + 1);
                end
                    
            end
            
        end
       iPrevious = iPrevious + 1;  
    end
    
    
    % Get the total waiting time 
    fTotalWaitTime = aafTruckDepartureTime( soln.anPart1(end - 1) + 1 );       
end
