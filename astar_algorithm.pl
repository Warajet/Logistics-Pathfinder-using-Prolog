%Station List
portList(['kualabelait', 'muara', 'kompongsom', 'phnompenh', 'cilacap', 'cirebon', 'jakarta', 'kupang', 'palembang', 'semarang', 'surabaya', 'ujungpandang,', 'bintulu', 'kotakinabalu', 'kuantan', 'kuching', 'kudat', 'labuan', 'lahaddatu', 'lumut', 'miri', 'pasirgudang', 'penang', 'portdickson', 'portklang', 'sandakan', 'sibu', 'tanjungpelepas', 'tawau', 'bassein', 'moulmein', 'yangon', 'batangas', 'cagayandeoro,', 'cebu', 'davao', 'iligano', 'iloilo', 'jolo', 'legaspi', 'manila', 'puertoprincesa,', 'sanfernando', 'subicbay', 'zamboanga', 'singapore', 'bangkok', 'laemchabang', 'pattani', 'phuket', 'sattahip', 'songkhla', 'sriracha', 'danang', 'haiphong', 'hochiminhcity', 'hongay', 'nhatrang']).

% Since the vertices are connected on undirected graph -> edges has 2
% ways
distance(X,Y,Z) :- neighbour_distance(X,Y,Z).
distance(X,Y,Z) :- neighbour_distance(Y,X,Z).
% A star algorithm consist of 2 sections f(n)= actual cost from
% source-> dest and h(n) = heuristic function

% A Star search Algorithm requires 2 function -> 1) get h(n)
% and 2) keep searching the nodes in undirected graph where
% h(n) = Estimated cost of the cheapest path from n to goal from source

aStarSearchAlgorithm(Source, Destination, Path, Cost):- getHeuristicValue(Source, Destination, HeuristicVal),
    search(Destination, [[Source,[Source], HeuristicVal]], [_, Path, Cost]).

search(Source_Station, [[Source_Station, Path, TotalCost] | T], [Source_Station, Path, TotalCost]) :- !.
search(Destination, [[Source_Station, Path, TotalCost] | T], Result) :- expand([Source_Station, Path, TotalCost], Destination, ExpandedNode),
    append(T, ExpandedNode, NewQueue),
    minsort(NewQueue, PriorityQueue),
    %write("\n\nQueue: "),
    write(NewQueue),
    %write("\nMin Sort: "),
    write(PriorityQueue), write("\n"),
    search(Destination, PriorityQueue, Result).

expand([Station,Path,_], Destination, Return) :- findall(X, distance(Station,X,_),NextStations),
						 checkPassedNode(NextStations, Path, [], NewNextStations),
						 createNode(NewNextStations, Destination, Path, [], Return).


checkPassedNode([],Path,NewStations, NewStations).
checkPassedNode([Station|T], Path, NewStations, Return) :-
    in(Station, Path) -> checkPassedNode(T, Path, NewStations, Return);
    append(NewStations,[Station],NNewStation),
    checkPassedNode(T, Path, NNewStation, Return).

in(Station, [PassedStation|Path]) :- Station == PassedStation -> !;in(Station,Path).

createNode([], Destination, Path, Nodes, Nodes).
createNode([Station|T], Destination,Path,Nodes, ExpandedNodes) :- append(Path, [Station], NewPath),
	append(Nodes, [[Station, NewPath, 0]], NewNodes),
	createNode(T, Destination, Path, NewNodes, Result),
	%write("RESULT: "),write(Result),write("\n"),
	getFn(Result, Destination, [], ExpandedNodes).


% f(n) = The summation of actual cost from the start node to node n +
% estimated cost of the cheapest path from n to the goal from source to
% destination
% f(n) = In other word = Estimated Cost of the cheapest solution thru n.
getFn([], Destination, Nodes, Nodes).
getFn([[Station, [FromPath|ToPath], TotalCost]|T], Destination, Nodes, Return) :-
    getGn(ToPath,FromPath, 0, Gn),
    getHeuristicValue(Station,Destination,Hn),
    Fn is Gn + Hn,
    % write("FN: ")
    write(Fn),
    append(Nodes, [[Station, [FromPath|ToPath], Fn]], NewNodes),
    getFn(T, Destination, NewNodes, Return).

% g(n) = get the actual cost for a path from the start node to node n.
getGn([], From, Acc, Acc).
getGn([To|T], From, Acc, Gn) :-	distance(From,To,StepCost),!,
    NewAcc is Acc + StepCost,
    getGn(T, To, NewAcc, Gn).

% Power function
power(X,Result) :- Result is X * X.

% h(n) = Estimated cost of the cheapest path from n to goal from source
% and destination location where Source(X1, Y1) -> Destination(X2, Y2).
getHeuristicValue(Start, Destination, HeuristicValue) :- location(_,Start,X1,Y1),
    location(_,Destination,X2,Y2),
    generateDistance(X1,Y1,X2,Y2,Distance),
    HeuristicValue is Distance/106.666667. %max deltaV train

% Calculates the raw distance using Haversine formula for Heuristic
% Distance calculations
generateDistance(X1, Y1, X2, Y2, ReturnV2) :- R is 6371e3, % metres
					      LatRadian1 is X1*pi/180,
					      LatRadian2 is X2*pi/180,
					      DifLat is (X2 - X1)*pi/180,
					      DifLon is (Y2 - Y1)*pi/180,
					      A1 is sin(DifLat/2) * sin(DifLat/2),
					      A2 is cos(LatRadian1) * cos(LatRadian2),
					      A3 is sin(DifLon/2) * sin(DifLon/2),
					      A is A1 + A2 * A3,
					      C is 2 * atan2(sqrt(A), sqrt(1-A)),
					      D is R*C,
					      ReturnV2 is D / 1000.

% Sorting
% setof function -> Sorting without duplicates -> Check if station,
% cost and total cost belongs to the queue or not
% member function -> use to check the membership property of the element

minsort(List, Return) :- setof([TotalCost,Station,Cost], member([Station,Cost,TotalCost], List), Result),
			 findall([Cost,TotalCost,Station], member([Station,Cost,TotalCost],Result),Return).

