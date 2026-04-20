
% Example 1 — 4 × 5 grid
grid1([[r, e, d, e, e],
       [e, e, f, e, s],
       [d, e, e, e, d],
       [e, s, e, f, s]]).

% Example 2 — 3 × 3 grid
grid2([[r, e, s],
       [d, f, e],
       [e, s, e]]).


% ============================================================
% GRID UTILITIES
% ============================================================

% get_cell(+Grid, +R, +C, -Cell)
get_cell(Grid, R, C, Cell) :-
    nth1(R, Grid, Row),
    nth1(C, Row, Cell).

% grid_dims(+Grid, -Rows, -Cols)
grid_dims(Grid, Rows, Cols) :-
    length(Grid, Rows),
    nth1(1, Grid, FirstRow),
    length(FirstRow, Cols).

% find_cell(+Grid, +Type, -pos(R,C))
find_cell(Grid, Type, pos(R, C)) :-
    nth1(R, Grid, Row),
    nth1(C, Row, Type), !.

% find_all_cells(+Grid, +Type, -Positions)
find_all_cells(Grid, Type, Positions) :-
    findall(pos(R, C),
            ( nth1(R, Grid, Row), nth1(C, Row, Type) ),
            Positions).


% ============================================================
% MOVE LOGIC
% ============================================================

% try_move(+Grid, +pos(R,C), +Visited, -pos(NR,NC))
try_move(Grid, pos(R, C), Visited, pos(NR, NC)) :-
    grid_dims(Grid, Rows, Cols),
    member(DR-DC, [-1-0, 1-0, 0-(-1), 0-1]),
    NR is R + DR,
    NC is C + DC,
    NR >= 1, NR =< Rows,
    NC >= 1, NC =< Cols,
    get_cell(Grid, NR, NC, Cell),
    Cell \= d,
    Cell \= f,
    \+ member(pos(NR, NC), Visited).


% ============================================================
% HEURISTIC
% ============================================================

% manhattan(+pos(R1,C1), +pos(R2,C2), -D)
manhattan(pos(R1, C1), pos(R2, C2), D) :-
    DR is abs(R1 - R2),
    DC is abs(C1 - C2),
    D is DR + DC.

% manhattan_min(+Pos, +TargetList, -MinDist)
% Minimum Manhattan distance to any target; 999 if none.
manhattan_min(_, [], 999) :- !.
manhattan_min(Pos, Targets, Min) :-
    findall(D, ( member(T, Targets), manhattan(Pos, T, D) ), Ds),
    msort(Ds, [Min|_]).


% ============================================================
% GBFS — single trip
% ============================================================


gbfs_one(Grid, Start, PathVisited, Targets, SegPath, GoalPos) :-
    manhattan_min(Start, Targets, H0),
    H0 < 999,
    gbfs_loop(Grid,
              [h(H0, gbfs_node(Start, [Start], 0))],  % open list
              [],           % closed list (positions already in open)
              PathVisited,
              Targets,
              SegPath, GoalPos).

% --- Goal: current position is a target survivor ---
gbfs_loop(_, [h(_, gbfs_node(Pos, RevPath, _))|_], _,
          _, Targets, SegPath, Pos) :-
    member(Pos, Targets), !,
    reverse(RevPath, SegPath).

% --- Open list empty: no path to any target ---
gbfs_loop(_, [], _, _, _, _, _) :- !, fail.

% --- Expand best (lowest H) node ---
gbfs_loop(Grid, [h(_, gbfs_node(Pos, RevPath, Steps))|Rest],
          SearchClosed, PathVisited, Targets, SegPath, GoalPos) :-
    % AllVisited = this state's path  ∪  robot's full journey so far
    append(RevPath, PathVisited, AllVisited),
    findall(gbfs_node(NP, [NP|RevPath], NS),
            ( try_move(Grid, Pos, AllVisited, NP),
              \+ member(NP, SearchClosed),   % not already in open
              NS is Steps + 1 ),
            Children),
    findall(P, member(gbfs_node(P, _, _), Children), CPs),
    append(SearchClosed, CPs, SearchClosed2),
    insert_children(Children, Targets, Rest, Open2),
    gbfs_loop(Grid, Open2, SearchClosed2, PathVisited,
              Targets, SegPath, GoalPos).

% insert_children(+Nodes, +Targets, +OpenIn, -OpenOut)
% Compute H for each child and insert into the sorted open list.
insert_children([], _, Open, Open).
insert_children([N|Ns], Targets, OpenIn, OpenOut) :-
    N = gbfs_node(Pos, _, _),
    manhattan_min(Pos, Targets, H),
    insert_h(h(H, N), OpenIn, Open2),
    insert_children(Ns, Targets, Open2, OpenOut).

% insert_h(+h(H,N), +SortedList, -NewSortedList) — ascending by H
% Strict < for tie-breaking: equal-H nodes stay in generation order (FIFO).
insert_h(E, [], [E]).
insert_h(h(H, N), [h(H2, N2)|T], [h(H, N), h(H2, N2)|T]) :-
    H < H2, !.
insert_h(h(H, N), [h(H2, N2)|T], [h(H2, N2)|T2]) :-
    insert_h(h(H, N), T, T2).


% ============================================================
% MULTI-TRIP COLLECTION
% ============================================================


% Base case: no survivors left to collect
collect_all(_, _, _, [], PathAcc, PathAcc, 0, []) :- !.

collect_all(Grid, Pos, PathVisited, Remaining, PathAcc,
            FinalPath, TotalSteps, Collected) :-
    (   gbfs_one(Grid, Pos, PathVisited, Remaining,
                 SegPath, GoalPos)
    ->  % SegPath = [Pos, ..., GoalPos]; drop leading Pos (already in PathAcc)
        SegPath = [_|SegTail],
        append(PathAcc, SegTail, PathAcc2),
        length(SegTail, SegSteps),
        % Update PathVisited with the new segment (persistent no-revisit)
        append(PathVisited, SegTail, PathVisited2),
        remove_first(Remaining, GoalPos, Remaining2),
        collect_all(Grid, GoalPos, PathVisited2, Remaining2,
                    PathAcc2, FinalPath, RestSteps, RestCollected),
        TotalSteps is SegSteps + RestSteps,
        Collected = [GoalPos|RestCollected]
    ;   % No survivor reachable — stop here
        FinalPath  = PathAcc,
        TotalSteps = 0,
        Collected  = []
    ).

% remove_first(+List, +Elem, -Rest)
remove_first([H|T], H, T) :- !.
remove_first([H|T], E, [H|R]) :- remove_first(T, E, R).


% ============================================================
% OUTPUT
% ============================================================

print_path([pos(R, C)]) :- !,
    format('(~w,~w)~n', [R, C]).
print_path([pos(R, C)|Rest]) :-
    format('(~w,~w) -> ', [R, C]),
    print_path(Rest).

run_gbfs(Grid) :-
    find_cell(Grid, r, Start),
    find_all_cells(Grid, s, AllSurvivors),
    collect_all(Grid, Start, [Start], AllSurvivors,
                [Start], FinalPath, TotalSteps, Collected),
    length(Collected, NC),
    (   NC =:= 0
    ->  write('No path found.'), nl
    ;   write('Path found: '), print_path(FinalPath),
        format('Survivors rescued: ~w~n', [NC]),
        format('Number of steps: ~w~n', [TotalSteps])
    ).


% ============================================================
% MAIN — runs both examples
% ============================================================
:- initialization(main, main).

main :-
    write('=== Example 1 (4x5 Grid) ==='), nl,
    grid1(G1),
    run_gbfs(G1),
    nl,
    write('=== Example 2 (3x3 Grid) ==='), nl,
    grid2(G2),
    run_gbfs(G2).
