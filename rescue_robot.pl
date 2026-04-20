% ============================================================
%  Rescue Robot Navigation — AI Assignment
%  Part 1 : BFS            (uninformed search)
%  Part 2 : Greedy Best-First Search  (informed search)
%
%  Run in SWI-Prolog:
%    swipl rescue_robot.pl
%  or query directly:
%    ?- part1.
%    ?- part2.
% ============================================================


% ============================================================
% 1. GRID DEFINITION
% ============================================================
% The environment is a list of rows; each row is a list of
% cell atoms:
%   r = robot start  |  s = survivor  |  d = debris
%   f = fire         |  e = empty
% Modify the grid here to test different scenarios.

grid([
    [e, e, d, e, s],   % row 0  (s at col 4)
    [e, f, e, d, e],   % row 1
    [r, e, e, e, e],   % row 2  (robot starts here)
    [e, d, f, e, s],   % row 3  (s at col 4)
    [e, e, e, e, e]    % row 4
]).


% ============================================================
% 2. GRID UTILITIES
% ============================================================

% get_cell(+Grid, +R, +C, -Cell)
get_cell(Grid, R, C, Cell) :-
    nth0(R, Grid, Row),
    nth0(C, Row, Cell).

% grid_dims(+Grid, -Rows, -Cols)
grid_dims(Grid, Rows, Cols) :-
    length(Grid, Rows),
    nth0(0, Grid, First),
    length(First, Cols).

% find_cell(+Grid, +Type, -pos(R,C)) — first occurrence
find_cell(Grid, Type, pos(R, C)) :-
    nth0(R, Grid, Row),
    nth0(C, Row, Type), !.

% find_all_cells(+Grid, +Type, -Positions)
find_all_cells(Grid, Type, Positions) :-
    findall(pos(R, C),
            ( nth0(R, Grid, Row), nth0(C, Row, Type) ),
            Positions).


% ============================================================
% 3. MOVE LOGIC
% ============================================================
% Four directions encoded as Row-Col deltas.
%
% try_move(+Grid, +pos(R,C), +Visited, -pos(NR,NC))
%   Generates one valid neighbouring cell:
%   - within grid bounds
%   - not debris (d) or fire (f)
%   - not already in Visited (no revisits)

try_move(Grid, pos(R, C), Visited, pos(NR, NC)) :-
    grid_dims(Grid, Rows, Cols),
    member(DR-DC, [-1-0, 1-0, 0-(-1), 0-1]),   % up/down/left/right
    NR is R + DR,
    NC is C + DC,
    NR >= 0, NR < Rows,
    NC >= 0, NC < Cols,
    get_cell(Grid, NR, NC, Cell),
    Cell \= d,
    Cell \= f,
    \+ member(pos(NR, NC), Visited).


% ============================================================
% 4. PART 1 : BFS  (Uninformed Search)
% ============================================================
%
% State representation: bfs_node(Pos, RevPath, Battery)
%   Pos      — current cell pos(R,C)
%   RevPath  — path from start stored newest-first
%              (reversed at output for readability)
%   Battery  — integer 0–100; start 100, -10 per step
%
% Open list  : FIFO queue — new nodes appended to the TAIL
%              (standard BFS level-by-level expansion)
% Closed list: flat list of visited positions
% Goal       : any cell whose value is 's'
%
% Because BFS expands nodes level by level (by step count),
% the first survivor reached is always the nearest one.

part1 :-
    write('=== PART 1: BFS (Uninformed Search) ==='), nl,
    grid(Grid),
    (   bfs(Grid, Path, Steps, Battery)
    ->  format('Path         : ~w~n', [Path]),
        format('Steps        : ~w~n', [Steps]),
        format('Battery left : ~w%~n', [Battery])
    ;   write('No path to any survivor found.'), nl
    ).

% bfs(+Grid, -Path, -Steps, -Battery)
bfs(Grid, Path, Steps, Battery) :-
    find_cell(Grid, r, Start),
    bfs_loop(Grid,
             [bfs_node(Start, [Start], 100)],  % initial open list
             [Start],                           % initial closed list
             Path, Steps, Battery).

% --- Goal check (must come first) ---
bfs_loop(Grid, [bfs_node(Pos, RevPath, Bat)|_], _,
         Path, Steps, Bat) :-
    Pos = pos(R, C),
    get_cell(Grid, R, C, s), !,
    reverse(RevPath, Path),
    length(RevPath, L),
    Steps is L - 1.

% --- Battery exhausted: discard node and continue ---
bfs_loop(Grid, [bfs_node(_, _, 0)|Rest], Closed,
         Path, Steps, Battery) :- !,
    bfs_loop(Grid, Rest, Closed, Path, Steps, Battery).

% --- Normal expansion ---
bfs_loop(Grid, [bfs_node(Pos, RevPath, Bat)|Rest], Closed,
         Path, Steps, Battery) :-
    % Generate all valid successor nodes
    findall(
        bfs_node(NP, [NP|RevPath], NB),
        ( try_move(Grid, Pos, Closed, NP), NB is Bat - 10 ),
        Children),
    % Add child positions to closed list (avoid re-insertion)
    findall(P, member(bfs_node(P,_,_), Children), CPs),
    append(Closed, CPs, Closed2),
    % BFS: enqueue children at the tail of the open list
    append(Rest, Children, Open2),
    bfs_loop(Grid, Open2, Closed2, Path, Steps, Battery).


% ============================================================
% 5. PART 2 : GREEDY BEST-FIRST SEARCH  (Informed Search)
% ============================================================
%
% Strategy: repeatedly run a single GBFS trip from the current
%   position to the nearest reachable survivor, collect it,
%   then repeat from the new position until no more survivors
%   can be reached.  The closed list persists across trips so
%   the robot never revisits any cell across the whole journey.
%
% State (within one trip): gbfs_node(Pos, RevPath, Steps)
%   Pos      — current cell pos(R,C)
%   RevPath  — path newest-first
%   Steps    — steps taken so far in this trip
%
% Open list: sorted ascending by heuristic H (min at head)
%   Each entry: h(H, gbfs_node(...))
%
% Heuristic H: Manhattan distance to the nearest remaining
%   (uncollected) survivor.  Lower H = more promising.

part2 :-
    write('=== PART 2: Greedy Best-First Search (Informed) ==='), nl,
    grid(Grid),
    find_cell(Grid, r, Start),
    find_all_cells(Grid, s, AllSurvivors),
    collect_all(Grid, Start, [Start], AllSurvivors,
                [Start], FinalPath, TotalSteps, Collected),
    length(Collected, NC),
    format('Path               : ~w~n', [FinalPath]),
    format('Steps              : ~w~n', [TotalSteps]),
    format('Survivors collected: ~w~n', [NC]).

% collect_all(+Grid, +Pos, +Closed, +Remaining,
%             +PathAcc, -FinalPath, -TotalSteps, -Collected)
%
% Iteratively uses gbfs_one to collect survivors one by one.
% PathAcc accumulates the full forward path across all trips.

collect_all(_, _, _, [], PathAcc, PathAcc, 0, []) :- !.
collect_all(Grid, Pos, Closed, Remaining, PathAcc,
            FinalPath, TotalSteps, Collected) :-
    (   gbfs_one(Grid, Pos, Closed, Remaining,
                 SegPath, EndPos, NewClosed)
    ->  % SegPath = [Pos, ..., EndPos]; drop leading Pos (already in PathAcc)
        SegPath = [_|SegTail],
        append(PathAcc, SegTail, PathAcc2),
        length(SegTail, SegSteps),
        remove_first(Remaining, EndPos, Remaining2),
        collect_all(Grid, EndPos, NewClosed, Remaining2,
                    PathAcc2, FinalPath, RestSteps, RestCollected),
        TotalSteps is SegSteps + RestSteps,
        Collected = [EndPos|RestCollected]
    ;   % No path to any remaining survivor — stop here
        FinalPath  = PathAcc,
        TotalSteps = 0,
        Collected  = []
    ).

% gbfs_one(+Grid, +Start, +Closed, +Targets,
%          -Path, -GoalPos, -NewClosed)
% Runs one GBFS trip from Start to the nearest cell in Targets.
% Fails if no target is reachable.
gbfs_one(Grid, Start, Closed, Targets, Path, GoalPos, NewClosed) :-
    manhattan_min(Start, Targets, H0),
    H0 < 999,     % at least one target exists
    gbfs_loop(Grid,
              [h(H0, gbfs_node(Start, [Start], 0))],
              Closed, Targets,
              Path, GoalPos, NewClosed).

% --- Goal check ---
gbfs_loop(_, [h(_, gbfs_node(Pos, RevPath, _))|_], Closed,
          Targets, Path, Pos, Closed) :-
    member(Pos, Targets), !,
    reverse(RevPath, Path).

% --- Open list empty: no path ---
gbfs_loop(_, [], _, _, _, _, _) :- !, fail.

% --- Expand best (lowest H) node ---
gbfs_loop(Grid, [h(_, gbfs_node(Pos, RevPath, Steps))|Rest],
          Closed, Targets, Path, GoalPos, NewClosed) :-
    findall(
        gbfs_node(NP, [NP|RevPath], NS),
        ( try_move(Grid, Pos, Closed, NP), NS is Steps + 1 ),
        Children),
    findall(P, member(gbfs_node(P,_,_), Children), CPs),
    append(Closed, CPs, Closed2),
    insert_children(Children, Targets, Rest, Open2),
    gbfs_loop(Grid, Open2, Closed2, Targets, Path, GoalPos, NewClosed).

% insert_children(+Nodes, +Targets, +OpenIn, -OpenOut)
% Computes H for each child and inserts into the sorted open list.
insert_children([], _, Open, Open).
insert_children([N|Ns], Targets, OpenIn, OpenOut) :-
    N = gbfs_node(Pos, _, _),
    manhattan_min(Pos, Targets, H),
    insert_h(h(H, N), OpenIn, Open2),
    insert_children(Ns, Targets, Open2, OpenOut).

% insert_h(+h(H,Node), +SortedList, -NewSortedList)
% Inserts into ascending order by H.
insert_h(E, [], [E]).
insert_h(h(H,N), [h(H2,N2)|T], [h(H,N), h(H2,N2)|T]) :-
    H =< H2, !.
insert_h(h(H,N), [h(H2,N2)|T], [h(H2,N2)|T2]) :-
    insert_h(h(H,N), T, T2).


% ============================================================
% 6. HEURISTIC FUNCTION
% ============================================================

% manhattan(+pos(R1,C1), +pos(R2,C2), -D)
% Manhattan distance between two grid positions.
manhattan(pos(R1,C1), pos(R2,C2), D) :-
    DR is abs(R1 - R2),
    DC is abs(C1 - C2),
    D is DR + DC.

% manhattan_min(+Pos, +Targets, -MinDist)
% Minimum Manhattan distance from Pos to any target.
% Returns 999 when Targets is empty (no survivors remain).
manhattan_min(_, [], 999) :- !.
manhattan_min(Pos, Targets, Min) :-
    findall(D, (member(T, Targets), manhattan(Pos, T, D)), Ds),
    msort(Ds, [Min|_]).


% ============================================================
% 7. UTILITIES
% ============================================================

% remove_first(+List, +Elem, -Rest)
% Removes the first occurrence of Elem from List.
remove_first([H|T], H, T) :- !.
remove_first([H|T], E, [H|R]) :- remove_first(T, E, R).


% ============================================================
% 8. ENTRY POINT
% ============================================================
:- initialization(main, main).

main :-
    part1, nl,
    part2.
