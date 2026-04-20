
% Example 1 — 4 × 5 grid
grid1([[r, e, d, e, e],
       [e, e, f, e, s],
       [d, e, e, e, e],
       [e, s, e, f, e]]).

% Example 2 — 3 × 3 grid
grid2([[r, e, e],
       [d, f, e],
       [e, e, s]]).


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

% find_cell(+Grid, +Type, -pos(R,C))  [first occurrence, 1-indexed]
find_cell(Grid, Type, pos(R, C)) :-
    nth1(R, Grid, Row),
    nth1(C, Row, Type), !.


% ============================================================
% MOVE LOGIC
% ============================================================


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
% BFS SEARCH
% ============================================================

% bfs(+Grid, -Path, -Steps, -Battery)
bfs(Grid, Path, Steps, Battery) :-
    find_cell(Grid, r, Start),
    bfs_loop(Grid,
             [bfs_node(Start, [Start], 100)],   % open list (FIFO)
             [Start],                            % closed list
             Path, Steps, Battery).

% --- Goal check: current cell contains a survivor ---
bfs_loop(Grid, [bfs_node(Pos, RevPath, Bat)|_], _,
         Path, Steps, Bat) :-
    Pos = pos(R, C),
    get_cell(Grid, R, C, s), !,
    reverse(RevPath, Path),
    length(RevPath, L),
    Steps is L - 1.

% --- Battery exhausted: discard this node, continue ---
bfs_loop(Grid, [bfs_node(_, _, 0)|Rest], Closed,
         Path, Steps, Battery) :- !,
    bfs_loop(Grid, Rest, Closed, Path, Steps, Battery).

% --- Normal expansion ---
bfs_loop(Grid, [bfs_node(Pos, RevPath, Bat)|Rest], Closed,
         Path, Steps, Battery) :-
    % Generate all valid successors
    findall(bfs_node(NP, [NP|RevPath], NB),
            ( try_move(Grid, Pos, Closed, NP), NB is Bat - 10 ),
            Children),
    % Add child positions to closed list
    findall(P, member(bfs_node(P, _, _), Children), CPs),
    append(Closed, CPs, Closed2),
    % BFS: enqueue children at the TAIL (FIFO order)
    append(Rest, Children, Open2),
    bfs_loop(Grid, Open2, Closed2, Path, Steps, Battery).


% ============================================================
% OUTPUT
% ============================================================

% print_path(+Path)  — prints  (R,C) -> (R,C) -> ... (R,C)\n
print_path([pos(R, C)]) :- !,
    format('(~w,~w)~n', [R, C]).
print_path([pos(R, C)|Rest]) :-
    format('(~w,~w) -> ', [R, C]),
    print_path(Rest).

% run_bfs(+Grid)
run_bfs(Grid) :-
    (   bfs(Grid, Path, Steps, Battery)
    ->  write('Path found: '), print_path(Path),
        format('Number of steps: ~w~n', [Steps]),
        format('Remaining Battery: ~w%~n', [Battery])
    ;   write('No path found.'), nl
    ).


% ============================================================
% MAIN — runs both examples
% ============================================================
:- initialization(main, main).

main :-
    write('=== Example 1 (4x5 Grid) ==='), nl,
    grid1(G1),
    run_bfs(G1),
    nl,
    write('=== Example 2 (3x3 Grid) ==='), nl,
    grid2(G2),
    run_bfs(G2).
