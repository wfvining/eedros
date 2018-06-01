-module(ros_port).

-export([advertise/1]).

advertise(Topic) ->
    % let it fail, the server will handle it
    % process_flag(trap_exit, true),
    Port = open_port({spawn, "python raw_echo.py " ++ Topic}, [{packet, 2}]),
    advertise_loop(Port, Topic).

advertise_loop(Port, Topic) ->
    receive
        stop ->
            Port ! {self(), close},
            receive
                {Port, closed} ->
                    exit(normal)
            end;
        {Port, {data, Data}} ->
            eedros_server:publish(Topic, Data)
    end,
    advertise_loop(Port, Topic).
