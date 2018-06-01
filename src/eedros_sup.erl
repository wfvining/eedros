%%%-------------------------------------------------------------------
%% @doc eedros top level supervisor.
%% @end
%%%-------------------------------------------------------------------

-module(eedros_sup).

-behaviour(supervisor).

%% API
-export([start_link/1]).

%% Supervisor callbacks
-export([init/1]).

-define(SERVER, ?MODULE).
-define(SERVER_ID, eedros_server).

%%====================================================================
%% API functions
%%====================================================================

start_link(Robots) ->
    supervisor:start_link({local, ?SERVER}, ?MODULE, [Robots]).

%%====================================================================
%% Supervisor callbacks
%%====================================================================

%% Child :: {Id,StartFunc,Restart,Shutdown,Type,Modules}
init([Robots]) ->

    ServerSpec = #{ id       => ?SERVER_ID,
                    start    => {?SERVER_ID, start_link, [self()]},
                    restart  => permanent,
                    shutdown => 20000,
                    type     => worker,
                    modules  => [eedros_server]},

    {ok, { {one_for_all, 0, 1}, [ServerSpec] }}.

%%====================================================================
%% Internal functions
%%====================================================================
