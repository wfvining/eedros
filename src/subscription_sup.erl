%%%-------------------------------------------------------------------
%%% @author Will Vining <wfvining@gmail.com>
%%% @copyright (C) 2018, Will Vining
%%% @doc
%%%
%%% @end
%%% Created : 31 May 2018 by Will Vining <wfvining@gmail.com>
%%%-------------------------------------------------------------------
-module(subscription_sup).

-behaviour(supervisor).

%% API
-export([start_link/0, subscribe/2]).

%% Supervisor callbacks
-export([init/1]).

-define(SERVER, ?MODULE).

-define(SUBSCRIBER_SPEC(Topic, Type), #{ id       => Topic,
                                         start    => {subscriber, start_link, [Topic, Type]},
                                         type     => worker,
                                         restart  => transient,
                                         shutdown => 5000 }).

%%%===================================================================
%%% API functions
%%%===================================================================

%%--------------------------------------------------------------------
%% @doc
%% Starts the supervisor
%% @end
%%--------------------------------------------------------------------
-spec start_link() -> {ok, Pid :: pid()} |
                      {error, {already_started, Pid :: pid()}} |
                      {error, {shutdown, term()}} |
                      {error, term()} |
                      ignore.
start_link() ->
    supervisor:start_link({local, ?SERVER}, ?MODULE, []).

subscribe(Topic, Type) ->
    supervisor:start_child(?SERVER, ?SUBSCRIBER_SPEC(Topic, Type)).

%%%===================================================================
%%% Supervisor callbacks
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Whenever a supervisor is started using supervisor:start_link/[2,3],
%% this function is called by the new process to find out about
%% restart strategy, maximum restart intensity, and child
%% specifications.
%% @end
%%--------------------------------------------------------------------
-spec init(Args :: term()) ->
                  {ok, {SupFlags :: supervisor:sup_flags(),
                        [ChildSpec :: supervisor:child_spec()]}} |
                  ignore.
init([]) ->

    SupFlags = #{strategy => one_for_one,
                 intensity => 1,
                 period => 5},

    {ok, {SupFlags, []}}.

%%%===================================================================
%%% Internal functions
%%%===================================================================
